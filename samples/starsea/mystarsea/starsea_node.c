#include "stdio.h"
#include "string.h"
#include "soc_osal.h"
#include "MQTTClient.h"
#include "i2c.h"
#include "securec.h"
#include "gpio.h"
#include "errcode.h"
#include <cJSON.h>
#include "osal_debug.h"
#include "cmsis_os2.h"
#include "sht20/hal_bsp_sht20.h"
#include "rgb_led/hal_bsp_aw2013.h"
#include "wifi/wifi_connect.h"
#include "mqtt/mqtt_connect.h"
#include "oled1306/hal_bsp_ssd1306.h"
#include "nfc/hal_bsp_nfc.h"
#include "ap3216/hal_bsp_ap3216.h"
#include "sle_server_adv/sle_server_adv.h"
#include "sle_server_adv/sle_server.h"
#include "app_init.h"

osMutexId_t Mutex_ID;             // 定义互斥锁 ID
osThreadId_t mqtt_ID;             // mqtt订阅数据任务
osThreadId_t sht20_ID;            // sht20任务的id
osThreadId_t oled_ID;             // oled任务的id
osThreadId_t nfc_ID;              // nfc任务的id
osThreadId_t ap3216_ID;           // ap3216任务的id
osThreadId_t controlActuators_ID; // ap3216任务的id
osMutexId_t i2c_mutex = NULL;
typedef struct {
    float temperature; // 温度
    float humidity;    // 湿度
} temp_humidity_data_t;

#define DELAY_TIME_MS 200 // 延时
extern MQTTClient client;

#define RGB_ON 45
#define RGB_OFF 0
// 全局温湿度
float temp = 0;
float humi = 0;
uint16_t als = 0;                                 // 光照强度传感器
uint16_t ps = 0;                                  //  接近传感器
bool lock_flag = false;                           // nfc状态变量
uint8_t *ndefBuff;                                // nfc接收数据变量
unsigned int g_msg_rev_size = sizeof(msg_data_t); // 消息接收大小
unsigned long g_msg_queue = 0;                    // 消息队列句柄
bool mqtt_LED = false;
bool temp_lx_LED = false;
bool mqtt_DC = false;
bool temp_lx_DC = false;
bool nfc_DC = false;
extern bool sle_DC;
bool mqtt_Buzzer = false;
bool temp_lx_Buzzer = false;
extern bool sle_Buzzer;

// mqtt发送数据
int mqtt_publish(const char *topic, char *g_msg)
{
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int ret = 0;
    pubmsg.payload = g_msg;
    pubmsg.payloadlen = (int)strlen(g_msg);
    pubmsg.qos = 1;
    pubmsg.retained = 0;
    ret = MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("mqtt_publish failed\r\n");
    }
    printf("mqtt_publish(), the payload is %s, the topic is %s\r\n", g_msg, topic);
    return ret;
}
// 自定义消息处理函数
void handle_message(const char *topic, const char *msg)
{
    printf("收到消息！主题: %s, 内容: %s\r\n", topic, msg);
    cJSON *root = cJSON_Parse(msg);
    if (root == NULL) {
        printf("JSON解析失败\r\n");
        return;
    }
    // 获取led字段
    cJSON *led_item = cJSON_GetObjectItem(root, "led");
    if (led_item != NULL && cJSON_IsString(led_item)) {
        const char *led_value = led_item->valuestring;

        if (strcmp(led_value, "greenon") == 0) {
            AW2013_Control_RGB(RGB_OFF, RGB_ON, RGB_OFF);
            printf("绿灯已开启\r\n");
        } else if (strcmp(led_value, "redon") == 0) {
            AW2013_Control_RGB(RGB_ON, RGB_OFF, RGB_OFF);
            printf("红灯已开启\r\n");
        } else if (strcmp(led_value, "blueon") == 0) {
            AW2013_Control_RGB(RGB_OFF, RGB_OFF, RGB_ON);
            printf("蓝灯已开启\r\n");
        } else if (strcmp(led_value, "rgboff") == 0) {
            AW2013_Control_RGB(RGB_OFF, RGB_OFF, RGB_OFF);
            printf("所有RGB灯已关闭\r\n");
        } else if (strcmp(led_value, "on") == 0) {
            mqtt_LED = true;
        } else if (strcmp(led_value, "off") == 0) {
            mqtt_LED = false;
        }
    }

    // 蜂鸣器
    cJSON *buzzer_item = cJSON_GetObjectItem(root, "buzzer");
    if (buzzer_item != NULL && cJSON_IsString(buzzer_item)) {
        const char *buzzer_value = buzzer_item->valuestring;
        if (strcmp(buzzer_value, "on") == 0) {
            mqtt_Buzzer = true;
            printf("蜂鸣器已开启\r\n");
        } else if (strcmp(buzzer_value, "off") == 0) {
            mqtt_Buzzer = false;
            printf("蜂鸣器已关闭\r\n");
        }
    }

    cJSON *DC_item = cJSON_GetObjectItem(root, "DC");
    if (DC_item != NULL && cJSON_IsString(DC_item)) {
        const char *DC_value = DC_item->valuestring;
        if (strcmp(DC_value, "on") == 0) {
            // 打开电机
            mqtt_DC = true;
            printf("直流电机已开启\r\n");
        } else if (strcmp(DC_value, "off") == 0) {
            // 关闭电机
            mqtt_DC = false;
            printf("直流电机已关闭\r\n");
        }
    }

    // 调节RGB亮度
    cJSON *pwm_item1 = cJSON_GetObjectItem(root, "greenpwm");
    if (pwm_item1 != NULL && cJSON_IsNumber(pwm_item1)) {
        int pwm_value = pwm_item1->valueint;
        printf("pwm后: %d\r\n", pwm_value);
        AW2013_Control_Green(pwm_value);
    }

    cJSON *pwm_item2 = cJSON_GetObjectItem(root, "redpwm");
    if (pwm_item2 != NULL && cJSON_IsNumber(pwm_item2)) {
        int pwm_value = pwm_item2->valueint;
        printf("pwm后: %d\r\n", pwm_value);
        AW2013_Control_Red(pwm_value);
    }

    cJSON *pwm_item3 = cJSON_GetObjectItem(root, "bluepwm");
    if (pwm_item3 != NULL && cJSON_IsNumber(pwm_item3)) {
        int pwm_value = pwm_item3->valueint;
        printf("pwm后: %d\r\n", pwm_value);
        AW2013_Control_Blue(pwm_value);
    }
}

// 温湿度任务
void sht20_task(void)
{
    float temperature = 0;
    float humidity = 0;
    char buf[64];
    char BH_buf[32];
    temp_humidity_data_t th_data;
    msg_data_t msg_data = {0};
    SHT20_Init(); // SHT20初始化
    printf("温湿度初始化完成\r\n");

    while (1) {
        osMutexAcquire(i2c_mutex, osWaitForever); // 加锁
        // 读取温湿度数据
        SHT20_ReadData(&temperature, &humidity);
        osMutexRelease(i2c_mutex); // 解锁
        temp = temperature;
        humi = humidity;
        th_data.temperature = temperature; // 温度值
        th_data.humidity = humidity;       // 湿度值
        // 填充 msg_data
        msg_data.value = (uint8_t *)&th_data;       // 数据指针
        msg_data.value_len = sizeof(th_data);       // 数据长度
        sle_server_send_report_by_handle(msg_data); // 温湿度发送给遥控
        // mqtt发送温湿度数据
        snprintf(buf, sizeof(buf), "{\"temp\":%.1f, \"humidity\":%.1f}", temperature, humidity);
        snprintf(BH_buf, sizeof(BH_buf), "{\"BH\":%d}", als);
        mqtt_publish("ws63_sht20", buf);
        osDelay(50);
        mqtt_publish("ws63_bh", BH_buf);
        osDelay(50);
        // 判断温湿度以及光照
        if ((temperature >= 27.0f && humidity >= 70.0f) || als >= 1000) {
            printf("打开LED\r\n");
            temp_lx_Buzzer = true;
            temp_lx_LED = true;
            // 打开电机
            temp_lx_DC = true;
        } else {
            printf("关闭LED\r\n");
            temp_lx_Buzzer = false;
            temp_lx_LED = false;
            // 关闭电机
            temp_lx_DC = false;
        }
        printf("%s\r\n", buf);
        osDelay(100);
    }
}

// mqtt任务
void mqtt_task(void)
{
    wifi_connect();
    osDelay(DELAY_TIME_MS);
    mqtt_connect();
    set_message_callback(handle_message);
    AW2013_Init(); // 三色LED灯的初始化
    AW2013_Control_RGB(RGB_OFF, RGB_OFF, RGB_OFF);
}

// oled屏任务
void oled_task(void)
{
    ssd1306_init(); // OLED 显示屏初始化
    printf("oled初始化完成\r\n");
    ssd1306_cls(); // 清屏

    while (1) {
        osMutexAcquire(i2c_mutex, osWaitForever); // 加锁
        // 屏幕显示温度
        char showtemp[30];
        snprintf(showtemp, sizeof(showtemp), "  Temp: %.1f    ", temp);
        ssd1306_show_str(OLED_TEXT16_COLUMN_0, OLED_TEXT16_LINE_0, showtemp, TEXT_SIZE_16);
        // 屏幕显示湿度
        char showhumi[30];
        snprintf(showhumi, sizeof(showhumi), "  Humi: %.1f%%    ", humi);
        ssd1306_show_str(OLED_TEXT16_COLUMN_0, OLED_TEXT16_LINE_1, showhumi, TEXT_SIZE_16);
        char showLx[30];
        snprintf(showLx, sizeof(showLx), "  Lx: %d      ", als);
        ssd1306_show_str(OLED_TEXT16_COLUMN_0, OLED_TEXT16_LINE_2, showLx, TEXT_SIZE_16);
        if (lock_flag) {
            ssd1306_show_str(OLED_TEXT16_COLUMN_0, OLED_TEXT16_LINE_3, "  NFCLock: OFF  ", TEXT_SIZE_16);
        } else {
            ssd1306_show_str(OLED_TEXT16_COLUMN_0, OLED_TEXT16_LINE_3, "  NFCLock: ON   ", TEXT_SIZE_16);
        }
        osMutexRelease(i2c_mutex); // 解锁
        osDelay(DELAY_TIME_MS);
    }
}

// nfc任务
void nfc_task(void)
{
    uint8_t ndefLen = 0;     // ndef包的长度
    uint8_t ndef_Header = 0; // ndef消息开始标志位-用不到
    NT3HEraseAllTag();       // 擦除NFC数据

    while (1) {
        if (NT3HReadHeaderNfc(&ndefLen, &ndef_Header)) {
            ndefLen += NDEF_HEADER_SIZE;
            ndefBuff = (uint8_t *)malloc(ndefLen + 1);
            if (get_NDEFDataPackage(ndefBuff, ndefLen) == ERRCODE_SUCC) {
                if (ndefBuff[9] == 0x6f && ndefBuff[10] == 0x6e && ps >= 80) {
                    lock_flag = true;
                    // 打开电机
                    nfc_DC = true;
                    osDelay(DELAY_TIME_MS);
                    // 关闭电机
                    nfc_DC = false;
                    NT3HEraseAllTag(); // 擦除NFC数据
                } else {
                    if (ndefBuff[9] == 0x6f && ndefBuff[10] == 0x6e) {
                        NT3HEraseAllTag(); // 擦除NFC数据
                    }
                    lock_flag = false;
                }
            }
            free(ndefBuff);
        }
        osDelay(150);
    }
}

// 人体红外、接近、光敏任务
void ap3216_task(void)
{
    uint16_t ir = 0; // 人体红外传感器
    AP3216C_Init();
    while (1) {
        // 读取传感器数据
        osMutexAcquire(i2c_mutex, osWaitForever); // 加锁
        AP3216C_ReadData(&ir, &als, &ps);
        osMutexRelease(i2c_mutex); // 解锁
        osDelay(DELAY_TIME_MS);
    }
}

/* SLE服务器任务线程 */
static void *sle_server_task(const char *arg)
{
    unused(arg);
    osal_msleep(500); /* 延时500毫秒 */
    sle_server_init();
    while (1) {
        msg_data_t msg_data = {0};
        int msg_ret = osal_msg_queue_read_copy(g_msg_queue, &msg_data, &g_msg_rev_size, OSAL_WAIT_FOREVER);
        if (msg_ret != OSAL_SUCCESS) {
            printf("msg queue read copy fail.");
            if (msg_data.value != NULL) {
                osal_vfree(msg_data.value);
            }
        }
        if (msg_data.value != NULL) {
            sle_server_send_report_by_handle(msg_data);
        }
    }
    return NULL;
}

// 蜂鸣器、电机、LED统一开关
void controlActuators(void)
{
    // 设置LED灯输出模式
    uapi_gpio_set_dir(GPIO_13, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_dir(GPIO_10, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(GPIO_10, GPIO_LEVEL_LOW);  // 默认关闭蜂鸣器
    uapi_gpio_set_val(GPIO_13, GPIO_LEVEL_HIGH); // 默认关闭LED
    printf("蜂鸣器、LED灯初始化完成\r\n");
    while (1) {
        if (mqtt_LED || temp_lx_LED) {
            uapi_gpio_set_val(GPIO_13, GPIO_LEVEL_LOW);
        } else {
            uapi_gpio_set_val(GPIO_13, GPIO_LEVEL_HIGH); // 关闭LED
        }
        if (nfc_DC || temp_lx_DC || mqtt_DC || sle_DC) {
            // 打开电机
            uapi_gpio_set_val(GPIO_08, GPIO_LEVEL_LOW);
            uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_HIGH);
        } else {
            // 关闭电机
            uapi_gpio_set_val(GPIO_08, GPIO_LEVEL_LOW);
            uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_LOW);
        }
        if (sle_Buzzer || mqtt_Buzzer || temp_lx_Buzzer) {
            uapi_gpio_set_val(GPIO_10, GPIO_LEVEL_HIGH); // 打开蜂鸣器
        } else {
            uapi_gpio_set_val(GPIO_10, GPIO_LEVEL_LOW); // 关闭蜂鸣器
        }
        osDelay(100);
    }
}

// 入口函数
void starsea_node(void)
{
    i2c_mutex = osMutexNew(NULL);
    if (i2c_mutex == NULL) {
        printf("Failed to create I2C mutex!\r\n");
        return;
    }
    // 初始化电机
    uapi_gpio_set_dir(GPIO_08, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_dir(GPIO_09, GPIO_DIRECTION_OUTPUT);

    // 温湿度线程
    osThreadAttr_t attr;
    attr.name = "sht20";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x2000;
    attr.priority = osPriorityAboveNormal; // 较高优先级，因为环境数据通常很重要
    sht20_ID = osThreadNew((osThreadFunc_t)sht20_task, NULL, &attr);
    if (sht20_ID != NULL) {
        printf("sht20线程创建成功\r\n");
    }

    // mqtt线程
    attr.name = "mqtt";
    attr.priority = osPriorityHigh; // 最高优先级，网络通信需要及时响应
    mqtt_ID = osThreadNew((osThreadFunc_t)mqtt_task, NULL, &attr);
    if (mqtt_ID != NULL) {
        printf("mqtt线程创建成功\r\n");
    }

    // oled线程
    attr.name = "oled";
    attr.priority = osPriorityBelowNormal; // 较低优先级，显示更新可以稍慢
    oled_ID = osThreadNew((osThreadFunc_t)oled_task, NULL, &attr);
    if (oled_ID != NULL) {
        printf("oled线程创建成功\r\n");
    }

    // nfc线程
    attr.name = "nfc";
    attr.priority = osPriorityNormal; // 中等优先级
    nfc_ID = osThreadNew((osThreadFunc_t)nfc_task, NULL, &attr);
    if (nfc_ID != NULL) {
        printf("nfc线程创建成功\r\n");
    }

    // led、电机、蜂鸣器线程
    attr.name = "controlactuators";
    attr.priority = osPriorityNormal; // 中等优先级
    controlActuators_ID = osThreadNew((osThreadFunc_t)controlActuators, NULL, &attr);
    if (controlActuators_ID != NULL) {
        printf("led、蜂鸣器、电机线程创建成功\r\n");
    }

    // 光照和红外线程
    attr.name = "ap3216";
    attr.priority = osPriorityAboveNormal; // 较高优先级，环境传感器数据通常重要
    ap3216_ID = osThreadNew((osThreadFunc_t)ap3216_task, NULL, &attr);
    if (ap3216_ID != NULL) {
        printf("ap3216线程创建成功\r\n");
    }

    // 星闪线程
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    int ret = osal_msg_queue_create("sle_msg", g_msg_rev_size, &g_msg_queue, 0, g_msg_rev_size);
    if (ret != OSAL_SUCCESS) {
        printf("create queue failure!,error:%x\n", ret);
    }

    task_handle =
        osal_kthread_create((osal_kthread_handler)sle_server_task, 0, "sle_server_task", SLE_SERVER_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_SERVER_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

app_run(starsea_node);