#include "common_def.h"
#include "soc_osal.h"
#include "securec.h"
#include "product.h"
#include "bts_le_gap.h"
#include "uart.h"
#include "pinctrl.h"
#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "sle_client.h"
#include "stdio.h"
#include "string.h"
#include "osal_debug.h"
#include "cmsis_os2.h"
#include "hal_bsp_sc12b/hal_bsp_sc12b.h"
#include "hal_bsp_oled/bsp_st7789_4line.h"
#include "app_init.h"
#define SLE_MTU_SIZE_DEFAULT 512
#define SLE_SEEK_INTERVAL_DEFAULT 100
#define SLE_SEEK_WINDOW_DEFAULT 100
#define UUID_16BIT_LEN 2
#define UUID_128BIT_LEN 16
// 全局变量存储最新温湿度
static float g_current_temp = 0.0f;
static float g_current_humidity = 0.0f;

#define SLE_SERVER_NAME "sle_test"

unsigned long g_msg_queue = 0;
unsigned int g_msg_rev_size = sizeof(msg_data_t);
/* 串口接收缓冲区大小 */
#define UART_RX_MAX 1024
uint8_t uart_rx_buffer[UART_RX_MAX];

static ssapc_find_service_result_t g_sle_find_service_result = {0};
static sle_announce_seek_callbacks_t g_sle_uart_seek_cbk = {0};
static sle_connection_callbacks_t g_sle_uart_connect_cbk = {0};
static ssapc_callbacks_t g_sle_uart_ssapc_cbk = {0};
static sle_addr_t g_sle_remote_addr = {0};
ssapc_write_param_t g_sle_send_param = {0};
uint16_t g_sle_uart_conn_id = 0;
/* 开启扫描 */
void sle_start_scan(void)
{
    sle_seek_param_t param = {0};
    param.own_addr_type = 0;
    param.filter_duplicates = 0;
    param.seek_filter_policy = 0;
    param.seek_phys = 1;
    param.seek_type[0] = 1;
    param.seek_interval[0] = SLE_SEEK_INTERVAL_DEFAULT;
    param.seek_window[0] = SLE_SEEK_WINDOW_DEFAULT;
    sle_set_seek_param(&param);
    sle_start_seek();
}
/* 星闪协议栈使能回调 */
static void sle_client_sle_enable_cbk(errcode_t status)
{
    unused(status);
    printf("sle enable.\r\n");
    sle_start_scan();
}
/* 扫描使能回调函数 */
static void sle_client_seek_enable_cbk(errcode_t status)
{
    if (status != 0) {
        printf("[%s] status error\r\n", __FUNCTION__);
    }
}
/* 返回扫描结果回调 */
static void sle_client_seek_result_info_cbk(sle_seek_result_info_t *seek_result_data)
{
    printf("[seek_result_info_cbk] scan data : "); // 打印扫描到的设备名称 hex
    for (uint8_t i = 0; i < seek_result_data->data_length; i++) {
        printf("0x%X ", seek_result_data->data[i]);
    }
    printf("\r\n");
    if (seek_result_data == NULL) {
        printf("status error\r\n");
    } else if (strstr((const char *)seek_result_data->data, SLE_SERVER_NAME) != NULL) { // 名称对比成功
        memcpy_s(&g_sle_remote_addr, sizeof(sle_addr_t), &seek_result_data->addr,
                 sizeof(sle_addr_t)); // 扫描到目标设备，将目标设备的名字拷贝到g_sle_remote_addr
        sle_stop_seek();              // 停止扫描
    }
}
/* 扫描关闭回调函数 */
static void sle_client_seek_disable_cbk(errcode_t status)
{
    if (status != 0) {
        printf("[%s] status error = %x\r\n", __FUNCTION__, status);
    } else {
        sle_remove_paired_remote_device(&g_sle_remote_addr); // 关闭扫描后，先删除之前的配对
        sle_connect_remote_device(&g_sle_remote_addr);       // 发送连接请求
    }
}
/* 扫描注册初始化函数 */
static void sle_client_seek_cbk_register(void)
{
    g_sle_uart_seek_cbk.sle_enable_cb = sle_client_sle_enable_cbk;
    g_sle_uart_seek_cbk.seek_enable_cb = sle_client_seek_enable_cbk;
    g_sle_uart_seek_cbk.seek_result_cb = sle_client_seek_result_info_cbk;
    g_sle_uart_seek_cbk.seek_disable_cb = sle_client_seek_disable_cbk;
    sle_announce_seek_register_callbacks(&g_sle_uart_seek_cbk);
}
/* 连接状态改变回调 */
static void sle_client_connect_state_changed_cbk(uint16_t conn_id,
                                                 const sle_addr_t *addr,
                                                 sle_acb_state_t conn_state,
                                                 sle_pair_state_t pair_state,
                                                 sle_disc_reason_t disc_reason)
{
    unused(addr);
    unused(pair_state);
    printf("[%s] disc_reason:0x%x\r\n", __FUNCTION__, disc_reason);
    g_sle_uart_conn_id = conn_id;
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        printf(" SLE_ACB_STATE_CONNECTED\r\n");
        if (pair_state == SLE_PAIR_NONE) {
            sle_pair_remote_device(&g_sle_remote_addr);
        }
        printf(" sle_low_latency_rx_enable \r\n");
    } else if (conn_state == SLE_ACB_STATE_NONE) {
        printf(" SLE_ACB_STATE_NONE\r\n");
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        printf(" SLE_ACB_STATE_DISCONNECTED\r\n");
        sle_remove_paired_remote_device(&g_sle_remote_addr);
        sle_start_scan();
    } else {
        printf(" status error\r\n");
    }
}
/* 配对完成回调 */
void sle_client_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    printf("[%s] conn_id:%d, state:%d,addr:%02x***%02x%02x\n", __FUNCTION__, conn_id, status, addr->addr[0],
           addr->addr[4], addr->addr[5]); /* 0 14 15: addr */
    if (status == 0) {
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE_DEFAULT;
        info.version = 1;
        ssapc_exchange_info_req(0, g_sle_uart_conn_id, &info);
    }
}
/* 连接注册初始化函数 */
static void sle_client_connect_cbk_register(void)
{
    g_sle_uart_connect_cbk.connect_state_changed_cb = sle_client_connect_state_changed_cbk;
    g_sle_uart_connect_cbk.pair_complete_cb = sle_client_pair_complete_cbk;
    sle_connection_register_callbacks(&g_sle_uart_connect_cbk);
}
/* 更新mtu回调 */
static void sle_client_exchange_info_cbk(uint8_t client_id,
                                         uint16_t conn_id,
                                         ssap_exchange_info_t *param,
                                         errcode_t status)
{
    printf("[%s] pair complete client id:%d status:%d\r\n", __FUNCTION__, client_id, status);
    printf("[%s] mtu size: %d, version: %d.\r\n", __FUNCTION__, param->mtu_size, param->version);
    ssapc_find_structure_param_t find_param = {0};
    find_param.type = SSAP_FIND_TYPE_PROPERTY;
    find_param.start_hdl = 1;
    find_param.end_hdl = 0xFFFF;
    ssapc_find_structure(0, conn_id, &find_param);
}
/* 发现服务回调 */
static void sle_client_find_structure_cbk(uint8_t client_id,
                                          uint16_t conn_id,
                                          ssapc_find_service_result_t *service,
                                          errcode_t status)
{
    printf("[%s] client: %d conn_id:%d status: %d \r\n", __FUNCTION__, client_id, conn_id, status);
    printf("[%s] find structure start_hdl:[0x%02x], end_hdl:[0x%02x], uuid len:%d\r\n", __FUNCTION__,
           service->start_hdl, service->end_hdl, service->uuid.len);
    g_sle_find_service_result.start_hdl = service->start_hdl;
    g_sle_find_service_result.end_hdl = service->end_hdl;
    memcpy_s(&g_sle_find_service_result.uuid, sizeof(sle_uuid_t), &service->uuid, sizeof(sle_uuid_t));
}
/* 发现特征回调 */
static void sle_client_find_property_cbk(uint8_t client_id,
                                         uint16_t conn_id,
                                         ssapc_find_property_result_t *property,
                                         errcode_t status)
{
    printf(
        "[%s] client id: %d, conn id: %d, operate ind: %d, "
        "descriptors count: %d status:%d property->handle %d\r\n",
        __FUNCTION__, client_id, conn_id, property->operate_indication, property->descriptors_count, status,
        property->handle);
    g_sle_send_param.handle = property->handle;
    g_sle_send_param.type = SSAP_PROPERTY_TYPE_VALUE;
}
/* 发现特征完成回调 */
static void sle_client_find_structure_cmp_cbk(uint8_t client_id,
                                              uint16_t conn_id,
                                              ssapc_find_structure_result_t *structure_result,
                                              errcode_t status)
{
    unused(conn_id);
    printf("[%s] client id:%d status:%d type:%d uuid len:%d \r\n", __FUNCTION__, client_id, status,
           structure_result->type, structure_result->uuid.len);
}
/* 收到写响应回调 */
static void sle_client_write_cfm_cbk(uint8_t client_id,
                                     uint16_t conn_id,
                                     ssapc_write_result_t *write_result,
                                     errcode_t status)
{
    printf("[%s] conn_id:%d client id:%d status:%d handle:%02x type:%02x\r\n", __FUNCTION__, conn_id, client_id, status,
           write_result->handle, write_result->type);
}
/* 收到通知回调 */
void sle_notification_cbk(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);

    // 检查数据长度是否合法（假设温湿度是2个float，共8字节）
    if (data->data_len != sizeof(float) * 2) {
        printf("Error: Invalid data length (%d bytes)\n", data->data_len);
        return;
    }

    // 解析温湿度数据（假设小端字节序）
    memcpy(&g_current_temp, data->data, sizeof(float));
    memcpy(&g_current_humidity, data->data + sizeof(float), sizeof(float));
    char buf[64];
    char buf1[64];
    snprintf(buf, sizeof(buf), "temp: %.1f", g_current_temp);
    snprintf(buf1, sizeof(buf1), "humi: %.1f%%", g_current_humidity);

    DisplayString_chat(0, 90, WHITE, buf);
    DisplayString_chat(130, 90, WHITE, buf1);

    // 格式化输出
    printf("Temp: %.1f°C, Humi: %.1f%%\n", g_current_temp, g_current_humidity);
}
/* 收到指示回调 */
void sle_indication_cbk(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    printf("recv len:%d data: ", data->data_len);
    for (uint16_t i = 0; i < data->data_len; i++) {
        printf("%c", data->data[i]);
    }
    printf("\r\n");
}

static void sle_client_ssapc_cbk_register(void)
{
    g_sle_uart_ssapc_cbk.exchange_info_cb = sle_client_exchange_info_cbk;
    g_sle_uart_ssapc_cbk.find_structure_cb = sle_client_find_structure_cbk;
    g_sle_uart_ssapc_cbk.ssapc_find_property_cbk = sle_client_find_property_cbk;
    g_sle_uart_ssapc_cbk.find_structure_cmp_cb = sle_client_find_structure_cmp_cbk;
    g_sle_uart_ssapc_cbk.write_cfm_cb = sle_client_write_cfm_cbk;
    g_sle_uart_ssapc_cbk.notification_cb = sle_notification_cbk;
    g_sle_uart_ssapc_cbk.indication_cb = sle_indication_cbk;
    ssapc_register_callbacks(&g_sle_uart_ssapc_cbk);
}

void sle_client_init(void)
{
    osal_msleep(500); /* 500: 延时 */
    printf("sle enable.\r\n");
    sle_client_seek_cbk_register();
    sle_client_connect_cbk_register();
    sle_client_ssapc_cbk_register();
    if (enable_sle() != ERRCODE_SUCC) {
        printf("sle enbale fail !\r\n");
    }
}

static void *sle_client_task(char *arg)
{
    unused(arg);
    // app_uart_init_config();
    sle_client_init();
    return NULL;
}

osThreadId_t task1_ID; // 任务1

#define DELAY_TIME_MS 10
#define INT_IO GPIO_14
uint16_t current_state, prev_state;
uint8_t int_flag;
void press_callback_func(pin_t pin, uintptr_t param)
{
    unused(pin);
    unused(param);
    int_flag = 1;
}
void user_key_init(void)
{
    /* 配置引脚为下拉，并设置为输入模式 */
    uapi_pin_set_mode(INT_IO, PIN_MODE_0);
    uapi_pin_set_pull(INT_IO, PIN_PULL_TYPE_DOWN);
    uapi_gpio_set_dir(INT_IO, GPIO_DIRECTION_INPUT);
    /* 注册指定GPIO下降沿中断，回调函数为gpio_callback_func */
    if (uapi_gpio_register_isr_func(INT_IO, GPIO_INTERRUPT_RISING_EDGE, press_callback_func) != 0) {
        printf("register_isr_func fail!\r\n");
        uapi_gpio_unregister_isr_func(INT_IO);
    }
    /* 使能中断 */
    uapi_gpio_enable_interrupt(INT_IO);
}

void get_key_info(uint16_t key_value)
{
    uint16_t current = key_value;
    char *cmd_str = NULL; // 要发送的功能字符串

    switch (current) {
        case 0x2000:
            cmd_str = "关机";
            DisplayString_chat(0, 0, WHITE, "DC: ON ");
            break;
        case 0x200:
            cmd_str = "主菜单";
            DisplayString_chat(0, 0, WHITE, "DC: Off");
            break;
        case 0x1000:
            cmd_str = "前";
            DisplayString_chat(130, 26, WHITE, "GRB_Green: ON ");
            DisplayString_chat(130, 58, WHITE, "GRB: ON ");
            break;
        case 0x400:
            cmd_str = "后";
            DisplayString_chat(130, 58, WHITE, "GRB: Off");
            DisplayString_chat(130, 26, WHITE, "GRB_Green: Off");
            break;
        case 0x4000:
            cmd_str = "左";
            DisplayString_chat(0, 26, WHITE, "GRB_Red: ON ");
            DisplayString_chat(130, 58, WHITE, "GRB: ON ");
            break;
        case 0x100:
            cmd_str = "右";
            DisplayString_chat(0, 56, WHITE, "GRB_Blue: ON ");
            DisplayString_chat(130, 58, WHITE, "GRB: ON ");
            break;
        case 0x800:
            cmd_str = "OK";
            DisplayString_chat(130, 26, WHITE, "GRB_Green: Off");
            DisplayString_chat(0, 26, WHITE, "GRB_Red: Off");
            DisplayString_chat(0, 58, WHITE, "GRB_Blue: Off");
            DisplayString_chat(130, 58, WHITE, "GRB: Off");
            break;
        case 0x8000:
            cmd_str = "减";
            DisplayString_chat(130, 58, WHITE, "GRB: Off");
            DisplayString_chat(0, 26, WHITE, "GRB_Red: Off");
            break;
        case 0x80:
            cmd_str = "加";
            DisplayString_chat(130, 58, WHITE, "GRB: Off");
            DisplayString_chat(0, 56, WHITE, "GRB_Blue: Off");
            break;
        case 0x10:
            cmd_str = "设置";
            DisplayString_chat(130, 0, WHITE, "Buzzer: ON ");
            break;
        case 0x20:
            cmd_str = "返回";
            DisplayString_chat(130, 0, WHITE, "Buzzer: Off");
            break;
        case 0x40:
            cmd_str = "功能";
            break;
        default:
            return; // 不处理未知按键
    }

    // 构造 SSAPC 数据包并发送
    ssapc_write_param_t send_param = {0};
    send_param.handle = g_sle_send_param.handle; // 使用之前发现的 handle
    send_param.type = g_sle_send_param.type;     // 使用之前设置的 type
    send_param.data_len = strlen(cmd_str);       // 数据长度
    send_param.data = (uint8_t *)cmd_str;        // 数据内容

    // 发送数据
    ssapc_write_cmd(0, g_sle_uart_conn_id, &send_param);
}

void task1(void)
{
    // 初始化触摸芯片
    sc12b_init();
    // 初始化按键中断引脚
    user_key_init();
    // 初始化 spi 引脚及配置
    app_spi_init_pin();
    app_spi_master_init_config();
    // 屏幕初始化
    ST7789_Init();

    // 初始化 SLE 协议栈（不再需要消息队列）
    osal_task *task_handle =
        osal_kthread_create((osal_kthread_handler)sle_client_task, 0, "sle_client_task", SLE_SERVER_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_SERVER_TASK_PRIO);
        osal_kfree(task_handle);
    }

    DisplayString_chat(0, 0, WHITE, "DC: Off");
    DisplayString_chat(130, 26, WHITE, "GRB_Green: Off");
    DisplayString_chat(0, 26, WHITE, "GRB_Red: Off");
    DisplayString_chat(0, 58, WHITE, "GRB_Blue: Off");
    DisplayString_chat(130, 58, WHITE, "GRB: Off");
    DisplayString_chat(130, 0, WHITE, "Buzzer: Off");
    DisplayString_chat(0, 90, WHITE, "temp: 0.0");
    DisplayString_chat(130, 90, WHITE, "humi: 0.0%");

    while (1) {
        // 如果发生中断，则读取按键键值
        if (int_flag) {
            SC12B_ReadRegByteData(OUTPUT1, &current_state);
            // 获取按键的功能
            get_key_info(current_state);
            // 记录上次按下的按键
            prev_state = current_state;
            // 清除标志位
            int_flag = 0;
        }

        osDelay(DELAY_TIME_MS);
    }
}
static void sle_client(void)
{
    printf("Enter base_sc12b_demo()!\r\n");

    osThreadAttr_t attr;
    attr.name = "task1";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x2000;
    attr.priority = osPriorityNormal;

    task1_ID = osThreadNew((osThreadFunc_t)task1, NULL, &attr);
    if (task1_ID != NULL) {
        printf("ID = %d, Create task1_ID is OK!\r\n", task1_ID);
    }
}

app_run(sle_client);