#include "soc_osal.h"
#include "app_init.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common_def.h"
#include "MQTTClientPersistence.h"
#include "MQTTClient.h"
#include "errcode.h"

#define SERVER_IP_ADDR "110.42.63.197"      //mqtt服务地址
#define SERVER_IP_PORT 24187                //mqtt端口号
#define CLIENT_ID "WS63"                    //客户端id
#define MQTT_USERNAME "WS63"                // mqtt用户名定义
#define MQTT_PASSWORD "123456"              // mqtt密码定义
#define MQTT_TOPIC_SUB "starseaphone"       //订阅主题
MQTTClient client;
extern int MQTTClient_init(void);
volatile MQTTClient_deliveryToken deliveredToken;
// 声明全局回调函数指针
static void (*g_message_callback)(const char*, const char*) = NULL;

/* 回调函数，处理连接丢失 */
void connlost(void *context, char *cause)
{
    unused(context);
    printf("Connection lost: %s\r\n", cause);
}
/* 回调函数，处理接收到的消息 */
int messageArrived(void *context, char *topic, int topicLen, MQTTClient_message *msg)
{
    unused(context);
    unused(topic);
    unused(topicLen);
    if (g_message_callback) {
        g_message_callback(topic, (char*)msg->payload);
    }
    MQTTClient_freeMessage(&msg);
    return 1; // 表示消息已被处理
}
/* 回调函数，处理消息到达 */
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    unused(context);
    printf("Message with token value %d delivery confirmed\r\n", dt);
    deliveredToken = dt;
}
/* 消息订阅 */
int mqtt_subscribe(const char *topic)
{
    printf("subscribe start\r\n");
    MQTTClient_subscribe(client, topic, 1);
    return 0;
}
// 设置回调的函数（其他文件调用这个）
void set_message_callback(void (*callback)(const char*, const char*)) {
    g_message_callback = callback;
}

errcode_t mqtt_connect(void)
{
    int ret;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.username = MQTT_USERNAME;  // mqtt用户名
    conn_opts.password = MQTT_PASSWORD;  // mqtt密码
    /* 初始化MQTT客户端 */
    MQTTClient_init();
    char server_uri[64];
    snprintf(server_uri, sizeof(server_uri), "tcp://%s:%d", SERVER_IP_ADDR, SERVER_IP_PORT);
    /* 创建 MQTT 客户端 */
    ret = MQTTClient_create(&client, server_uri, CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("Failed to create MQTT client, return code %d\r\n", ret);
        return ERRCODE_FAIL;
    }
    conn_opts.keepAliveInterval = 120; /* 120: 保活时间  */
    conn_opts.cleansession = 1;
    // 绑定回调函数
    MQTTClient_setCallbacks(client, NULL, connlost, messageArrived, delivered);

    // 尝试连接
    if ((ret = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\r\n", ret);
        MQTTClient_destroy(&client); // 连接失败时销毁客户端
        return ERRCODE_FAIL;
    }
    printf("Connected to MQTT broker!\r\n");
    osDelay(200); /* 200: 延时2s  */
    // 订阅MQTT主题
    mqtt_subscribe(MQTT_TOPIC_SUB);

    return ERRCODE_SUCC;
}
