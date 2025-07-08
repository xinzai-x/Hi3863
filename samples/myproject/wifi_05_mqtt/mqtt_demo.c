/*
 * Copyright (c) 2024 Beijing HuaQingYuanJian Education Technology Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
#include "wifi/wifi_connect.h"
osThreadId_t mqtt_init_task_id; // mqtt订阅数据任务

#define SERVER_IP_ADDR "110.42.63.197"
#define SERVER_IP_PORT 24187
#define CLIENT_ID "WS63"

#define MQTT_TOPIC_SUB "subTopic"       //订阅主题
#define MQTT_TOPIC_PUB "pubTopic"       //发布主题
#define MQTT_USERNAME "WS63"           // mqtt用户名定义
#define MQTT_PASSWORD "123456"         // mqtt密码定义

char *g_msg = "hello!";
MQTTClient client;
volatile MQTTClient_deliveryToken deliveredToken;
extern int MQTTClient_init(void);
/* 回调函数，处理消息到达 */
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    unused(context);
    printf("Message with token value %d delivery confirmed\r\n", dt);
    deliveredToken = dt;
}

/* 回调函数，处理接收到的消息 */
int messageArrived(void *context, char *topicname, int topiclen, MQTTClient_message *message)
{
    unused(context);
    unused(topiclen);
    printf("Message arrived on topic: %s\r\n", topicname);
    printf("Message: %.*s\r\n", message->payloadlen, (char *)message->payload);
    return 1; // 表示消息已被处理
}

/* 回调函数，处理连接丢失 */
void connlost(void *context, char *cause)
{
    unused(context);
    printf("Connection lost: %s\r\n", cause);
}
/* 消息订阅 */
int mqtt_subscribe(const char *topic)
{
    printf("subscribe start\r\n");
    MQTTClient_subscribe(client, topic, 1);
    return 0;
}

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
    while (1) {
        osDelay(800); /* 100: 延时1s  */
        mqtt_publish(MQTT_TOPIC_PUB, g_msg);
    }

    return ERRCODE_SUCC;
}
void mqtt_init_task(const char *argument)
{
    unused(argument);
    wifi_connect();
    osDelay(200); /* 200: 延时2s  */
    mqtt_connect();
}

void network_wifi_mqtt_example(void)
{
    printf("Enter network_wifi_mqtt_example()!\r\n");

    osThreadAttr_t options;
    options.name = "mqtt_init_task";
    options.attr_bits = 0;
    options.cb_mem = NULL;
    options.cb_size = 0;
    options.stack_mem = NULL;
    options.stack_size = 0x3000;
    options.priority = osPriorityNormal;

    mqtt_init_task_id = osThreadNew((osThreadFunc_t)mqtt_init_task, NULL, &options);
    if (mqtt_init_task_id != NULL) {
        printf("ID = %d, Create mqtt_init_task_id is OK!\r\n", mqtt_init_task_id);
    }
}
/* Run the sample. */
app_run(network_wifi_mqtt_example);