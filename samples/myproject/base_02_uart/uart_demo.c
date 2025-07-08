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

#include "pinctrl.h"
#include "uart.h"
#include "osal_debug.h"
#include "soc_osal.h"
#include "app_init.h"
#include "string.h"
#include "cmsis_os2.h"
osThreadId_t task1_ID; // 任务1 ID

#define DELAY_TIME_MS 1
#define UART_RECV_SIZE 10

uint8_t uart_recv[UART_RECV_SIZE] = {0};

#define UART_INT_MODE 1
#if (UART_INT_MODE)
static uint8_t uart_rx_flag = 0;
#endif
uart_buffer_config_t g_app_uart_buffer_config = {.rx_buffer = uart_recv, .rx_buffer_size = UART_RECV_SIZE};
// 初始化 UART 引脚
void uart_gpio_init(void)
{
    uapi_pin_set_mode(GPIO_17, PIN_MODE_1);
    uapi_pin_set_mode(GPIO_18, PIN_MODE_1);
}
// 初始化 UART 配置
void uart_init_config(void)
{
    // 定义一个 uart_attr_t 类型的结构体变量 attr，用于存储 UART 的属性
    uart_attr_t attr = {
        .baud_rate = 115200, .data_bits = UART_DATA_BIT_8, .stop_bits = UART_STOP_BIT_1, .parity = UART_PARITY_NONE};

    uart_pin_config_t pin_config = {.tx_pin = S_MGPIO0, .rx_pin = S_MGPIO1, .cts_pin = PIN_NONE, .rts_pin = PIN_NONE};
    uapi_uart_deinit(UART_BUS_0);
    int ret = uapi_uart_init(UART_BUS_0, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
    if (ret != 0) {
        printf("uart init failed ret = %02x\n", ret);
    }
}

#if (UART_INT_MODE)
void uart_read_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (buffer == NULL || length == 0) {
        return;
    }
    if (memcpy_s(uart_recv, length, buffer, length) != EOK) {
        return;
    }
    uart_rx_flag = 1;
}
#endif

void *uart_task(const char *arg)
{
    // 调用 unused 宏，通常用于避免编译器警告，表示 arg 参数未被使用
    unused(arg);
    // 对 UART 相关的 GPIO 进行初始化
    uart_gpio_init();
    // 对 UART 进行初始化配置
    uart_init_config();
    // 如果定义了 UART_INT_MODE 宏，即使用中断模式
#if (UART_INT_MODE)
    // 注册串口回调函数
    if (uapi_uart_register_rx_callback(0, UART_RX_CONDITION_MASK_IDLE, 1, uart_read_handler) == ERRCODE_SUCC) {
    }
#endif

    while (1) {
#if (UART_INT_MODE) // 如果定义了 UART_INT_MODE 宏，即使用中断模式
        while (!uart_rx_flag) {
            osDelay(DELAY_TIME_MS);
        }
        uart_rx_flag = 0;
        printf("uart int rx = [%s]\n", uart_recv);
        memset(uart_recv, 0, UART_RECV_SIZE);
#else // 如果未定义 UART_INT_MODE 宏，即使用轮询模式
        if (uapi_uart_read(UART_BUS_0, uart_recv, UART_RECV_SIZE, 0)) {
            printf("uart poll rx = ");
            uapi_uart_write(UART_BUS_0, uart_recv, UART_RECV_SIZE, 0);
        }
#endif
    }
    return NULL;
}

static void uart_entry(void)
{
    printf("Enter uart_entry()!\r\n");

    osThreadAttr_t attr;
    attr.name = "UartTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000;
    attr.priority = osPriorityNormal;

    task1_ID = osThreadNew((osThreadFunc_t)uart_task, NULL, &attr);
    if (task1_ID != NULL) {
        printf("ID = %d, Create task1_ID is OK!\r\n", task1_ID);
    }
}

/* Run the uart_entry. */
app_run(uart_entry);