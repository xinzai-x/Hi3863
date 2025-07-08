#include "common_def.h"
#include "pinctrl.h"
#include "uart.h"
#include "soc_osal.h"
#include "app_init.h"
#include "securec.h"
#include "sle_errcode.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_server_adv/sle_server_adv.h"
#include "sle_server_adv/sle_server.h"

unsigned int g_msg_rev_size = sizeof(msg_data_t); // 消息接收大小
/* 全局变量 */
unsigned long g_msg_queue = 0;                    // 消息队列句柄

/* SLE服务器任务线程 */
static void *sle_server_task(const char *arg)
{
    unused(arg);
    osal_msleep(500); /* 延时500毫秒 */
    sle_server_init();
    // app_uart_init_config();
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

/* SLE服务器入口函数 */
static void sle_server_entry(void)
{
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

/* 运行SLE服务器入口函数 */
app_run(sle_server_entry);