#include "common_def.h"
#include "pinctrl.h"
#include "uart.h"
#include "soc_osal.h"
#include "app_init.h"
#include "securec.h"
#include "gpio.h"
#include "sle_errcode.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_server_adv.h"
#include "../rgb_led/hal_bsp_aw2013.h"
#include "sle_server.h"

/* 常量定义 */
#define OCTET_BIT_LEN 8 // 字节位数
#define UUID_LEN_2 2    // UUID长度（2字节）
#define RGB_ON 100
#define RGB_OFF 0

/* 串口配置 */
#define UART_RX_MAX 512                // 串口接收缓冲区最大长度
uint8_t g_uart_rx_buffer[UART_RX_MAX]; // 串口接收缓冲区

/* SLE协议配置 */
#define SLE_MTU_SIZE_DEFAULT 512                         // 默认MTU大小
#define DEFAULT_PAYLOAD_SIZE (SLE_MTU_SIZE_DEFAULT - 12) // 设置有效载荷大小（避免客户端问题）

/* UUID定义 */
#define SLE_UUID_SERVER_SERVICE 0xABCD    // 服务UUID
#define SLE_UUID_SERVER_NTF_REPORT 0xBCDE // 通知属性UUID

/* 广播配置 */
#define SLE_ADV_HANDLE_DEFAULT 1 // 默认广播句柄

/* UUID和属性值存储 */
static char g_sle_uuid_app_uuid[UUID_LEN_2] = {0x0, 0x0};                         // 应用UUID
static char g_sle_property_value[OCTET_BIT_LEN] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; // 属性值

/* 连接和服务句柄 */
static uint16_t g_sle_conn_hdl = 0;    // 当前连接句柄
static uint8_t g_server_id = 0;        // 服务器ID
static uint16_t g_service_handle = 0;  // 服务句柄
static uint16_t g_property_handle = 0; // 属性句柄

/* UUID长度定义 */
#define UUID_16BIT_LEN 2   // 16位UUID长度
#define UUID_128BIT_LEN 16 // 128位UUID长度

/* 基础UUID值 */
static uint8_t g_sle_base[] = {0x73, 0x6C, 0x65, 0x5F, 0x74, 0x65, 0x73, 0x74}; // "sle_test"的ASCII码

/* 将16位数据编码为小端格式的2字节 */
static void encode2byte_little(uint8_t *ptr, uint16_t data)
{
    *(uint8_t *)((ptr) + 1) = (uint8_t)((data) >> 0x8); // 高位字节
    *(uint8_t *)(ptr) = (uint8_t)(data);                // 低位字节
}

/* 设置UUID基础部分 */
static void sle_uuid_set_base(sle_uuid_t *out)
{
    errcode_t ret;
    ret = memcpy_s(out->uuid, SLE_UUID_LEN, g_sle_base, SLE_UUID_LEN);
    if (ret != EOK) {
        printf("[%s] memcpy fail\n", __FUNCTION__);
        out->len = 0;
        return;
    }
    out->len = UUID_LEN_2;
}

/* 设置16位UUID */
static void sle_uuid_setu2(uint16_t u2, sle_uuid_t *out)
{
    sle_uuid_set_base(out);
    out->len = UUID_LEN_2;
    encode2byte_little(&out->uuid[14], u2); /* 14: UUID中的索引位置 */
}

/* 打印UUID信息 */
static void sle_uuid_print(sle_uuid_t *uuid)
{
    printf("[%s] ", __FUNCTION__);
    if (uuid == NULL) {
        printf("uuid is null\r\n");
        return;
    }
    if (uuid->len == UUID_16BIT_LEN) {
        printf("uuid: %02x %02x.\n", uuid->uuid[14], uuid->uuid[15]); /* 14 15: 16位UUID的存储位置 */
    } else if (uuid->len == UUID_128BIT_LEN) {
        for (size_t i = 0; i < UUID_128BIT_LEN; i++) {
            printf("0x%02x ", uuid->uuid[i]); // 打印128位UUID
        }
    }
}

/* MTU改变回调函数 */
static void ssaps_mtu_changed_cbk(uint8_t server_id, uint16_t conn_id, ssap_exchange_info_t *mtu_size, errcode_t status)
{
    printf("[%s] server_id:%d, conn_id:%d, mtu_size:%d, status:%d\r\n", __FUNCTION__, server_id, conn_id,
           mtu_size->mtu_size, status);
}

/* 服务启动回调函数 */
static void ssaps_start_service_cbk(uint8_t server_id, uint16_t handle, errcode_t status)
{
    printf("[%s] server_id:%d, handle:%x, status:%x\r\n", __FUNCTION__, server_id, handle, status);
}

/* 添加服务回调函数 */
static void ssaps_add_service_cbk(uint8_t server_id, sle_uuid_t *uuid, uint16_t handle, errcode_t status)
{
    printf("[%s] server_id:%x, handle:%x, status:%x\r\n", __FUNCTION__, server_id, handle, status);
    sle_uuid_print(uuid);
}

/* 添加属性回调函数 */
static void ssaps_add_property_cbk(uint8_t server_id,
                                   sle_uuid_t *uuid,
                                   uint16_t service_handle,
                                   uint16_t handle,
                                   errcode_t status)
{
    printf("[%s] server_id:%x, service_handle:%x,handle:%x, status:%x\r\n", __FUNCTION__, server_id, service_handle,
           handle, status);
    sle_uuid_print(uuid);
}

/* 添加描述符回调函数 */
static void ssaps_add_descriptor_cbk(uint8_t server_id,
                                     sle_uuid_t *uuid,
                                     uint16_t service_handle,
                                     uint16_t property_handle,
                                     errcode_t status)
{
    printf("[%s] server_id:%x, service_handle:%x, property_handle:%x,status:%x\r\n", __FUNCTION__, server_id,
           service_handle, property_handle, status);
    sle_uuid_print(uuid);
}

/* 删除所有服务回调函数 */
static void ssaps_delete_all_service_cbk(uint8_t server_id, errcode_t status)
{
    printf("[%s] server_id:%x, status:%x\r\n", __FUNCTION__, server_id, status);
}

/* 读请求回调函数 */
static void ssaps_read_request_cbk(uint8_t server_id,
                                   uint16_t conn_id,
                                   ssaps_req_read_cb_t *read_cb_para,
                                   errcode_t status)
{
    printf("[%s] server_id:%x, conn_id:%x, handle:%x, status:%x\r\n", __FUNCTION__, server_id, conn_id,
           read_cb_para->handle, status);
}

/* 写请求回调函数（处理客户端命令） */
static void ssaps_write_request_cbk(uint8_t server_id,
                                    uint16_t conn_id,
                                    ssaps_req_write_cb_t *write_cb_para,
                                    errcode_t status)
{
    unused(status);
    unused(conn_id);
    unused(server_id); // 标记未使用的参数

    // 1. 创建缓冲区保存接收到的命令
    static char received_command[64] = {0}; // 静态变量保存最新命令
    uint16_t copy_len = write_cb_para->length;

    // 2. 确保不超过缓冲区大小
    if (copy_len > sizeof(received_command) - 1) {
        copy_len = sizeof(received_command) - 1;
    }

    // 3. 拷贝数据并添加字符串终止符
    if (copy_len > 0 && write_cb_para->value) {
        memcpy(received_command, write_cb_para->value, copy_len);
        received_command[copy_len] = '\0'; // 确保字符串正确终止

        printf("收到命令: %s\r\n", received_command);

        // 4. 根据命令内容执行相应操作
        if (strcmp(received_command, "关机") == 0) {
            printf("执行打开电机...\r\n");
            uapi_gpio_set_val(GPIO_08, GPIO_LEVEL_LOW);
            uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_HIGH);
        } else if (strcmp(received_command, "主菜单") == 0) {
            printf("执行关闭电机...\r\n");
            uapi_gpio_set_val(GPIO_08, GPIO_LEVEL_LOW);
            uapi_gpio_set_val(GPIO_09, GPIO_LEVEL_LOW);
        } else if (strcmp(received_command, "前") == 0) {
            printf("执行打开绿灯...\r\n");
            AW2013_Control_Green(RGB_ON);
        } else if (strcmp(received_command, "左") == 0) {
            printf("执行打开红灯...\r\n");
            AW2013_Control_Red(RGB_ON);
        } else if (strcmp(received_command, "右") == 0) {
            printf("执行打开蓝灯...\r\n");
            AW2013_Control_Blue(RGB_ON);
        } else if (strcmp(received_command, "OK") == 0) {
            printf("执行关闭RGB...\r\n");
            AW2013_Control_RGB(RGB_OFF, RGB_OFF, RGB_OFF);
        } else if (strcmp(received_command, "后") == 0) {
            printf("执行关闭绿灯...\r\n");
            AW2013_Control_Green(RGB_OFF);
        } else if (strcmp(received_command, "减") == 0) {
            printf("执行关闭红灯...\r\n");
            AW2013_Control_Red(RGB_OFF);
        } else if (strcmp(received_command, "加") == 0) {
            printf("执行关闭蓝灯...\r\n");
            AW2013_Control_Blue(RGB_OFF);
        } else if (strcmp(received_command, "设置") == 0) {
            printf("执行打开蜂鸣器...\r\n");
            uapi_gpio_set_val(GPIO_10, GPIO_LEVEL_HIGH); // 打开蜂鸣器
        } else if (strcmp(received_command, "返回") == 0) {
            printf("执行关闭蜂鸣器...\r\n");
            uapi_gpio_set_val(GPIO_10, GPIO_LEVEL_LOW); // 关闭蜂鸣器
        }
        // 其他命令处理...
        else {
            printf("未知命令: %s\r\n", received_command);
        }
    }
}

/* 注册SLE服务回调函数 */
static errcode_t sle_ssaps_register_cbks(void)
{
    errcode_t ret;
    ssaps_callbacks_t ssaps_cbk = {0};
    ssaps_cbk.add_service_cb = ssaps_add_service_cbk;
    ssaps_cbk.add_property_cb = ssaps_add_property_cbk;
    ssaps_cbk.add_descriptor_cb = ssaps_add_descriptor_cbk;
    ssaps_cbk.start_service_cb = ssaps_start_service_cbk;
    ssaps_cbk.delete_all_service_cb = ssaps_delete_all_service_cbk;
    ssaps_cbk.mtu_changed_cb = ssaps_mtu_changed_cbk;
    ssaps_cbk.read_request_cb = ssaps_read_request_cbk;
    ssaps_cbk.write_request_cb = ssaps_write_request_cbk;
    ret = ssaps_register_callbacks(&ssaps_cbk);
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}

/* 添加SLE服务UUID */
static errcode_t sle_uuid_server_service_add(void)
{
    errcode_t ret;
    sle_uuid_t service_uuid = {0};
    sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &service_uuid);
    ret = ssaps_add_service_sync(g_server_id, &service_uuid, 1, &g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] fail, ret:%x\r\n", __FUNCTION__, ret);
        return ERRCODE_SLE_FAIL;
    }
    return ERRCODE_SLE_SUCCESS;
}

/* 添加SLE服务属性 */
static errcode_t sle_uuid_server_property_add(void)
{
    errcode_t ret;
    ssaps_property_info_t property = {0};
    ssaps_desc_info_t descriptor = {0};
    uint8_t ntf_value[] = {0x01, 0x0};

    property.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    sle_uuid_setu2(SLE_UUID_SERVER_NTF_REPORT, &property.uuid);
    property.value = (uint8_t *)osal_vmalloc(sizeof(g_sle_property_value));
    if (property.value == NULL) {
        printf("[%s] property mem fail.\r\n", __FUNCTION__);
        return ERRCODE_SLE_FAIL;
    }
    if (memcpy_s(property.value, sizeof(g_sle_property_value), g_sle_property_value, sizeof(g_sle_property_value)) !=
        EOK) {
        printf("[%s] property mem cpy fail.\r\n", __FUNCTION__);
        osal_vfree(property.value);
        return ERRCODE_SLE_FAIL;
    }
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_property_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] fail, ret:%x\r\n", __FUNCTION__, ret);
        osal_vfree(property.value);
        return ERRCODE_SLE_FAIL;
    }
    descriptor.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    descriptor.type = SSAP_DESCRIPTOR_USER_DESCRIPTION;
    descriptor.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    descriptor.value = ntf_value;
    descriptor.value_len = sizeof(ntf_value);

    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] fail, ret:%x\r\n", __FUNCTION__, ret);
        osal_vfree(property.value);
        osal_vfree(descriptor.value);
        return ERRCODE_SLE_FAIL;
    }
    osal_vfree(property.value);
    return ERRCODE_SLE_SUCCESS;
}

/* 添加SLE服务器 */
static errcode_t sle_server_add(void)
{
    errcode_t ret;
    sle_uuid_t app_uuid = {0};

    printf("[%s] in\r\n", __FUNCTION__);
    app_uuid.len = sizeof(g_sle_uuid_app_uuid);
    if (memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid)) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    ssaps_register_server(&app_uuid, &g_server_id);

    if (sle_uuid_server_service_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    if (sle_uuid_server_property_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    printf("[%s] server_id:%x, service_handle:%x, property_handle:%x\r\n", __FUNCTION__, g_server_id, g_service_handle,
           g_property_handle);
    ret = ssaps_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] fail, ret:%x\r\n", __FUNCTION__, ret);
        return ERRCODE_SLE_FAIL;
    }
    printf("[%s] out\r\n", __FUNCTION__);
    return ERRCODE_SLE_SUCCESS;
}

/* 通过UUID向主机发送报告数据 */
errcode_t sle_server_send_report_by_uuid(msg_data_t msg_data)
{
    ssaps_ntf_ind_by_uuid_t param = {0};
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.start_handle = g_service_handle;
    param.end_handle = g_property_handle;
    param.value_len = msg_data.value_len;
    param.value = msg_data.value;
    sle_uuid_setu2(SLE_UUID_SERVER_NTF_REPORT, &param.uuid);
    ssaps_notify_indicate_by_uuid(g_server_id, g_sle_conn_hdl, &param);
    return ERRCODE_SLE_SUCCESS;
}

/* 通过句柄向主机发送报告数据 */
errcode_t sle_server_send_report_by_handle(msg_data_t msg_data)
{
    ssaps_ntf_ind_t param = {0};
    param.handle = g_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = msg_data.value;
    param.value_len = msg_data.value_len;
    return ssaps_notify_indicate(g_server_id, g_sle_conn_hdl, &param);
}

/* 连接状态改变回调函数 */
static void sle_connect_state_changed_cbk(uint16_t conn_id,const sle_addr_t *addr, sle_acb_state_t conn_state, sle_pair_state_t pair_state,sle_disc_reason_t disc_reason)
{
    printf("[%s] conn_id:0x%02x, conn_state:0x%x, pair_state:0x%x, disc_reason:0x%x\r\n", __FUNCTION__, conn_id,
           conn_state, pair_state, disc_reason);
    printf("[%s] addr:%02x:**:**:**:%02x:%02x\r\n", __FUNCTION__, addr->addr[0], addr->addr[4]); /* 0 4: 蓝牙地址索引 */
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        g_sle_conn_hdl = conn_id;
        sle_set_data_len(conn_id, DEFAULT_PAYLOAD_SIZE); // 设置有效载荷大小
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        g_sle_conn_hdl = 0;
        sle_start_announce(SLE_ADV_HANDLE_DEFAULT); // 断开后重新开始广播
    }
}

/* 配对完成回调函数 */
static void sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    printf("[%s] conn_id:%02x, status:%x\r\n", __FUNCTION__, conn_id, status);
    printf("[%s] addr:%02x:**:**:**:%02x:%02x\r\n", __FUNCTION__, addr->addr[0], addr->addr[4]); /* 0 4: 蓝牙地址索引 */
}

/* 注册连接回调函数 */
static errcode_t sle_conn_register_cbks(void)
{
    errcode_t ret;
    sle_connection_callbacks_t conn_cbks = {0};
    conn_cbks.connect_state_changed_cb = sle_connect_state_changed_cbk;
    conn_cbks.pair_complete_cb = sle_pair_complete_cbk;
    ret = sle_connection_register_callbacks(&conn_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}

/* 初始化SLE服务器 */
errcode_t sle_server_init(void)
{
    errcode_t ret;

    /* 启用SLE功能 */
    if (enable_sle() != ERRCODE_SUCC) {
        printf("sle enbale fail !\r\n");
        return ERRCODE_FAIL;
    }

    ret = sle_announce_register_cbks();
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] sle_announce_register_cbks fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    ret = sle_conn_register_cbks();
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] sle_conn_register_cbks fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    ret = sle_ssaps_register_cbks();
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] sle_ssaps_register_cbks fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    ret = sle_server_add();
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] sle_server_add fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    ssap_exchange_info_t parameter = {0};
    parameter.mtu_size = SLE_MTU_SIZE_DEFAULT;
    parameter.version = 1;
    ssaps_set_info(g_server_id, &parameter);
    ret = sle_server_adv_init();
    if (ret != ERRCODE_SLE_SUCCESS) {
        printf("[%s] sle_server_adv_init fail :%x\r\n", __FUNCTION__, ret);
        return ret;
    }
    printf("[%s] init ok\r\n", __FUNCTION__);
    return ERRCODE_SLE_SUCCESS;
}
