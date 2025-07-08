#include "stdio.h"
#include "pinctrl.h"
#include "gpio.h"
#include "i2c.h"
#include "osal_task.h"
#include "securec.h"
#include "hal_bsp_nfc.h"

/**
 * @brief  从Page页中组成NDEF协议的包裹
 * @note
 * @param  *dataBuff: 最终的内容
 * @param  dataBuff_MaxSize: 存储缓冲区的长度
 * @retval
 */
uint32_t get_NDEFDataPackage(uint8_t *dataBuff, const uint16_t dataBuff_MaxSize)
{
    if (dataBuff == NULL || dataBuff_MaxSize <= 0) {
        printf("dataBuff==NULL or dataBuff_MaxSize<=0\r\n");
        return ERRCODE_FAIL;
    }

    uint8_t userMemoryPageNum = 0; // 用户的数据操作页数

    // 算出要取多少页
    if (dataBuff_MaxSize <= NFC_PAGE_SIZE) {
        userMemoryPageNum = 1; // 1页
    } else {
        // 需要访问多少页
        userMemoryPageNum = (dataBuff_MaxSize / NFC_PAGE_SIZE) + ((dataBuff_MaxSize % NFC_PAGE_SIZE) >= 0 ? 1 : 0);
    }

    // 内存拷贝
    uint8_t *p_buff = (uint8_t *)malloc(userMemoryPageNum * NFC_PAGE_SIZE);
    if (p_buff == NULL) {
        printf("p_buff == NULL.\r\n");
        return ERRCODE_FAIL;
    }

    // 读取数据
    for (int i = 0; i < userMemoryPageNum; i++) {
        if (NT3HReadUserData(i) == true) {
            memcpy_s(p_buff + i * NFC_PAGE_SIZE, userMemoryPageNum * NFC_PAGE_SIZE, nfcPageBuffer, NFC_PAGE_SIZE);
        }
    }

    memcpy_s(dataBuff, dataBuff_MaxSize, p_buff, dataBuff_MaxSize);

    free(p_buff);
    p_buff = NULL;

    return ERRCODE_SUCC;
}
