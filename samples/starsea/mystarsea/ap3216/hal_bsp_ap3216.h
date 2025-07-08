#ifndef HAL_BSP_AP3216C_H
#define HAL_BSP_AP3216C_H

#include "cmsis_os2.h"

#define AP3216C_I2C_ADDR 0x1E    // 器件的I2C从机地址
#define AP3216C_I2C_IDX 1        // 模块的I2C总线号
#define AP3216C_I2C_SPEED 100000 // 100KHz
#define I2C_MASTER_ADDR 0x0
/* io */
#define I2C_SCL_MASTER_PIN 16
#define I2C_SDA_MASTER_PIN 15
#define CONFIG_PIN_MODE 2

/**
 * @brief AP3216C 读 接近传感器、光照强度传感器、人体红外传感器的数值
 * @param irData 人体红外传感器
 * @param alsData 光强传感器
 * @param psData 接近传感器
 * @return Returns {@link IOT_SUCCESS} 成功;
 *         Returns {@link IOT_FAILURE} 失败.
 */
uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData);

/**
 * @brief AP3216C 初始化
 * @return Returns {@link IOT_SUCCESS} 成功;
 *         Returns {@link IOT_FAILURE} 失败.
 */
uint32_t AP3216C_Init(void);

#endif // !__HAL_BSP_AP3216C_H__
