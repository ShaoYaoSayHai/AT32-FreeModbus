

#ifndef _CRC_H_
#define _CRC_H_

#include "base_config.h"

/**
 * @brief 计算数据的 CRC-16 MODBUS 校验值
 * @param puc_data 指向输入数据缓冲区的指针
 * @param us_length 输入数据的长度（字节数）
 * @return 计算得到的 CRC-16 MODBUS 校验值
 */
uint16_t crc16_MODBUS(uint8_t *puc_data, uint16_t us_length);

#endif
