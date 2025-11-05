

#ifndef _MB_RTU_H_
#define _MB_RTU_H_

#include "uart_config.h"

#define MB_REG_SIZE (64)

typedef struct mb_rtu_rx
{
    uint8_t *buffer;
    uint16_t size;
} mb_rtu_rx;

typedef struct mb_rtu_slave
{
    uint8_t address;     // 设备地址
    uint16_t buffer[MB_REG_SIZE]; // 保持寄存器
}mb_rtu_slave;

/**
 * @brief MDOBUS初始化
 */
void mb_init(void);

/**
 * @brief MODBUS轮询执行
 */
void mb_polling(void);

#endif
