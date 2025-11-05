

#ifndef _TCH_MB_H_
#define _TCH_MB_H_

#include "uart_config.h"

#define MB_REG_SIZE (64)

typedef struct mb_rtu_rx
{
    uint8_t *buffer;
    uint16_t size;
} mb_rtu_rx;

typedef struct mb_rtu_slave
{
    uint8_t address;              // 设备地址
    uint16_t buffer[MB_REG_SIZE]; // 保持寄存器
} mb_rtu_slave;

typedef struct func_lists
{
    uint16_t reg_address;
    void (*func)(uint8_t *puc_buffer, uint16_t size);
} func_lists;

/**
 * @brief MDOBUS初始化
 */
void mb_init(void);

/**
 * @brief MODBUS轮询执行
 */
void mb_polling(void);

/**
 * @brief 解析并执行 我姑且称之为MODBUS TCH
 */
void mb_receive_data_analysis(uint8_t *input, uint16_t size);

void mb_func_op_electronic_valve(uint8_t *buffer, uint16_t size);

#endif
