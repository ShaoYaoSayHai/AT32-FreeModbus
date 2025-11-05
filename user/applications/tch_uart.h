

#ifndef _TCH_UART_H_
#define _TCH_UART_H_

#include "drv_uart.h"

/**
 * @brief 初始化外设信息
 *
 */
void serial_device_1_init(void);

/**
 * @brief 选择对应的串口外设初始化
 */
void serial_device_init(const char *name);

/**
 * @brief 获取到串口指针
 */
struct serial_device *tch_get_serial(const char *name);

#endif
