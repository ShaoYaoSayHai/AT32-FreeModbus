
#ifndef _DRV_UART_H_
#define _DRV_UART_H_

#include "uart_config.h"

/**
 * @brief 初始化接收函数结构体
 */
void drv_serial_1_rx_handler_init(void);

void drv_serial_rx_clear(struct serial_rx_configure *serial_rx);

/**
 * @brief 获取句柄指针
 */
struct serial_rx_configure *drv_get_serial_fifo_1(void);

/**
 * 函    数：串口发送一个数组
 * 参    数: usart_x 需要发送数据的串口
 * 参    数：data 要发送数组的首地址
 * 参    数：size 要发送数组的长度
 * 返 回 值：无
 */
void usart_write_block(usart_type *usart_x, uint8_t *data, uint16_t size);

/**
 * @brief 根据配置结构体内容进行配置
 *
 * @param px_serialDevice
 */
int drv_uart_config(struct serial_device *px_serialDevice, struct serial_configure *cfg);

/**
 * @brief 通过串口发送数据
 *
 * @param serial 串口信息总集合
 * @param value  要发送的数据
 */
int drv_putc(struct serial_device *serial, char value);

/**
 * @brief 依次读取接收FIFO区域数据
 *
 * @param serial
 * @return int
 */
int drv_getc(struct serial_device *serial, uint8_t *puc_buffer, uint16_t size);

/**
 * @brief 按照长度通过串口发送数据
 *
 * @param serial 串口信息总集合
 * @param buffer 待发送数组
 * @param size   要发送的数据长度
 * @return int
 */
int drv_putc_size(struct serial_device *serial, char *buffer, uint16_t size);

#endif
