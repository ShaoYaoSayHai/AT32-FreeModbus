/**
 * @file uart_config.h
 * @author Enzo
 * @brief  关于串口信息的配置文件
 * @version 0.1
 * @date 2025-11-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _UART_CONFIG_H_
#define _UART_CONFIG_H_

#include "base_config.h"

#define MB_UART (USART1) // MODBUS串口

#define MB_UART_RX_MAX_SIZE (300)

typedef enum
{
    uart_err,
    uart_ok,

} uart_status;

struct base_uart
{
    usart_type *uart;
    gpio_type *tx_port;
    uint16_t tx_pin;
    gpio_type *rx_port;
    uint16_t rx_pin;
    IRQn_Type irqn;
    char *name;
};

struct serial_configure
{
    uint8_t irq_priority;               // 中断优先级
    usart_data_bit_num_type data_bits;  // 数据位
    usart_stop_bit_num_type stop_bits;  // 停止位
    usart_parity_selection_type parity; // 优先级
    uint32_t bufsz;
    uint32_t reserved;
    uint32_t baud_rate; // 波特率
};

struct serial_rx_configure
{
    /* software fifo */
    uint8_t *buffer;

    uint16_t size;

    uint8_t is_finished;

    uint16_t max_size; // 新增：缓冲区最大容量

    uint16_t timeout_count; // 新增：超时计数器
    uint8_t receiving;      // 新增：接收中标志
};

struct serial_device
{
    struct base_uart uart;

    struct serial_configure config;

    struct uart_ops *ops;

    // struct serial_rx_configure rx_fifo;
};

/**
 * @brief 操作集合
 */
struct uart_ops
{
    int (*configure)(struct serial_device *serial, struct serial_configure *cfg);
    int (*putc)(struct serial_device *serial, char c);
    int (*getc)(struct serial_device *serial, uint8_t *puc_buffer, uint16_t size);
    int (*putc_sz)(struct serial_device *serial, char *buffer, uint16_t size); // 发送数组
};

typedef struct uart_timeout_config
{

} uart_timeout_config;

#endif
