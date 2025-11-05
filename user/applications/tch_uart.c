
#include "tch_uart.h"
#include "string.h"

struct uart_ops uart_ops_1 = {
    .putc = &drv_putc,
    .putc_sz = &drv_putc_size,
    .configure = &drv_uart_config,
    .getc = &drv_getc};

struct serial_device serial_device_1 = {
    .uart = 0,
    .ops = &uart_ops_1,
    .config = 0};

/**
 * @brief 初始化外设信息
 *        需要在main初始化
 */
void serial_device_1_init(void)
{
    // 初始化硬件配置信息
    serial_device_1.uart.name = "usart1";
    serial_device_1.uart.irqn = USART1_IRQn;
    serial_device_1.uart.uart = USART1;
    serial_device_1.uart.rx_pin = GPIO_PINS_10;
    serial_device_1.uart.tx_pin = GPIO_PINS_9;
    serial_device_1.uart.rx_port = GPIOA;
    serial_device_1.uart.tx_port = GPIOA;

    // 初始化串口配置信息
    serial_device_1.config.baud_rate = 9600;
    serial_device_1.config.bufsz = MB_UART_RX_MAX_SIZE;
    serial_device_1.config.data_bits = USART_DATA_8BITS;
    serial_device_1.config.irq_priority = 7;
    serial_device_1.config.parity = USART_PARITY_NONE;
    serial_device_1.config.reserved = 0;
    serial_device_1.config.stop_bits = USART_STOP_1_BIT;

    // 初始化接收句柄
    drv_serial_1_rx_handler_init();

    // 初始化
    serial_device_1.ops->configure(&serial_device_1, &(serial_device_1.config));

    // serial_device_1.ops->putc_sz(&serial_device_1 , "test\r\n" , 6);
}

/**
 * @brief 选择对应的串口外设初始化
 */
void serial_device_init(const char *name)
{
    if (strcmp(name, "usart1") == 0) // 使用strcmp比较内容
    {
        /* 处理usart1的初始化代码 */
        serial_device_1_init();
    }
    else if (strcmp(name, "usart2") == 0)
    {
        /* 处理usart2的初始化代码 */
    }
}

/**
 * @brief 获取到串口指针
 */
struct serial_device *tch_get_serial(const char *name)
{
    if (strcmp(name, "usart1") == 0) // 使用strcmp比较内容
    {
        return &serial_device_1;
    }
    else if (strcmp(name, "usart2") == 0)
    {
    }
    return NULL;
}

/**
 * @brief 清除对应串口上的数据
 */
int tch_serial_rx_reset(struct serial_device *px_serial)
{
    struct serial_rx_configure *px_serial_rx = NULL;
    if (strcmp((px_serial->uart.name), "usart1") == 0)
    {
        px_serial_rx = drv_get_serial_fifo_1();
    }
    else if (strcmp(px_serial->uart.name, "usart2") == 0)
    {
        px_serial_rx = NULL;
    }
    if (px_serial_rx == NULL)
    {
        return FALSE;
    }
    drv_serial_rx_clear(px_serial_rx);
    return TRUE;
}
