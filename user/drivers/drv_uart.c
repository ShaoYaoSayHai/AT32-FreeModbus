
#include "drv_uart.h"
#include "stdio.h"

// 用来接收数据的数组
uint8_t usart_1_rx_fifo_buffer[MB_UART_RX_MAX_SIZE];

struct serial_rx_configure serial_1_rx_handler;

void USART1_IRQHandler(void)
{
    // // RX 单次接收完成中断标志位
    // if (usart_interrupt_flag_get(USART1, USART_RDBF_FLAG) != RESET)
    // {
    //     if (serial_1_rx_handler.size > MB_UART_RX_MAX_SIZE)
    //     {
    //         serial_1_rx_handler.size = 0;
    //     }
    //     serial_1_rx_handler.buffer[serial_1_rx_handler.size++] = usart_data_receive((USART1));
    //     // 清除标志位
    //     usart_flag_clear((USART1), USART_RDBF_FLAG);
    // }
    // // 空闲中断
    // if (usart_interrupt_flag_get(USART1, USART_IDLEF_FLAG) != RESET)
    // {
    //     serial_1_rx_handler.is_finished = TRUE; // 设置FIFO标志位
    //     // 清除标志位
    //     usart_flag_clear((USART1), USART_IDLEF_FLAG);
    // }
    // RX 单次接收完成中断标志位
    if (usart_interrupt_flag_get(USART1, USART_RDBF_FLAG) != RESET)
    {
        // 如果是第一次接收数据，启动接收过程
        if (!serial_1_rx_handler.receiving)
        {
            serial_1_rx_handler.receiving = TRUE;
            serial_1_rx_handler.timeout_count = 0; // 重置超时计数器
        }
        else
        {
            // 每次收到新数据都重置超时计数器
            serial_1_rx_handler.timeout_count = 0;
        }
        // 检查缓冲区边界
        if (serial_1_rx_handler.size < serial_1_rx_handler.max_size)
        {
            // 存储接收到的数据
            uint8_t received_data = usart_data_receive(USART1);
            serial_1_rx_handler.buffer[serial_1_rx_handler.size++] = received_data;
        }
        else
        {
            // 缓冲区已满，可以在这里处理缓冲区溢出
            // 例如：丢弃数据或触发错误处理
        }

        // 清除接收标志位
        usart_flag_clear(USART1, USART_RDBF_FLAG);
    }
}

/**
 * @brief  this function handles TMR6 handler.
 * @param  none
 * @retval none
 */
void TMR6_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR6, TMR_OVF_FLAG) != RESET)
    {
        // 检查是否正在接收数据
        if (serial_1_rx_handler.receiving)
        {
            serial_1_rx_handler.timeout_count++;

            // 如果8ms内没有新数据（假设定时器中断为1ms）
            if (serial_1_rx_handler.timeout_count >= 8)
            {
                // 设置接收完成标志
                serial_1_rx_handler.is_finished = TRUE;
                serial_1_rx_handler.receiving = FALSE;
                serial_1_rx_handler.timeout_count = 0;

                // 可选：在这里可以触发回调函数通知主程序
            }
        }
        tmr_flag_clear(TMR6, TMR_OVF_FLAG);
    }
}

/**
 * @brief 初始化接收函数结构体
 */
void drv_serial_1_rx_handler_init(void)
{
    serial_1_rx_handler.buffer = &usart_1_rx_fifo_buffer[0];
    serial_1_rx_handler.size = 0;
    serial_1_rx_handler.is_finished = FALSE;
    serial_1_rx_handler.max_size = MB_UART_RX_MAX_SIZE;
    serial_1_rx_handler.timeout_count = 0;
    serial_1_rx_handler.receiving = FALSE;
}

void drv_serial_rx_clear(struct serial_rx_configure *serial_rx)
{
    for (size_t i = 0; i < serial_rx->size; i++)
    {
        serial_rx->buffer[i] = 0;
    }
    serial_1_rx_handler.size = 0;
    serial_1_rx_handler.is_finished = FALSE;
    serial_1_rx_handler.timeout_count = 0;
    serial_1_rx_handler.receiving = FALSE;
}

/**
 * @brief 获取句柄指针
 */
struct serial_rx_configure *drv_get_serial_fifo_1(void)
{
    return &serial_1_rx_handler;
}

/**
 * 函    数：串口发送一个数组
 * 参    数: usart_x 需要发送数据的串口
 * 参    数：data 要发送数组的首地址
 * 参    数：size 要发送数组的长度
 * 返 回 值：无
 */
void usart_write_block(usart_type *usart_x, uint8_t *data, uint16_t size)
{
    uint16_t i = 0;
    while (usart_flag_get(usart_x, USART_TDBE_FLAG) == RESET)
        ; // 传输数据缓冲区空标志
    usart_flag_clear(usart_x, USART_TDBE_FLAG);
    for (i = 0; i < size; i++)
    {
        usart_data_transmit(usart_x, (uint16_t)data[i]);
        while (usart_flag_get(usart_x, USART_TDC_FLAG) == RESET)
            ;
        usart_flag_clear(usart_x, USART_TDC_FLAG);
    }
}

/**
 * @brief 根据配置结构体内容进行配置
 *
 * @param px_serialDevice
 */
int drv_uart_config(struct serial_device *px_serialDevice, struct serial_configure *cfg)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    /* configure the TX pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    // gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pins = px_serialDevice->uart.tx_pin;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init((px_serialDevice->uart.tx_port), &gpio_init_struct);

    /* configure the RX pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    // gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init_struct.gpio_pins = px_serialDevice->uart.rx_pin;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init((px_serialDevice->uart.rx_port), &gpio_init_struct);

    /* configure param */
    // usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_init(
        (px_serialDevice->uart.uart),
        (cfg->baud_rate),
        (cfg->data_bits),
        (cfg->stop_bits));

    usart_transmitter_enable((px_serialDevice->uart.uart), TRUE);
    usart_receiver_enable((px_serialDevice->uart.uart), TRUE);
    usart_parity_selection_config(
        (px_serialDevice->uart.uart),
        (cfg->parity));
    usart_hardware_flow_control_set((px_serialDevice->uart.uart), USART_HARDWARE_FLOW_NONE);

    /**
     * Users need to configure USART1 interrupt functions according to the actual application.
     * 1. Call the below function to enable the corresponding USART1 interrupt.
     *     --usart_interrupt_enable(...)
     * 2. Add the user's interrupt handler code into the below function in the at32f403a_407_int.c file.
     *     --void USART1_IRQHandler(void)
     */
    nvic_irq_enable(
        (px_serialDevice->uart.irqn), (cfg->irq_priority), 0);                  // 配置中断优先级并使能
    usart_interrupt_enable((px_serialDevice->uart.uart), USART_RDBF_INT, TRUE); // 配置串口接收中断
    // usart_interrupt_enable((px_serialDevice->uart.uart), USART_IDLE_INT, TRUE); // 都默认配置成空闲中断
    usart_enable((px_serialDevice->uart.uart), TRUE);

    return SET;
}

/**
 * @brief 通过串口发送数据
 *
 * @param serial 串口信息总集合
 * @param value  要发送的数据
 */
int drv_putc(struct serial_device *serial, char value)
{
    uint8_t uc_value = (uint8_t)value;
    usart_write_block(serial->uart.uart, &uc_value, 1);
    return SET;
}

/**
 * @brief 按照长度通过串口发送数据
 *
 * @param serial 串口信息总集合
 * @param buffer 待发送数组
 * @param size   要发送的数据长度
 * @return int
 */
int drv_putc_size(struct serial_device *serial, char *buffer, uint16_t size)
{
    usart_write_block(serial->uart.uart, (uint8_t *)buffer, size);
    return SET;
}

/**
 * @brief 依次读取接收FIFO区域数据
 *
 * @param serial
 * @return int RESET 并未读取到数据
 *             SET   已经成功读取到数据
 */
int drv_getc(struct serial_device *serial, uint8_t *data_buf, uint16_t buf_size)
{
    if (data_buf == NULL || buf_size == 0)
    {
        return RESET;
    }

    int ret = RESET;
    if (strcmp((serial->uart.name), "usart1") == 0) // 使用strcmp比较内容
    {
        if (serial_1_rx_handler.is_finished)
        {
            uint16_t copy_size = (serial_1_rx_handler.size < buf_size) ? serial_1_rx_handler.size : buf_size;

            // 复制数据到用户缓冲区
            for (uint16_t i = 0; i < copy_size; i++)
            {
                data_buf[i] = serial_1_rx_handler.buffer[i];
            }
            ret = copy_size;
        }
    }
    else if (strcmp((serial->uart.name), "usart2") == 0)
    {
    }

    return ret;
}

/**
 * @brief 串口重定向
 *
 * @param ch
 * @param file
 * @return int
 */
int fputc(int ch, FILE *file)
{
    usart_write_block((USART1), (uint8_t *)&ch, 1);
    return ch;
}
