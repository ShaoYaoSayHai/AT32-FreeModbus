
#include "mb_rtu.h"
#include "crc.h"
#include "tch_uart.h"

struct serial_device *px_serialDevice1 = NULL;

uint8_t uc_mb_buffer[MB_UART_RX_MAX_SIZE];
mb_rtu_rx mb_rtu_handler = {
    .buffer = uc_mb_buffer,
    .size = MB_UART_RX_MAX_SIZE};

mb_rtu_slave mb_rtu_slave_handler = {
    .address = 0x01,
    .buffer = 0};

/**
 * @brief MDOBUS初始化
 */
void mb_init(void)
{
    px_serialDevice1 = tch_get_serial("usart1"); // 指针赋值
    serial_device_init("usart1");                // 串口初始化

    mb_rtu_slave_handler.address = 0x01; // 设备地址初始化
}

/**
 * @brief MODBUS轮询执行
 */
void mb_polling(void)
{
    int ret = px_serialDevice1->ops->getc(px_serialDevice1, mb_rtu_handler.buffer, 128);
    if (ret != RESET)
    {
        // 此处处理接收到的MODBUS数据
    }
}
