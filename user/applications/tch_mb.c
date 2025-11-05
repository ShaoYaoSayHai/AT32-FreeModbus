
#include "tch_mb.h"
#include "crc.h"
#include "tch_uart.h"

struct serial_device *px_serialDevice1 = NULL;

uint8_t uc_mb_buffer[MB_UART_RX_MAX_SIZE];
mb_rtu_rx mb_rtu_handler = {
    .buffer = uc_mb_buffer,
    .size = MB_UART_RX_MAX_SIZE};

func_lists mb_reg_func_lists[] = {
    {0x2001, mb_func_op_electronic_valve},
    {0x2002, mb_func_op_electronic_valve}};

mb_rtu_slave mb_rtu_slave_handler = {
    .address = 0x01,
    .buffer = 0};

struct serial_rx_configure *serial_rx_1 = NULL;

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
        // 此处进行协议的解析
        mb_receive_data_analysis(mb_rtu_handler.buffer, mb_rtu_handler.size);
        // 清空接收
        tch_serial_rx_reset(px_serialDevice1);
    }
}

/**
 * @brief 解析并执行 我姑且称之为MODBUS TCH
 */
void mb_receive_data_analysis(uint8_t *input, uint16_t size)
{
    // 对传入的参数进行解析 计划参照MODBUS的协议方式
    // 地址(1bytes) 指令(1byte) 寄存器地址(2bytes) 数据长度(1byte) 有效数据(n-byte) 校验位高 校验位低
    // 先判断地址是否是本机地址
    if (input[0] != mb_rtu_slave_handler.address)
    {
        // 该处什么也不应该返回，直接不响应
        return;
    }
    else if (input[0] == 0x00)
    {
        // 返回设备地址信息

        return;
    }

    if (input[1] == 0x03)
    {
        /* 此处执行读取指令 不过这次设计貌似没有读取？ */
    }
    else if (input[1] == 0x06)
    {
        /* 此处执行写入指令 通过写入改变对应的功能状态 此后可在此扩展对应的写入到内存的 */
        if (input[4] == 0x00) // 数据长度错误
        {
            return;
        }
        uint8_t uc_data_size = input[4];
        uint8_t uc_total_size = 5 + uc_data_size;                  // 接收到的除了i校验位之外的全部长度
        uint16_t us_crc_code = crc16_MODBUS(input, uc_total_size); // 进行CRC校验
        uint16_t us_crc_load = input[uc_total_size] << 8 | input[uc_total_size + 1];
        if (us_crc_load != us_crc_code)
        {
            return;
        }
        // 接下来就是校验合格的结果
        // 校验通过之后的执行 对应各种指令地址进行
        uint16_t reg_address = input[2] << 8 | input[3];
        int func_list_size = sizeof(mb_reg_func_lists) / sizeof(mb_reg_func_lists[0]);
        for (size_t i = 0; i < func_list_size; i++)
        {
            if (reg_address == mb_reg_func_lists[i].reg_address)
            {
                // 寄存器地址相同执行对应的函数
                mb_reg_func_lists[i].func(&input[5], uc_data_size); // 只传输实际有效数据
            }
        }
    }
    else
    {
        return;
    }
}

void mb_func_op_electronic_valve(uint8_t *buffer, uint16_t size)
{
    if (buffer[0] == 0x01)
    {
        // 执行开阀动作
    }
    else if (buffer[0] == 0x00)
    {
        // 执行关阀动作
    }
}
