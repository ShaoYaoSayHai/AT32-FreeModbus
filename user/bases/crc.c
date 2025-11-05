#include "crc.h"

/**
 * @brief 计算数据的 CRC-16 MODBUS 校验值
 * @param puc_data 指向输入数据缓冲区的指针
 * @param us_length 输入数据的长度（字节数）
 * @return 计算得到的 CRC-16 MODBUS 校验值
 */
uint16_t crc16_MODBUS(uint8_t *puc_data, uint16_t us_length)
{
    uint16_t crc = 0xFFFF; // 初始化 CRC 寄存器为 0xFFFF
    for (uint16_t i = 0; i < us_length; i++)
    {
        crc ^= (uint16_t)puc_data[i]; // 当前数据字节与 CRC 进行异或
        // 处理每个字节的 8bytes
        for (uint8_t j = 0; j < 8; j++)
        {
            // 判断当前 CRC 的最低位 (LSB) 是否为为1
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001; // 右移一位并与反向多项式 0xA001 异或
            }
            else
            {
                crc >>= 1; // 仅右移一位
            }
        }
    }
    // 注意：MODBUS 协议要求校验值在传输时低字节在前
    // 此函数返回值本身是16位的整型，调用方需确保在组成通信帧时，将其转换为两个字节并以低字节在前（小端序）方式放置。
    // 例如：uint8_t crc_low = (uint8_t)(crc & 0xFF); uint8_t crc_high = (uint8_t)(crc >> 8);
    // 然后将 crc_low 放在帧中先传输，crc_high 随后。
    return crc; 
}