#include "stm32f10x.h"                  // 设备头文件
#include "MX30102_Reg.h"
#include "MyI2C.h"
#include "Delay.h"
#include "MX30102.h"

// 设备地址（7位地址0x57，左移一位得到读/写地址）
#define MAX30102_WRITE_ADDR  0xAE   // 0x57 << 1
#define MAX30102_READ_ADDR   0xAF   // (0x57 << 1) | 0x01

// FIFO 深度，固定32个采样
#define FIFO_DEPTH           32

// 每个采样数据的字节数（红光3字节 + 红外3字节）
#define SAMPLE_BYTES         6

// ==============================================
// 底层读写函数
// ==============================================

/**
 * @brief 向 MAX30102 寄存器写入一个字节
 * @param reg  寄存器地址
 * @param data 要写入的数据
 * @retval 0 成功，1 失败（收到 NACK）
 */
uint8_t MAX30102_WriteRegister(uint8_t reg, uint8_t data)
{
    MX30102_I2C_Start();
    MX30102_I2C_SendByte(MAX30102_WRITE_ADDR);
    if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 1; }

    MX30102_I2C_SendByte(reg);
    if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 1; }

    MX30102_I2C_SendByte(data);
    if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 1; }

    MX30102_I2C_Stop();
    return 0;
}

/**
 * @brief 从 MAX30102 寄存器读取一个字节
 * @param reg 寄存器地址
 * @retval 读取到的值，失败返回 0xFF
 */
uint8_t MAX30102_ReadRegister(uint8_t reg)
{
    uint8_t value = 0xFF;

    // 1. 发送寄存器地址（写操作）
    MX30102_I2C_Start();
    MX30102_I2C_SendByte(MAX30102_WRITE_ADDR);
    if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 0xFF; }
    MX30102_I2C_SendByte(reg);
    if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 0xFF; }
    MX30102_I2C_Stop();  // 释放总线

    // 2. 重新开始，读取数据
    MX30102_I2C_Start();
    MX30102_I2C_SendByte(MAX30102_READ_ADDR);
    if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 0xFF; }
    value = MX30102_ReceiveByte();
    MX30102_I2C_SendAck(1);  // 发送 NACK 结束读取
    MX30102_I2C_Stop();

    return value;
}

// ==============================================
// 逻辑功能函数
// ==============================================

/**
 * @brief 获取芯片的 Part ID，用于验证通信
 * @retval Part ID，应为 0x15
 */
uint8_t MAX30102_GetPartID(void)
{
    return MAX30102_ReadRegister(REG_PART_ID);
}

/**
 * @brief 获取 Revision ID
 * @retval Revision ID
 */
uint8_t MAX30102_GetRevisionID(void)
{
    return MAX30102_ReadRegister(REG_REV_ID);
}

/**
 * @brief 复位 MAX30102
 */
void MAX30102_Reset(void)
{
    MAX30102_WriteRegister(REG_MODE_CONFIG, 0x40);  // 置位 RST 位
    Delay_ms(100);  // 等待复位完成（典型值 ~10ms，这里放宽）
}

/**
 * @brief 初始化 MAX30102 为 SpO2/心率模式
 * @note  采样率 100Hz，脉宽 400us，ADC 满量程 4096 nA，LED 电流约 13mA
 */
void MAX30102_Init(void)
{
    // 1. 复位芯片
	MX30102_I2C_Init();
    // 等待芯片上电稳定（这里简略）
    Delay_ms(100);
    MAX30102_Reset();

    // 2. 关闭所有中断，使用查询方式读取
    MAX30102_WriteRegister(REG_INTR_ENABLE_1, 0x00);
    MAX30102_WriteRegister(REG_INTR_ENABLE_2, 0x00);

    // 3. 配置 FIFO
    //    - 平均采样 = 1（不平均）
    //    - FIFO 回卷等参数保持默认
    MAX30102_WriteRegister(REG_FIFO_CONFIG, 0x00);
    MAX30102_WriteRegister(REG_FIFO_WR_PTR, 0x00);
    MAX30102_WriteRegister(REG_OVF_COUNTER, 0x00);
    MAX30102_WriteRegister(REG_FIFO_RD_PTR, 0x00);

    // 4. 配置 SpO2 模式
    //    - ADC_RGE = 00 (4096 nA)
    //    - SR     = 011 (100 Hz)
    //    - LED_PW = 011 (400 us)
    //    组合成 0b01101100 = 0x6C
    MAX30102_WriteRegister(REG_SPO2_CONFIG, 0x6C);

    // 5. 配置 LED 电流（红光和红外，默认值 0x1F ~ 13mA）
    MAX30102_WriteRegister(REG_LED1_PA, 0x1F);
    MAX30102_WriteRegister(REG_LED2_PA, 0x1F);
    MAX30102_WriteRegister(REG_PILOT_PA, 0x00);

    // 6. 设置为 SpO2 模式（MODE[2:0] = 011）
    MAX30102_WriteRegister(REG_MODE_CONFIG, 0x03);

    // 7. 等待配置生效
    Delay_ms(100);
}

/**
 * @brief 检查 FIFO 是否有新数据
 * @retval 1 有数据，0 无数据
 */
uint8_t MAX30102_IsDataReady(void)
{
    uint8_t wr_ptr = MAX30102_ReadRegister(REG_FIFO_WR_PTR);
    uint8_t rd_ptr = MAX30102_ReadRegister(REG_FIFO_RD_PTR);
    return (wr_ptr != rd_ptr);
}

/**
 * @brief 读取 FIFO 中的所有可用采样（最多 32 个）
 * @param red   红光数据的数组（需足够大）
 * @param ir    红外数据的数组（需足够大）
 * @param count 返回实际读取的采样数
 * @retval 0 成功，1 无数据或失败
 */
uint8_t MAX30102_ReadFIFO(uint32_t *red, uint32_t *ir, uint8_t *count)
{
    uint8_t wr_ptr, rd_ptr;
    uint8_t num_samples;

    // 获取当前的写/读指针
    wr_ptr = MAX30102_ReadRegister(REG_FIFO_WR_PTR) & 0x1F;
    rd_ptr = MAX30102_ReadRegister(REG_FIFO_RD_PTR) & 0x1F;

    // 计算有效采样数（处理回卷情况）
    if (wr_ptr >= rd_ptr)
        num_samples = wr_ptr - rd_ptr;
    else
        num_samples = FIFO_DEPTH - rd_ptr + wr_ptr;

    if (num_samples == 0) {
        *count = 0;
        return 1;  // 无数据
    }

    // 最多一次读取 32 个采样（实际不会超过）
    if (num_samples > FIFO_DEPTH)
        num_samples = FIFO_DEPTH;

    // 依次读取
    for (uint8_t i = 0; i < num_samples; i++) {
        uint8_t data[SAMPLE_BYTES];

        // --- 写寄存器地址（指向 FIFO_DATA）---
        MX30102_I2C_Start();
        MX30102_I2C_SendByte(MAX30102_WRITE_ADDR);
        if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 1; }
        MX30102_I2C_SendByte(REG_FIFO_DATA);
        if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 1; }
        MX30102_I2C_Stop();        // 释放总线

        // --- 开始读取 6 个字节 ---
        MX30102_I2C_Start();
        MX30102_I2C_SendByte(MAX30102_READ_ADDR);
        if (MX30102_I2C_ReceiveAck()) { MX30102_I2C_Stop(); return 1; }

        for (uint8_t j = 0; j < SAMPLE_BYTES; j++) {
            data[j] = MX30102_ReceiveByte();
            if (j < SAMPLE_BYTES - 1)
                MX30102_I2C_SendAck(0);  // ACK
            else
                MX30102_I2C_SendAck(1);  // NACK 结束
        }
        MX30102_I2C_Stop();

        // 组合成 18 位值（高位在前）
        red[i] = (((uint32_t)data[0] & 0x03) << 16) | ((uint32_t)data[1] << 8) | data[2];
        ir[i]  = (((uint32_t)data[3] & 0x03) << 16) | ((uint32_t)data[4] << 8) | data[5];
    }

    *count = num_samples;
    return 0;
}

/**
 * @brief 读取 MAX30102 芯片内部温度
 * @param tempCenti 输出温度，单位 0.01 度
 * @retval 0 成功，1 失败
 */
uint8_t MAX30102_ReadTemp(int16_t *tempCenti)
{
    int8_t integer;
    uint8_t frac;

    MAX30102_WriteRegister(REG_TEMP_CONFIG, 0x01);  /* 温度传感器开始转换 */
    Delay_ms(50);

    integer = (int8_t)MAX30102_ReadRegister(REG_TEMP_INTR);
    frac    = MAX30102_ReadRegister(REG_TEMP_FRAC);

    /* 温度 = integer + frac * 0.0625 C */
    *tempCenti = (int16_t)integer * 100 + (int16_t)((frac * 25) / 4);
    return 0;
}
