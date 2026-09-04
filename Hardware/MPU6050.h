#ifndef __MPU6050_H_
#define __MPU6050_H_

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/**
  * @brief  使能MPU6050硬件运动/自由落体中断(INT引脚输出高有效50us脉冲)
  * @param  motionEn    1-使能运动检测中断(抬腕等动作)
  * @param  freeFallEn  1-使能自由落体中断(跌倒初期)
  * @note   阈值需按实际量程/佩戴方式在实机上微调
  */
void MPU6050_EnableInt(uint8_t motionEn, uint8_t freeFallEn);

/**
  * @brief  读取中断状态寄存器(读后硬件自动清除)
  * @retval INT_STATUS值, bit7=运动, bit6=自由落体
  */
uint8_t MPU6050_ReadIntStatus(void);

#endif
