#ifndef __MYI2C_H
#define __MYI2C_H

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

void MX30102_I2C_Init(void);
void MX30102_I2C_Start(void);
void MX30102_I2C_Stop(void);
void MX30102_I2C_SendByte(uint8_t Byte);
uint8_t MX30102_ReceiveByte(void);
void MX30102_I2C_SendAck(uint8_t AckBit);
uint8_t MX30102_I2C_ReceiveAck(void);

#endif
