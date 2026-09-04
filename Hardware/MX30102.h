#ifndef __MX30102_H_
#define __MX30102_H_

#include <stdint.h>

uint8_t MAX30102_WriteRegister(uint8_t reg, uint8_t data);
uint8_t MAX30102_ReadRegister(uint8_t reg);

uint8_t MAX30102_GetPartID(void);
uint8_t MAX30102_GetRevisionID(void);
void MAX30102_Reset(void);
void MAX30102_Init(void);
void MAX30102_ClearFIFO(void);

uint8_t MAX30102_IsDataReady(void);
uint8_t MAX30102_ReadFIFO(uint32_t *red, uint32_t *ir, uint8_t *count);

/**
  * @brief  Read the MAX30102 die temperature.
  * @param  tempCenti  output, temperature in 0.01 degree C
  * @retval 0 success, 1 failure
  */
uint8_t MAX30102_ReadTemp(int16_t *tempCenti);

#endif
