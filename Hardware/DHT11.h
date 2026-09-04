#ifndef __DHT11_H_
#define __DHT11_H_

#include <stdint.h>

void DHT11_Init(void);

/**
  * @brief  Read temperature / humidity.
  * @param  pHumidity     humidity integer (0-100 %)
  * @param  pTemperature  temperature integer (0-50 C)
  * @retval 0 success, 1 failure
  */
uint8_t DHT11_Read(uint8_t *pHumidity, uint8_t *pTemperature);

/* 0=success, 1=response low timeout, 2=response high timeout,
 * 3=data low timeout, 4=checksum error. */
uint8_t DHT11_GetLastError(void);

#endif
