#ifndef __PEDOMETER_H_
#define __PEDOMETER_H_

#include <stdint.h>

void Pedometer_Init(void);

/**
  * @brief  Feed one accelerometer sample into the step detector.
  * @param  ax, ay, az  raw 16-bit accelerometer data
  * @param  now_ms      current time in ms (e.g. xTaskGetTickCount())
  */
void Pedometer_Update(int16_t ax, int16_t ay, int16_t az, uint32_t now_ms);

uint32_t Pedometer_GetSteps(void);
void Pedometer_Reset(void);

#endif
