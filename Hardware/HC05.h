#ifndef __HC05_H_
#define __HC05_H_

#include <stdint.h>

void HC05_Init(void);
void HC05_SendByte(uint8_t data);
void HC05_SendString(char *str);
uint16_t HC05_RxAvailable(void);
uint32_t HC05_GetRxByteCount(void);
uint32_t HC05_GetRxErrorCount(void);

/**
  * @brief  Receive one byte with timeout.
  * @param  timeout_ms  timeout in FreeRTOS ticks
  * @retval received byte, or 0 on timeout
  */
uint8_t HC05_ReceiveByte(uint32_t timeout_ms);

/* called from USART1_IRQHandler in stm32f10x_it.c */
void HC05_USART_IRQHandler(void);

/* AT-mode helpers (need EN wiring, see PinMap.h) */
void HC05_EnterATMode(void);
void HC05_ExitATMode(void);

#endif
