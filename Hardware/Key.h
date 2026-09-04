#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

/* key event codes (returned by Key_GetEvent) */
#define KEY1_SHORT      1
#define KEY2_SHORT      2
#define KEY3_SHORT      3
#define KEY3_LONG       4

void Key_Init(void);
void Key_ScanTask(void *pvParameters);
void Key_HandlerTask(void *pvParameters);

/**
  * @brief  Get the last key event (non-blocking, clears it).
  * @retval key code (KEYx_xxx) or 0 if none pending
  */
uint8_t Key_GetEvent(void);

#endif
