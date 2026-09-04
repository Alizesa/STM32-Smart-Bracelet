#ifndef __MYRTC_H_
#define __MYRTC_H_

#include <stdint.h>

/* year, month, day, hour, minute, second */
extern int16_t MyRTC_Time[6];

void MyRTC_Init(void);
void MyRTC_SetTime(void);
void MyRTC_ReadTime(void);

#endif
