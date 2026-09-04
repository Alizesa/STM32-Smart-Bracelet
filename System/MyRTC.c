#include "stm32f10x.h"                  // 设备头文件
#include <time.h>
#include "MyRTC.h"

int16_t MyRTC_Time[] = {2026, 8, 24, 13, 36, 0};	//定义全局的时间数组，里面的数据分别为年、月、日、时、分、秒

void MyRTC_SetTime(void);
void MyRTC_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP,ENABLE);

	PWR_BackupAccessCmd(ENABLE);

	if(BKP_ReadBackupRegister(BKP_DR1)!=0xA5A5)
	{
		RCC_LSEConfig(RCC_LSE_ON);
		{
			uint32_t lseTimeout = 1000000;		//LSE 就绪超时（约1s），避免无晶振时程序卡死
			while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)!=SET)
			{
				if (--lseTimeout == 0) break;
			}
		}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		RCC_RTCCLKCmd(ENABLE);
		RTC_WaitForSynchro();
		RTC_WaitForLastTask();

		RTC_SetPrescaler(32768-1);
		RTC_WaitForLastTask();
		MyRTC_SetTime();
		BKP_WriteBackupRegister(BKP_DR1,0XA5A5);
	}
	else
	{
		RTC_WaitForSynchro();
		RTC_WaitForLastTask();
	}
}

void MyRTC_SetTime(void)
{
	time_t time_cnt;		//定义秒计数器变量
	struct tm time_date;	//定义时间结构体变量

	time_date.tm_year = MyRTC_Time[0] - 1900;		//给时间结构体赋值，转换到年份
	time_date.tm_mon = MyRTC_Time[1] - 1;
	time_date.tm_mday = MyRTC_Time[2];
	time_date.tm_hour = MyRTC_Time[3];
	time_date.tm_min = MyRTC_Time[4];
	time_date.tm_sec = MyRTC_Time[5];

	time_cnt = mktime(&time_date) - 8 * 60 * 60;	//使用mktime函数将时间结构体转换为秒计数器格式
													//- 8 * 60 * 60为北京时间与伦敦时间的时区差值
	RTC_SetCounter(time_cnt);						//将秒计数器写入到RTC的CNT中
	RTC_WaitForLastTask();							//等待上一次操作完成
}

/**
  * 函    数：RTC读取时间
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，RTC硬件电路的时间值会刷新到全局数组
  */
void MyRTC_ReadTime(void)
{
	time_t time_cnt;		//定义秒计数器变量
	struct tm time_date;	//定义时间结构体变量

	time_cnt = RTC_GetCounter() + 8 * 60 * 60;		//获取RTC的CNT，获取当前时间戳
													//+ 8 * 60 * 60为北京时间与伦敦时间的时区差值

	time_date = *localtime(&time_cnt);				//使用localtime函数将时间戳转换为时间结构体格式

	MyRTC_Time[0] = time_date.tm_year + 1900;		//将时间结构体赋值给全局时间数组
	MyRTC_Time[1] = time_date.tm_mon + 1;
	MyRTC_Time[2] = time_date.tm_mday;
	MyRTC_Time[3] = time_date.tm_hour;
	MyRTC_Time[4] = time_date.tm_min;
	MyRTC_Time[5] = time_date.tm_sec;
}
