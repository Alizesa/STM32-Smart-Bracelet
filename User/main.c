/* STM32 SmartWatch main - FreeRTOS application
 *
 * Peripherals:
 *   OLED 0.96"        I2C      PB8/PB9
 *   MPU6050           I2C      PB10/PB11  (pedometer / wrist-raise / fall)
 *   MAX30102          I2C      PB6/PB7    (heart rate)
 *   DHT11             1-wire   PA1        (temperature / humidity)
 *   HC-05 Bluetooth   USART1   PA9/PA10
 *   PN532 NFC         USART2   PA2/PA3
 *   KEY1/2/3          GPIO     PB1/PA6/PA4
 */
#include "stm32f10x.h"
#include <stdio.h>
#include "misc.h"     /* NVIC_PriorityGroupConfig */
#include "Delay.h"
#include "MyRTC.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MX30102.h"
#include "DHT11.h"
#include "HC05.h"
#include "PN532.h"
#include "Key.h"
#include "Pedometer.h"
#include "Motion.h"
#include "Menu.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---------------- shared data ---------------- */
/* (Menu.c 通过 extern 引用以下变量) */
volatile uint8_t  gHumidity;
volatile uint8_t  gTemperature;
volatile uint8_t  gDhtError;
volatile uint32_t gSteps;
volatile uint16_t gHeartRate;
volatile uint8_t  gNfcDetected;
volatile uint8_t  gNfcUidLen;
volatile uint8_t  gNfcUid[8];

/* 心率AC波形环形缓冲(供Menu.c绘制实时波形) */
#define HR_WAVE_LEN        64
volatile int16_t  HR_Wave[HR_WAVE_LEN];
volatile uint8_t  HR_WavePos;

/* ---------------- heart-rate detector ---------------- */
#define HR_AMP_THRESHOLD    800     /* AC amplitude above which a beat counts */
#define HR_MIN_INTERVAL_MS  400     /* >= 150 bpm */
#define HR_MAX_INTERVAL_MS  2000    /* >= 30 bpm */

static int32_t  hrDc;
static int32_t  hrLastAc;
static uint16_t hrBpm;
static uint32_t hrLastBeatMs;

static void HeartRate_Feed(int32_t ir)
{
	int32_t ac;

	if (hrDc == 0)
	{
		hrDc = ir;
	}
	hrDc += (ir - hrDc) >> 5;           /* DC / ambient light removal */
	ac = ir - hrDc;

	if ((ac > HR_AMP_THRESHOLD) && (hrLastAc <= HR_AMP_THRESHOLD))
	{
		uint32_t now = xTaskGetTickCount();
		uint32_t dt = now - hrLastBeatMs;

		if ((dt >= HR_MIN_INTERVAL_MS) && (dt <= HR_MAX_INTERVAL_MS))
		{
			hrBpm = (uint16_t)(60000UL / dt);
		}
		hrLastBeatMs = now;
	}
	hrLastAc = ac;

	/* 记录AC波形, 供OLED实时绘制 */
	if (ac > 4096) { ac = 4096; }
	else if (ac < -4096) { ac = -4096; }
	HR_Wave[HR_WavePos] = (int16_t)ac;
	HR_WavePos = (uint8_t)((HR_WavePos + 1) % HR_WAVE_LEN);
}

/* ---------------- tasks ---------------- */

/* reads MPU6050 -> pedometer, and DHT11 every 2s */
void Sensor_Task(void *pvParameters)
{
	int16_t ax, ay, az, gx, gy, gz;
	uint8_t hum = 0, temp = 0;
	uint32_t lastDht = 0;

	/* DHT11 needs at least 1s after power-up before its first sample. */
	vTaskDelay(pdMS_TO_TICKS(1000));

	for (;;)
	{
		MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
		Pedometer_Update(ax, ay, az, (uint32_t)xTaskGetTickCount());
		Motion_Update(ax, ay, az, (uint32_t)xTaskGetTickCount());
		gSteps = Pedometer_GetSteps();

		if (((uint32_t)xTaskGetTickCount() - lastDht) >= 2000)
		{
			lastDht = (uint32_t)xTaskGetTickCount();
			if (DHT11_Read(&hum, &temp) == 0)
			{
				gDhtError = 0;
				gHumidity = hum;
				gTemperature = temp;
			}
			else
			{
				gDhtError = DHT11_GetLastError();
			}
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

/* reads MAX30102 FIFO and computes a simple heart-rate */
void HeartRate_Task(void *pvParameters)
{
	uint32_t red[32];
	uint32_t ir[32];
	uint8_t count, i;

	for (;;)
	{
		if (MAX30102_IsDataReady())
		{
			if (MAX30102_ReadFIFO(red, ir, &count) == 0)
			{
				for (i = 0; i < count; i++)
				{
					HeartRate_Feed((int32_t)ir[i]);
				}
				gHeartRate = hrBpm;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

/* polls the PN532 once per second and caches the detected card UID */
void NFC_Task(void *pvParameters)
{
	uint8_t uid[8];
	uint8_t uidLen = 0;
	uint8_t nfcReady = 0;
	uint32_t lastPoll = 0;
	uint8_t i;

	(void)pvParameters;

	for (;;)
	{
		/* SAM configuration must run after the scheduler has started. */
		if (!nfcReady)
		{
			nfcReady = (PN532_SAMConfig() == 0);
			if (!nfcReady)
			{
				vTaskDelay(pdMS_TO_TICKS(500));
				continue;
			}
		}

		if (((uint32_t)xTaskGetTickCount() - lastPoll) >= 1000)
		{
			lastPoll = (uint32_t)xTaskGetTickCount();

			if (PN532_ReadPassiveTargetID(uid, &uidLen, 1200) == 0)
			{
				taskENTER_CRITICAL();
				gNfcUidLen = uidLen;
				for (i = 0; i < uidLen && i < 8; i++)
				{
					gNfcUid[i] = uid[i];
				}
				gNfcDetected = 1;
				taskEXIT_CRITICAL();
			}
			else
			{
				gNfcDetected = 0;
				gNfcUidLen = 0;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/* handles incoming HC-05 bluetooth commands */
static void Bluetooth_ProcessCommand(char *cmd)
{
	char buf[96];

	if (cmd[0] == 'A' || cmd[0] == 'a')
	{
		sprintf(buf, "T:%d C H:%d %%\r\n", (int)gTemperature, (int)gHumidity);
		HC05_SendString(buf);
		sprintf(buf, "STEPS:%lu\r\n", (unsigned long)gSteps);
		HC05_SendString(buf);
		sprintf(buf, "HR:%u\r\n", (unsigned int)gHeartRate);
		HC05_SendString(buf);
		if (gNfcDetected)
		{
			HC05_SendString("NFC:YES\r\n");
		}
		else
		{
			HC05_SendString("NFC:NO\r\n");
		}
	}
	else if (cmd[0] == 'T' || cmd[0] == 't')
	{
		sprintf(buf, "T:%d C H:%d %%\r\n", (int)gTemperature, (int)gHumidity);
		HC05_SendString(buf);
	}
	else if (cmd[0] == 'S' || cmd[0] == 's')
	{
		sprintf(buf, "STEPS:%lu\r\n", (unsigned long)gSteps);
		HC05_SendString(buf);
	}
	else if (cmd[0] == 'H' || cmd[0] == 'h')
	{
		sprintf(buf, "HR:%u\r\n", (unsigned int)gHeartRate);
		HC05_SendString(buf);
	}
	else if (cmd[0] == 'N' || cmd[0] == 'n')
	{
		if (gNfcDetected)
		{
			uint8_t i;
			HC05_SendString("NFC:YES UID:");
			for (i = 0; i < gNfcUidLen; i++)
			{
				sprintf(buf, "%02X", gNfcUid[i]);
				HC05_SendString(buf);
			}
			HC05_SendString("\r\n");
		}
		else
		{
			HC05_SendString("NFC:NO\r\n");
		}
	}
}

void Bluetooth_Task(void *pvParameters)
{
	char rxBuf[64];
	char txBuf[96];
	uint8_t idx = 0;
	uint32_t lastReport = 0;

	for (;;)
	{
		if (HC05_RxAvailable() > 0)
		{
			char ch = (char)HC05_ReceiveByte(pdMS_TO_TICKS(50));

			if (ch == '\r' || ch == '\n')
			{
				if (idx > 0)
				{
					rxBuf[idx] = '\0';
					Bluetooth_ProcessCommand(rxBuf);
					idx = 0;
				}
			}
			else if (idx < (sizeof(rxBuf) - 1))
			{
				rxBuf[idx++] = ch;
			}
		}

		/* 每5秒主动向手机上报外设监测数据(心率/计步/温湿度/NFC/抬腕/跌倒) */
		if (((uint32_t)xTaskGetTickCount() - lastReport) >= 5000)
		{
			lastReport = (uint32_t)xTaskGetTickCount();
			sprintf(txBuf, "DATA:HR=%u STEPS=%lu T=%d H=%d NFC=%s RAISE=%u FALL=%u\r\n",
					(unsigned int)gHeartRate,
					(unsigned long)gSteps,
					(int)gTemperature,
					(int)gHumidity,
					gNfcDetected ? "YES" : "NO",
					(unsigned int)(Motion_IsRaised() ? 1u : 0u),
					(unsigned int)(Motion_IsFall() ? 1u : 0u));
			HC05_SendString(txBuf);
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

/* ---------------- OLED UI (由 Hardware/Menu.c 的 Menu_Task 实现) ---------------- */

/* ---------------- FreeRTOS hooks ---------------- */
void vApplicationMallocFailedHook(void)
{
	while (1)
	{
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	while (1)
	{
	}
}

/* ---------------- main ---------------- */
int main(void)
{
	DWT_Init();                 /* DWT cycle counter for Delay_us()/Delay_ms() */

	/* 必须: NVIC优先级分组=4位全抢占(FreeRTOS要求; STM32库NVIC_Init依赖此分组才能正确编码优先级) */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	MyRTC_Init();               /* hardware RTC (LSE) */
	OLED_Init();                /* PB8/PB9 */
	Key_Init();                 /* PB1/PA6/PA4 */
	MPU6050_Init();             /* PB10/PB11 */
	MAX30102_Init();            /* PB6/PB7 */
	DHT11_Init();               /* PA1 */
	HC05_Init();                /* USART1 PA9/PA10 */
	PN532_Init();               /* USART2 PA2/PA3 */
	Pedometer_Init();
	Motion_Init();

	/* 抬腕/跌倒硬件中断: MPU6050 INT脚需飞线到 PinMap.h 的 MPU6050_INT_PIN(默认PB12) */
	Motion_IntPinInit();        /* 先配EXTI */
	MPU6050_EnableInt(1, 1);    /* 再使能运动 + 自由落体中断 */

	/* create application tasks */
	xTaskCreate(Key_ScanTask,    "KeyScan",  128, NULL, 3, NULL);
	xTaskCreate(Key_HandlerTask, "KeyHdl",   128, NULL, 2, NULL);
	xTaskCreate(Sensor_Task,     "Sensor",   512, NULL, 2, NULL);
	xTaskCreate(HeartRate_Task,  "Heart",    256, NULL, 2, NULL);
	xTaskCreate(Bluetooth_Task,  "BT",       256, NULL, 1, NULL);
	xTaskCreate(NFC_Task,        "NFC",      256, NULL, 1, NULL);
	xTaskCreate(Menu_Task,       "Menu",     512, NULL, 1, NULL);

	/* start the scheduler (never returns) */
	vTaskStartScheduler();

	while (1)
	{
	}
}
