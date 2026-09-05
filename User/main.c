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

#define BLUETOOTH_REPORT_INTERVAL_MS 10000

/* ---------------- shared data ---------------- */
/* (Menu.c 通过 extern 引用以下变量) */
volatile uint8_t  gHumidity;
volatile uint8_t  gTemperature;
volatile uint8_t  gDhtError;
volatile uint32_t gSteps;
volatile uint16_t gHeartRate;
volatile uint32_t gHeartRateLastValidMs = 0xFFFFFFFFUL;
volatile uint8_t  gHeartMeasureActive;
volatile uint8_t  gHeartSensorOk;
volatile uint8_t  gHeartFingerPresent;
volatile uint32_t gHeartSampleCount;
volatile uint8_t  gNfcOnline;
volatile uint8_t  gNfcDetected;
volatile uint8_t  gNfcUidLen;
volatile uint8_t  gNfcUid[8];

/* 心率AC波形环形缓冲(供Menu.c绘制实时波形) */
#define HR_WAVE_LEN        64
volatile int16_t  HR_Wave[HR_WAVE_LEN];
volatile uint8_t  HR_WavePos;

/* ---------------- heart-rate detector ---------------- */
#define HR_SAMPLE_PERIOD_MS 10      /* MAX30102 configured at 100 Hz */
#define HR_FINGER_THRESHOLD 1000    /* IR DC level indicating finger contact */
#define HR_AMP_THRESHOLD    80      /* AC amplitude above which a beat counts */
#define HR_MIN_INTERVAL_MS  333     /* <= 180 bpm */
#define HR_MAX_INTERVAL_MS  2000    /* >= 30 bpm */
#define HR_INTERVAL_COUNT   4

static int32_t  hrDc;
static int32_t  hrLastAc;
static uint16_t hrBpm;
static uint32_t hrLastBeatMs;
static uint16_t hrIntervals[HR_INTERVAL_COUNT];
static uint8_t  hrIntervalPos;
static uint8_t  hrIntervalCount;
static uint8_t  hrHasBeat;

static void HeartRate_Reset(void)
{
	hrDc = 0;
	hrLastAc = 0;
	hrBpm = 0;
	hrLastBeatMs = 0;
	hrIntervalPos = 0;
	hrIntervalCount = 0;
	hrHasBeat = 0;
}

static void HeartRate_Feed(int32_t ir, uint32_t sampleMs)
{
	int32_t ac;
	uint8_t i;
	uint32_t intervalSum = 0;

	gHeartSampleCount++;
	if (ir < HR_FINGER_THRESHOLD)
	{
		gHeartFingerPresent = 0;
		HeartRate_Reset();
		return;
	}
	gHeartFingerPresent = 1;

	if (hrDc == 0)
	{
		hrDc = ir;
	}
	hrDc += (ir - hrDc) >> 5;           /* DC / ambient light removal */
	ac = ir - hrDc;

	if ((ac > HR_AMP_THRESHOLD) && (hrLastAc <= HR_AMP_THRESHOLD))
	{
		uint32_t dt = sampleMs - hrLastBeatMs;

		if (!hrHasBeat)
		{
			hrHasBeat = 1;
		}
		else if ((dt >= HR_MIN_INTERVAL_MS) && (dt <= HR_MAX_INTERVAL_MS))
		{
			hrIntervals[hrIntervalPos] = (uint16_t)dt;
			hrIntervalPos = (uint8_t)((hrIntervalPos + 1) % HR_INTERVAL_COUNT);
			if (hrIntervalCount < HR_INTERVAL_COUNT) { hrIntervalCount++; }
			for (i = 0; i < hrIntervalCount; i++)
			{
				intervalSum += hrIntervals[i];
			}
			hrBpm = (uint16_t)(60000UL / (intervalSum / hrIntervalCount));
			gHeartRateLastValidMs = (uint32_t)xTaskGetTickCount();
		}
		hrLastBeatMs = sampleMs;
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
	uint32_t lastSampleMs = 0;
	uint8_t count, i;
	uint8_t sampleTimeValid = 0;
	uint8_t wasMeasuring = 0;

	(void)pvParameters;
	HeartRate_Reset();
	gHeartRate = 0;
	gHeartRateLastValidMs = 0xFFFFFFFFUL;
	gHeartFingerPresent = 0;

	for (;;)
	{
		if (!gHeartMeasureActive)
		{
			wasMeasuring = 0;
			vTaskDelay(pdMS_TO_TICKS(50));
			continue;
		}

		if (!wasMeasuring)
		{
			HeartRate_Reset();
			MAX30102_ClearFIFO();
			gHeartRate = 0;
			gHeartRateLastValidMs = 0xFFFFFFFFUL;
			gHeartFingerPresent = 0;
			sampleTimeValid = 0;
			wasMeasuring = 1;
		}

		if (MAX30102_IsDataReady())
		{
			if (MAX30102_ReadFIFO(red, ir, &count) == 0)
			{
				if (!sampleTimeValid)
				{
					lastSampleMs = (uint32_t)xTaskGetTickCount() -
						(uint32_t)(count - 1) * HR_SAMPLE_PERIOD_MS;
					sampleTimeValid = 1;
				}
				for (i = 0; i < count; i++)
				{
					if (i != 0) { lastSampleMs += HR_SAMPLE_PERIOD_MS; }
					HeartRate_Feed((int32_t)ir[i], lastSampleMs);
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
			gNfcOnline = nfcReady;
			if (!nfcReady)
			{
				vTaskDelay(pdMS_TO_TICKS(500));
				continue;
			}
		}

		if (((uint32_t)xTaskGetTickCount() - lastPoll) >= 500)
		{
			lastPoll = (uint32_t)xTaskGetTickCount();

			if (PN532_ReadPassiveTargetID(uid, &uidLen, 700) == 0)
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
		if (!gNfcOnline)
		{
			HC05_SendString("NFC:OFFLINE CHECK UART\r\n");
		}
		else if (gNfcDetected)
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

	(void)pvParameters;

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

		/* 周期上报监测数据，手机也可随时发送命令查询。 */
		if (((uint32_t)xTaskGetTickCount() - lastReport) >= BLUETOOTH_REPORT_INTERVAL_MS)
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
	gHeartSensorOk = (MAX30102_GetPartID() == 0x15) ? 1 : 0;
	DHT11_Init();               /* PA1 */
	HC05_Init();                /* USART1 PA9/PA10 */
	PN532_Init();               /* USART2 PA2/PA3 */
	Pedometer_Init();
	Motion_Init();

	/* 跌倒检测使用连续加速度采样的软件算法，不依赖 MPU6050 INT 接线。 */

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
