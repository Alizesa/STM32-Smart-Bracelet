#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Key.h"

#define KEY_DEBOUNCE_MS         20      /* scan period (ms) */
#define KEY_LONG_PRESS_MS       1000    /* key3 long-press threshold (ms) */

static QueueHandle_t xKeyQueue;
static volatile uint8_t LastEvent;

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* enable clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* KEY1 = PB1, input pull-up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* KEY2 = PA6, KEY3 = PA4, input pull-up */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	xKeyQueue = xQueueCreate(5, sizeof(uint8_t));
	LastEvent = 0;
}

void Key_ScanTask(void *pvParameters)
{
	uint8_t prevKeyState = 0;       /* previous scan state */
	uint8_t currKeyState = 0;       /* current scan state */
	uint32_t key3PressTime = 0;     /* key3 press counter (scan periods) */
	const TickType_t xScanPeriod = pdMS_TO_TICKS(KEY_DEBOUNCE_MS);

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, xScanPeriod);

		currKeyState = 0;
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) currKeyState |= 0x01; /* KEY1 */
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0) currKeyState |= 0x02; /* KEY2 */
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0) currKeyState |= 0x04; /* KEY3 */

		/* long-press timer for key3 */
		if (currKeyState & 0x04)
		{
			key3PressTime++;
		}
		else
		{
			key3PressTime = 0;
		}

		/* key release event */
		if (prevKeyState != 0 && currKeyState == 0)
		{
			uint8_t keyCode = 0;

			if (prevKeyState & 0x01)
			{
				keyCode = KEY1_SHORT;
			}
			else if (prevKeyState & 0x02)
			{
				keyCode = KEY2_SHORT;
			}
			else if (prevKeyState & 0x04)
			{
				if (key3PressTime >= (KEY_LONG_PRESS_MS / KEY_DEBOUNCE_MS))
				{
					keyCode = KEY3_LONG;
				}
				else
				{
					keyCode = KEY3_SHORT;
				}
			}

			if (keyCode != 0)
			{
				xQueueSend(xKeyQueue, &keyCode, 0);   /* non-blocking */
			}
		}

		prevKeyState = currKeyState;
	}
}

void Key_HandlerTask(void *pvParameters)
{
	uint8_t keyCode;

	for (;;)
	{
		if (xQueueReceive(xKeyQueue, &keyCode, portMAX_DELAY) == pdPASS)
		{
			LastEvent = keyCode;
		}
	}
}

uint8_t Key_GetEvent(void)
{
	uint8_t ev;

	taskENTER_CRITICAL();
	ev = LastEvent;
	LastEvent = 0;
	taskEXIT_CRITICAL();

	return ev;
}
