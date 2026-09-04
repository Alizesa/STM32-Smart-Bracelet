/**
  * HC-05 Bluetooth module driver (USART1)
  *
  * Wiring : HC05 TXD -> PA10 (STM32 RX)
  *          HC05 RXD -> PA9  (STM32 TX)
  *          VCC 3.3V / 5V (module VCC 5V, logic 3.3V), GND
  *          STATE -> PA8, EN -> PA0 (optional, see PinMap.h)
  *
  * Default baud rate of the module is usually 9600 (AT) / 9600 or 38400
  * (data). Change HC05_BAUDRATE if your module differs.
  */
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "PinMap.h"
#include "HC05.h"

#define HC05_BAUDRATE           9600
#define HC05_RX_BUF_SIZE        256     /* must be power of two */

static volatile uint8_t  RxBuf[HC05_RX_BUF_SIZE];
static volatile uint16_t RxHead;
static volatile uint16_t RxTail;
static volatile uint32_t RxByteCount;
static volatile uint32_t RxErrorCount;
static SemaphoreHandle_t RxSem;

/**
  * @brief  Initialise USART1 for the HC-05 module and enable RX interrupt.
  */
void HC05_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RxHead = 0;
	RxTail = 0;
	RxByteCount = 0;
	RxErrorCount = 0;

	/* counting semaphore mirrors the number of bytes in the ring buffer */
	RxSem = xSemaphoreCreateCounting(HC05_RX_BUF_SIZE, 0);

	/* enable clocks */
	RCC_APB2PeriphClockCmd(HC05_USART_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(HC05_USART_RCC, ENABLE);

	/* TX pin: push-pull alternate function */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = HC05_TX_PIN;
	GPIO_Init(HC05_TX_PORT, &GPIO_InitStructure);

	/* RX pin: floating input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = HC05_RX_PIN;
	GPIO_Init(HC05_RX_PORT, &GPIO_InitStructure);

	/* USART1: 8N1, no flow control */
	USART_InitStructure.USART_BaudRate = HC05_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(HC05_USART, &USART_InitStructure);

	/* RX interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;   /* >= configMAX_SYSCALL_INTERRUPT_PRIORITY(5), else FreeRTOS assert */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(HC05_USART, USART_IT_RXNE, ENABLE);
	USART_Cmd(HC05_USART, ENABLE);
}

/**
  * @brief  Send a single byte (blocking, waits for TX empty).
  */
void HC05_SendByte(uint8_t data)
{
	while (USART_GetFlagStatus(HC05_USART, USART_FLAG_TXE) == RESET)
	{
	}
	USART_SendData(HC05_USART, data);
	while (USART_GetFlagStatus(HC05_USART, USART_FLAG_TC) == RESET)
	{
	}
}

/**
  * @brief  Send a NULL-terminated string.
  */
void HC05_SendString(char *str)
{
	while (*str)
	{
		HC05_SendByte((uint8_t)*str++);
	}
}

/**
  * @brief  Number of bytes waiting in the RX ring buffer.
  */
uint16_t HC05_RxAvailable(void)
{
	uint16_t count;

	taskENTER_CRITICAL();
	count = (uint16_t)((uint16_t)(RxHead - RxTail) & (HC05_RX_BUF_SIZE - 1));
	taskEXIT_CRITICAL();

	return count;
}

uint32_t HC05_GetRxByteCount(void)
{
	return RxByteCount;
}

uint32_t HC05_GetRxErrorCount(void)
{
	return RxErrorCount;
}

/**
  * @brief  Receive one byte, blocking up to timeout_ms.
  * @param  timeout_ms  wait time in ticks units accepted by FreeRTOS
  * @retval received byte, or 0 on timeout
  */
uint8_t HC05_ReceiveByte(uint32_t timeout_ms)
{
	uint8_t data;

	if (RxSem == NULL)
	{
		return 0;
	}

	if (xSemaphoreTake(RxSem, timeout_ms) != pdPASS)
	{
		return 0;
	}

	taskENTER_CRITICAL();
	data = RxBuf[RxTail];
	RxTail = (RxTail + 1) & (HC05_RX_BUF_SIZE - 1);
	taskEXIT_CRITICAL();

	return data;
}

/**
  * @brief  UART RX interrupt handler, called from USART1_IRQHandler().
  */
void HC05_USART_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint16_t next;
	uint16_t status = HC05_USART->SR;

	if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
	{
		RxErrorCount++;
	}

	if (USART_GetITStatus(HC05_USART, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(HC05_USART);
		RxByteCount++;

		next = (RxHead + 1) & (HC05_RX_BUF_SIZE - 1);
		if (next != RxTail)                 /* buffer not full */
		{
			RxBuf[RxHead] = data;
			RxHead = next;
			xSemaphoreGiveFromISR(RxSem, &xHigherPriorityTaskWoken);
		}
		/* full: drop the byte */
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if defined(HC05_EN_PORT)
void HC05_EnterATMode(void)
{
	/* pulse EN high (module in AT mode after power up at 38400 baud) */
	GPIO_SetBits(HC05_EN_PORT, HC05_EN_PIN);
}
void HC05_ExitATMode(void)
{
	GPIO_ResetBits(HC05_EN_PORT, HC05_EN_PIN);
}
#else
void HC05_EnterATMode(void)
{
}
void HC05_ExitATMode(void)
{
}
#endif
