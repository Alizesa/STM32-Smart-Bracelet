/**
  * PN532 NFC / RFID module driver (UART interface, USART2)
  *
  * Wiring : PN532 TX  -> PA3 (STM32 RX)
  *          PN532 RX  -> PA2 (STM32 TX)
  *          VCC 3.3V, GND
  *          RST -> PA5, IRQ -> PA7 (optional, see PinMap.h)
  *
  * The module must be switched to UART mode (its DIP switches / solder
  * jumpers). Default UART baud rate of the PN532 is 115200.
  *
  * UART frame:
  *   host->PN532 : 00 00 FF LEN LCS D4 CMD DATA... DCS 00
  *   PN532->host : 00 00 FF LEN LCS D5 CMD+1 DATA... DCS 00
  *   ACK frame   : 00 00 FF 00 FF 00
  */
#include "stm32f10x.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Delay.h"
#include "PinMap.h"
#include "PN532.h"

#define PN532_BAUDRATE          115200
#define PN532_RX_BUF_SIZE       256     /* power of two */
#define PN532_MAX_CMD_LEN       32
#define PN532_MAX_RESP_LEN      64

/* ---------- RX ring buffer + counting semaphore ---------- */
static volatile uint8_t  RxBuf[PN532_RX_BUF_SIZE];
static volatile uint16_t RxHead;
static volatile uint16_t RxTail;
static SemaphoreHandle_t RxSem;

/* ---------- low level helpers ---------- */

static void PN532_SendByte(uint8_t data)
{
	while (USART_GetFlagStatus(PN532_USART, USART_FLAG_TXE) == RESET)
	{
	}
	USART_SendData(PN532_USART, data);
	while (USART_GetFlagStatus(PN532_USART, USART_FLAG_TC) == RESET)
	{
	}
}

static uint8_t PN532_ReadByteTimeout(uint8_t *byte, uint32_t timeout_ms)
{
	if (RxSem == NULL)
	{
		return 1;
	}
	if (xSemaphoreTake(RxSem, timeout_ms) != pdPASS)
	{
		return 1;               /* timeout */
	}
	taskENTER_CRITICAL();
	*byte = RxBuf[RxTail];
	RxTail = (RxTail + 1) & (PN532_RX_BUF_SIZE - 1);
	taskEXIT_CRITICAL();
	return 0;
}

/**
  * @brief  Send one complete command frame to the PN532.
  */
static void PN532_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len)
{
	uint8_t frame[PN532_MAX_CMD_LEN + 9];
	uint8_t i, sum;
	uint8_t LEN = len + 2;                      /* TFI + cmd + data */

	frame[0] = 0x00;                            /* preamble */
	frame[1] = 0x00;                            /* start code */
	frame[2] = 0xFF;                            /* start code */
	frame[3] = LEN;
	frame[4] = (uint8_t)(0x100 - LEN);          /* LCS */
	frame[5] = 0xD4;                            /* TFI: host -> PN532 */
	frame[6] = cmd;
	sum = 0xD4 + cmd;
	for (i = 0; i < len; i++)
	{
		frame[7 + i] = data[i];
		sum += data[i];
	}
	frame[7 + len] = (uint8_t)(0x100 - sum);    /* DCS */
	frame[8 + len] = 0x00;                      /* postamble */

	for (i = 0; i < 9 + len; i++)
	{
		PN532_SendByte(frame[i]);
	}
}

/**
  * @brief  Read one response frame (skipping ACK frames).
  * @param  payload     output buffer, receives [echo cmd, data..., DCS excluded]
  * @param  maxLen      size of payload buffer
  * @param  payloadLen  number of bytes stored in payload
  * @param  timeout_ms  overall timeout
  * @retval 0 success, 1 timeout / protocol error
  */
static uint8_t PN532_ReadFrame(uint8_t *payload, uint8_t maxLen,
		uint8_t *payloadLen, uint32_t timeout_ms)
{
	uint8_t b, len, lcs, dcs, i, sum;
	uint8_t buf[PN532_MAX_RESP_LEN];

	for (;;)
	{
		/* synchronise to start code: 00 00 FF */
		if (PN532_ReadByteTimeout(&b, timeout_ms)) return 1;
		if (b != 0x00) continue;
		if (PN532_ReadByteTimeout(&b, timeout_ms)) return 1;
		if (b != 0x00) continue;
		if (PN532_ReadByteTimeout(&b, timeout_ms)) return 1;
		if (b != 0xFF) continue;

		/* LEN / LCS */
		if (PN532_ReadByteTimeout(&len, timeout_ms)) return 1;
		if (PN532_ReadByteTimeout(&lcs, timeout_ms)) return 1;

		if (len == 0)
		{
			/* ACK frame 00 00 FF 00 FF 00 : read postamble, keep waiting */
			if (lcs != 0xFF) continue;
			if (PN532_ReadByteTimeout(&b, timeout_ms)) return 1;
			continue;
		}

		if (lcs != (uint8_t)(0x100 - len)) continue;        /* bad checksum */
		if (len > PN532_MAX_RESP_LEN) return 1;

		sum = 0;
		for (i = 0; i < len; i++)
		{
			if (PN532_ReadByteTimeout(&b, timeout_ms)) return 1;
			buf[i] = b;
			sum += b;
		}

		/* DCS follows the LEN payload; postamble is the next byte. */
		if (PN532_ReadByteTimeout(&dcs, timeout_ms)) return 1;
		if ((uint8_t)(sum + dcs) != 0) continue;             /* bad DCS */
		if (PN532_ReadByteTimeout(&b, timeout_ms)) return 1;
		if (b != 0x00) continue;                            /* bad postamble */

		if (buf[0] != 0xD5) continue;                       /* not a response */

		/* buf contains TFI + response code + data; DCS was read separately. */
		if ((len - 1) > maxLen)
		{
			return 1;
		}
		for (i = 0; i < (len - 1); i++)
		{
			payload[i] = buf[1 + i];
		}
		*payloadLen = len - 1;
		return 0;
	}
}

/**
  * @brief  Send a command and wait for the matching response.
  * @retval 0 success, 1 failure
  */
static uint8_t PN532_CommandExchange(uint8_t cmd, const uint8_t *txData,
		uint8_t txLen, uint8_t *rxData, uint8_t *rxLen, uint32_t timeout_ms)
{
	uint8_t resp[PN532_MAX_RESP_LEN];
	uint8_t respLen = 0;

	PN532_SendCommand(cmd, txData, txLen);

	if (PN532_ReadFrame(resp, sizeof(resp), &respLen, timeout_ms))
	{
		return 1;
	}
	if (respLen < 1)
	{
		return 1;
	}
	if (resp[0] != (cmd + 1))           /* wrong echo */
	{
		return 1;
	}

	if (rxLen) *rxLen = respLen - 1;
	if (rxData && respLen > 1)
	{
		memcpy(rxData, &resp[1], respLen - 1);
	}
	return 0;
}

/* ---------- UART IRQ ---------- */

void PN532_USART_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint16_t next;

	if (USART_GetITStatus(PN532_USART, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(PN532_USART);

		next = (RxHead + 1) & (PN532_RX_BUF_SIZE - 1);
		if (next != RxTail)
		{
			RxBuf[RxHead] = data;
			RxHead = next;
			xSemaphoreGiveFromISR(RxSem, &xHigherPriorityTaskWoken);
		}
		/* full: drop */
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ---------- init ---------- */

void PN532_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RxHead = 0;
	RxTail = 0;

	RxSem = xSemaphoreCreateCounting(PN532_RX_BUF_SIZE, 0);

	RCC_APB2PeriphClockCmd(PN532_USART_GPIO_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(PN532_USART_RCC, ENABLE);

	/* TX: push-pull alternate function */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PN532_TX_PIN;
	GPIO_Init(PN532_TX_PORT, &GPIO_InitStructure);

	/* RX: floating input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = PN532_RX_PIN;
	GPIO_Init(PN532_RX_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = PN532_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(PN532_USART, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;   /* >= configMAX_SYSCALL_INTERRUPT_PRIORITY(5), else FreeRTOS assert */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(PN532_USART, USART_IT_RXNE, ENABLE);
	USART_Cmd(PN532_USART, ENABLE);

#if defined(PN532_RST_PORT)
	/* hardware reset pulse */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PN532_RST_PIN;
	GPIO_Init(PN532_RST_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(PN532_RST_PORT, PN532_RST_PIN);
	Delay_ms(10);
	GPIO_SetBits(PN532_RST_PORT, PN532_RST_PIN);
	Delay_ms(100);
#endif

	/* wake the module (required after power up / reset) */
	PN532_SendByte(0x55);
	PN532_SendByte(0x55);
	Delay_ms(50);
}

/* ---------- high level API ---------- */

uint8_t PN532_GetFirmwareVersion(uint8_t *ic, uint8_t *ver,
		uint8_t *rev, uint8_t *support)
{
	uint8_t resp[8];
	uint8_t respLen = 0;

	if (PN532_CommandExchange(0x02, NULL, 0, resp, &respLen, 500))
	{
		return 1;
	}
	if (respLen != 4)               /* [IC][Ver][Rev][Support] */
	{
		return 1;
	}

	*ic      = resp[0];
	*ver     = resp[1];
	*rev     = resp[2];
	*support = resp[3];
	return 0;
}

uint8_t PN532_SAMConfig(void)
{
	uint8_t cfg[3] = {0x01, 0x14, 0x01};    /* normal mode, ~1s timeout */
	uint8_t resp[4];
	uint8_t respLen = 0;

	if (PN532_CommandExchange(0x14, cfg, 3, resp, &respLen, 500))
	{
		return 1;
	}
	if (respLen != 1 || resp[0] != 0x00)    /* status */
	{
		return 1;
	}
	return 0;
}

uint8_t PN532_ReadPassiveTargetID(uint8_t *uid, uint8_t *uidLen,
		uint32_t timeout_ms)
{
	uint8_t tx[2] = {0x01, 0x00};           /* MaxTg=1, 106kbps type A */
	uint8_t resp[64];
	uint8_t respLen = 0;
	uint8_t i;

	if (PN532_CommandExchange(0x4A, tx, 2, resp, &respLen, timeout_ms))
	{
		return 1;
	}
	/* resp = [nbTg][Tg][SENS0][SENS1][SEL][UIDlen][UID...] */
	if (respLen < 6)
	{
		return 1;
	}
	if (resp[0] == 0)                       /* no card detected */
	{
		return 1;
	}

	/* UID length is byte 5; bytes 6 onward contain the UID. */
	if (resp[5] == 0 || resp[5] > 8 || respLen < (uint8_t)(6 + resp[5]))
	{
		return 1;
	}
	if (uidLen) *uidLen = resp[5];
	if (uid)
	{
		for (i = 0; i < resp[5]; i++)
		{
			uid[i] = resp[6 + i];
		}
	}
	return 0;
}

uint8_t PN532_InRelease(void)
{
	uint8_t tx[1] = {0x00};
	uint8_t resp[4];
	uint8_t respLen = 0;

	if (PN532_CommandExchange(0x52, tx, 1, resp, &respLen, 500))
	{
		return 1;
	}
	if (respLen != 1 || resp[0] != 0x00)    /* status */
	{
		return 1;
	}
	return 0;
}

/* ---------- MIFARE Classic ---------- */

/**
  * @brief  Authenticate a MIFARE Classic block with Key A.
  */
static uint8_t PN532_MifareAuth(uint8_t block, const uint8_t *keyA)
{
	uint8_t cmd[8];
	uint8_t resp[8];
	uint8_t respLen = 0;
	uint8_t i;

	cmd[0] = 0x60;                          /* MIFARE_CMD_AUTH_A */
	cmd[1] = block;
	for (i = 0; i < 6; i++)
	{
		cmd[2 + i] = keyA[i];
	}

	if (PN532_CommandExchange(0x40, cmd, 8, resp, &respLen, 500))
	{
		return 1;
	}
	/* InDataExchange response: [status][data...] */
	if (respLen < 1 || resp[0] != 0x00)
	{
		return 1;
	}
	return 0;
}

uint8_t PN532_ReadMifareBlock(uint8_t block, const uint8_t *keyA,
		uint8_t *dataOut)
{
	uint8_t cmd[2] = {0x30, block};         /* MIFARE_CMD_READ */
	uint8_t resp[32];
	uint8_t respLen = 0;

	if (PN532_MifareAuth(block, keyA))
	{
		return 1;
	}

	if (PN532_CommandExchange(0x40, cmd, 2, resp, &respLen, 500))
	{
		return 1;
	}
	if (respLen != 17 || resp[0] != 0x00)   /* [status][16 data] */
	{
		return 1;
	}

	memcpy(dataOut, &resp[1], 16);
	return 0;
}

uint8_t PN532_WriteMifareBlock(uint8_t block, const uint8_t *keyA,
		const uint8_t *dataIn)
{
	uint8_t cmd[18];
	uint8_t resp[8];
	uint8_t respLen = 0;

	if (PN532_MifareAuth(block, keyA))
	{
		return 1;
	}

	cmd[0] = 0xA0;                          /* MIFARE_CMD_WRITE */
	cmd[1] = block;
	memcpy(&cmd[2], dataIn, 16);

	if (PN532_CommandExchange(0x40, cmd, 18, resp, &respLen, 500))
	{
		return 1;
	}
	if (respLen < 1 || resp[0] != 0x00)
	{
		return 1;
	}
	return 0;
}
