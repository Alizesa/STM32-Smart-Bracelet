/**
  * PN532 NFC / RFID module driver (software I2C interface)
  *
  * Wiring : PN532 SCL -> PA2
  *          PN532 SDA -> PA3
  *          VCC 3.3V, GND
  *          RST -> PA5, IRQ -> PA7 (optional, see PinMap.h)
  *
  * The module must be switched to I2C mode (its DIP switches / solder
  * jumpers). PN532 I2C address is 0x24 (7-bit).
  *
  * PN532 frame (carried over I2C):
  *   host->PN532 : 00 00 FF LEN LCS D4 CMD DATA... DCS 00
  *   PN532->host : 00 00 FF LEN LCS D5 CMD+1 DATA... DCS 00
  *   ACK frame   : 00 00 FF 00 FF 00
  */
#include "stm32f10x.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Delay.h"
#include "PinMap.h"
#include "PN532.h"

#define PN532_I2C_ADDR          0x24
#define PN532_I2C_READ_LEN      40
#define PN532_MAX_CMD_LEN       32
#define PN532_MAX_RESP_LEN      64

static volatile uint8_t RxSeen;

/* ---------- raw RX capture of the last exchange (diagnostics) ---------- */
#define PN532_DBG_MAX           64
static volatile uint8_t  DbgBuf[PN532_DBG_MAX];
static volatile uint8_t  DbgLen;
static volatile uint8_t  DbgCap;     /* non-zero while waiting for a response */

/* ---------- low level helpers (bit-bang I2C, clock-stretch aware) ----------
 * Readiness is detected by polling the I2C read address: the PN532 NAKs it
 * while busy and ACKs it once a response is available. This matches the
 * reference implementation verified on the bench (no IRQ line needed). */

static volatile uint8_t I2C_BusError;

static void PN532_I2C_Delay(void) { Delay_us(1); }

/* Release SCL (open-drain) and wait until the line is actually high so a
 * slave that stretches the clock is respected. */
static void PN532_I2C_SCL_Release(void)
{
	uint32_t guard = 1000;              /* ~1 ms */
	GPIO_SetBits(PN532_I2C_PORT, PN532_I2C_SCL);
	while (GPIO_ReadInputDataBit(PN532_I2C_PORT, PN532_I2C_SCL) == Bit_RESET && guard--)
	{
		Delay_us(1);
	}
	if (GPIO_ReadInputDataBit(PN532_I2C_PORT, PN532_I2C_SCL) == Bit_RESET)
	{
		I2C_BusError = 1;               /* SCL stuck low */
	}
	PN532_I2C_Delay();
}

static void PN532_I2C_SCL_Low(void)  { GPIO_ResetBits(PN532_I2C_PORT, PN532_I2C_SCL); PN532_I2C_Delay(); }
static void PN532_I2C_SDA_High(void) { GPIO_SetBits(PN532_I2C_PORT, PN532_I2C_SDA); PN532_I2C_Delay(); }
static void PN532_I2C_SDA_Low(void)  { GPIO_ResetBits(PN532_I2C_PORT, PN532_I2C_SDA); PN532_I2C_Delay(); }
static uint8_t PN532_I2C_SDA_Read(void) { return GPIO_ReadInputDataBit(PN532_I2C_PORT, PN532_I2C_SDA); }

static void PN532_I2C_Start(void)
{
	PN532_I2C_SDA_High();
	PN532_I2C_SCL_Release();
	PN532_I2C_SDA_Low();
	PN532_I2C_SCL_Low();
}

static void PN532_I2C_Stop(void)
{
	PN532_I2C_SDA_Low();
	PN532_I2C_SCL_Release();
	PN532_I2C_SDA_High();
}

/* Send one byte, MSB first. Returns 0 = ACK, 1 = NACK, 2 = SCL stuck. */
static uint8_t PN532_I2C_SendByte(uint8_t value)
{
	uint8_t i, ack;
	I2C_BusError = 0;
	for (i = 0; i < 8; i++)
	{
		if (value & 0x80) PN532_I2C_SDA_High(); else PN532_I2C_SDA_Low();
		value <<= 1;
		PN532_I2C_SCL_Release();
		PN532_I2C_SCL_Low();
	}
	PN532_I2C_SDA_High();                       /* release SDA for slave ACK */
	PN532_I2C_SCL_Release();
	ack = PN532_I2C_SDA_Read();                 /* 0 = ACK, 1 = NACK */
	PN532_I2C_SCL_Low();
	PN532_I2C_SDA_High();
	if (I2C_BusError) return 2;
	return ack;
}

/* Read one byte. ack==1 -> master ACKs (keep reading); ack==0 -> NAK (last). */
static uint8_t PN532_I2C_ReadByte(uint8_t ack)
{
	uint8_t i, value = 0;
	PN532_I2C_SDA_High();                       /* release SDA for the slave */
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		PN532_I2C_SCL_Release();
		if (PN532_I2C_SDA_Read()) value |= 0x01;
		PN532_I2C_SCL_Low();
	}
	if (ack) PN532_I2C_SDA_Low(); else PN532_I2C_SDA_High();  /* ACK=low, NAK=high */
	PN532_I2C_SCL_Release();
	PN532_I2C_SCL_Low();
	PN532_I2C_SDA_High();
	return value;
}

/* Send one complete host command frame. Returns 0 on success (all ACKed).
 * The whole transaction runs with the scheduler suspended so a higher
 * priority task cannot preempt the bit-bang mid-byte and corrupt it. */
static uint8_t PN532_I2C_WriteFrame(const uint8_t *frame, uint8_t len)
{
	uint8_t i, r;
	vTaskSuspendAll();
	PN532_I2C_Start();
	r = PN532_I2C_SendByte((uint8_t)((PN532_I2C_ADDR << 1) | 0));   /* write addr */
	for (i = 0; r == 0 && i < len; i++)
	{
		Delay_us(30);                           /* inter-byte gap */
		r = PN532_I2C_SendByte(frame[i]);
	}
	PN532_I2C_Stop();
	xTaskResumeAll();
	return (r == 0) ? 0 : 1;
}

/* Try to pull one response chunk. The PN532 NAKs the read address while it
 * has nothing ready; once ACKed, read PN532_I2C_READ_LEN bytes in one
 * transaction (master ACKs all but the last byte). Returns 0 on success.
 * Scheduler suspended during the transaction for the same reason as above. */
static uint8_t PN532_I2C_ReadChunk(uint8_t *buffer)
{
	uint8_t i, r;
	vTaskSuspendAll();
	PN532_I2C_Start();
	r = PN532_I2C_SendByte((uint8_t)((PN532_I2C_ADDR << 1) | 1));  /* read addr */
	if (r != 0)                                 /* NACK = not ready yet */
	{
		PN532_I2C_Stop();
		xTaskResumeAll();
		return 1;
	}
	RxSeen = 1;
	for (i = 0; i < PN532_I2C_READ_LEN; i++)
	{
		buffer[i] = PN532_I2C_ReadByte((i == PN532_I2C_READ_LEN - 1) ? 0 : 1);
		if (I2C_BusError) break;
	}
	PN532_I2C_Stop();
	xTaskResumeAll();
	if (I2C_BusError) return 1;
	return 0;
}

/**
  * @brief  Send one complete command frame to the PN532.
  */
static uint8_t PN532_SendCommand(uint8_t cmd, const uint8_t *data, uint8_t len)
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

	return PN532_I2C_WriteFrame(frame, 9 + len);
}

/**
  * @brief  Scan a response chunk for a complete PN532 response frame
  *         (00 00 FF LEN LCS D5 ...), skipping any ACK frame before it.
  * @param  payload     receives [echo cmd, data...] (DCS/postamble excluded)
  * @retval 0 success, 1 frame not found / does not fit
  */
static uint8_t PN532_ExtractFrame(const uint8_t *buf, uint8_t n,
		uint8_t *payload, uint8_t maxLen, uint8_t *payloadLen)
{
	uint8_t i, j, len;

	for (i = 0; i + 5 < n; i++)
	{
		if (buf[i] != 0x00 || buf[i + 1] != 0x00 || buf[i + 2] != 0xFF) continue;
		len = buf[i + 3];
		if (len == 0) { i += 5; continue; }                 /* ACK frame */
		if ((uint8_t)(0x100 - len) != buf[i + 4]) continue; /* bad LCS */
		if (buf[i + 5] != 0xD5) continue;                   /* not a response */
		if (i + 6 + len > n) break;                         /* chunk too short */
		if (len - 1 > maxLen) return 1;
		*payloadLen = len - 1;
		for (j = 0; j < len - 1; j++) payload[j] = buf[i + 6 + j];
		return 0;
	}
	return 1;
}

/**
  * @brief  Read one response frame (polling the I2C read address, no IRQ).
  * @param  payload     output buffer, receives [echo cmd, data..., DCS excluded]
  * @param  maxLen      size of payload buffer
  * @param  payloadLen  number of bytes stored in payload
  * @param  timeout_ms  overall timeout
  * @retval 0 success, 1 timeout / protocol error
  */
static uint8_t PN532_ReadFrame(uint8_t *payload, uint8_t maxLen,
		uint8_t *payloadLen, uint32_t timeout_ms)
{
	uint8_t buf[PN532_I2C_READ_LEN];
	uint8_t i, dn;
	TickType_t start = xTaskGetTickCount();

	for (;;)
	{
		if (PN532_I2C_ReadChunk(buf) == 0)
		{
			if (PN532_ExtractFrame(buf, sizeof(buf), payload, maxLen, payloadLen) == 0)
			{
				if (DbgCap)
				{
					dn = (sizeof(buf) < PN532_DBG_MAX) ? sizeof(buf) : PN532_DBG_MAX;
					for (i = 0; i < dn; i++) DbgBuf[i] = buf[i];
					DbgLen = dn;
				}
				return 0;
			}
		}
		if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) return 1;
		vTaskDelay(pdMS_TO_TICKS(1));
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

	/* start a fresh raw-RX capture for this exchange */
	DbgLen = 0;
	DbgCap = 1;

	if (PN532_SendCommand(cmd, txData, txLen))
	{
		DbgCap = 0;
		return 1;
	}
	/* PN532 needs a short processing interval before the first read poll. */
	vTaskDelay(pdMS_TO_TICKS(1));

	if (PN532_ReadFrame(resp, sizeof(resp), &respLen, timeout_ms))
	{
		DbgCap = 0;
		return 1;
	}
	DbgCap = 0;
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

/* Kept as a no-op for compatibility with the shared interrupt file. */

void PN532_USART_IRQHandler(void)
{
}

/* ---------- init ---------- */

void PN532_Wakeup(void)
{
	/* I2C mode wakes on a bus transaction; a short idle period is sufficient. */
	Delay_ms(100);
}

void PN532_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RxSeen = 0;
	DbgLen = 0;
	DbgCap = 0;
	I2C_BusError = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = PN532_IRQ_PIN;
	GPIO_Init(PN532_IRQ_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = PN532_RST_PIN;
	GPIO_Init(PN532_RST_PORT, &GPIO_InitStructure);
	GPIO_SetBits(PN532_RST_PORT, PN532_RST_PIN);
	GPIO_ResetBits(PN532_RST_PORT, PN532_RST_PIN);
	Delay_ms(40);
	GPIO_SetBits(PN532_RST_PORT, PN532_RST_PIN);
	Delay_ms(100);

	PN532_Wakeup();
	/* Allow the oscillator and HSU parser to stabilize after power-up. */
	Delay_ms(400);
}

uint8_t PN532_HasUartRx(void)
{
	return RxSeen;
}

/**
  * @brief  Copy the raw bytes captured during the most recent command
  *         exchange (used to diagnose a BAD RX link: baud mismatch, NACK,
  *         swapped TX/RX or a non-standard PN532 frame).
  * @param  buf    output buffer
  * @param  maxLen size of buf
  * @retval number of bytes copied (0 = nothing was received)
  */
uint16_t PN532_DebugGetLastRx(uint8_t *buf, uint16_t maxLen)
{
	uint16_t n = ((uint16_t)DbgLen < maxLen) ? (uint16_t)DbgLen : maxLen;
	uint16_t i;

	taskENTER_CRITICAL();
	for (i = 0; i < n; i++)
	{
		buf[i] = DbgBuf[i];
	}
	taskEXIT_CRITICAL();
	return n;
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

	if (PN532_CommandExchange(0x14, cfg, 3, resp, &respLen, 2000))
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
