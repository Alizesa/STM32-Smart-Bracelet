/**
  * DHT11 temperature & humidity sensor driver (single-wire)
  *
  * Wiring : DHT11 DATA -> PA1 (see PinMap.h), pull-up 4.7k to 3.3V
  * Protocol: 1-wire. Host pulls line low >= 18ms, releases, DHT11 answers
  *           with 80us low + 80us high, then 40 data bits
  *           (humidity int, humidity dec, temp int, temp dec, checksum).
  *
  * NOTE: the 40-bit read section is timing critical, interrupts are
  *       disabled for ~4ms. Keep the calling task at a moderate priority.
  */
#include "stm32f10x.h"
#include "Delay.h"
#include "PinMap.h"
#include "DHT11.h"

/**
  * @brief  Initialise the DHT11 data pin (open-drain, released high).
  */
void DHT11_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(DHT11_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;   /* open-drain + external pull-up */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = DHT11_GPIO_PIN;
	GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);

	GPIO_SetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);     /* idle high */
}

/**
  * @brief  Wait until the data line equals the expected level.
  * @param  level       expected level (0/1)
  * @param  timeout_us  maximum wait time in microseconds
  * @retval 0 on success, 1 on timeout
  */
static uint8_t DHT11_WaitLevel(uint8_t level, uint32_t timeout_us)
{
	while (timeout_us--)
	{
		if (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == level)
		{
			return 0;
		}
		Delay_us(1);
	}
	return 1;
}

/**
  * @brief  Read one temperature / humidity sample.
  * @param  pHumidity     pointer to store humidity (0-100 %, integer)
  * @param  pTemperature  pointer to store temperature (0-50 C, integer)
  * @retval 0 on success, 1 on failure (no response / timeout / checksum)
  */
uint8_t DHT11_Read(uint8_t *pHumidity, uint8_t *pTemperature)
{
	uint8_t data[5] = {0, 0, 0, 0, 0};
	uint8_t i, j;
	uint8_t ret = 0;

	/* ---- host start signal: pull low > 18ms ---- */
	GPIO_ResetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
	Delay_us(20000);

	/* timing critical section: release + response + 40 bits (~5ms) */
	__disable_irq();
	GPIO_SetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
	Delay_us(40);

	/* DHT11 response: 80us low then 80us high */
	if (DHT11_WaitLevel(0, 100)) { ret = 1; goto out; }
	if (DHT11_WaitLevel(1, 100)) { ret = 1; goto out; }

	/* read 40 data bits */
	for (i = 0; i < 5; i++)
	{
		for (j = 0; j < 8; j++)
		{
			/* every bit starts with a ~50us low level */
			if (DHT11_WaitLevel(0, 100)) { ret = 1; goto out; }

			/* sample after 40us: 26-28us high => "0", ~70us high => "1" */
			Delay_us(40);
			if (GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
			{
				data[i] |= (0x80 >> j);
			}

			/* The next bit starts with low; do not wait for high here.
			 * For a logic-0 bit the high pulse has already ended at 40us. */
		}
	}

out:
	__enable_irq();

	if (ret)
	{
		return 1;
	}

	/* checksum: h_int + h_dec + t_int + t_dec (low byte) */
	if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
	{
		return 1;
	}

	*pHumidity    = data[0];   /* integer humidity % */
	*pTemperature = data[2];   /* integer temperature C */
	return 0;
}
