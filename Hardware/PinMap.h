#ifndef __PINMAP_H
#define __PINMAP_H

/*
 * ============================================================================
 *  STM32 SmartWatch - Pin allocation table (STM32F103C8T6, LQFP48)
 * ============================================================================
 *
 *  Peripheral            Interface        Pins
 *  --------------------- ---------------- -----------------------------------
 *  OLED 0.96"            I2C (bit-bang)   PB8 = SCL, PB9 = SDA      (unchanged)
 *  MPU6050 (accel/gyro)  I2C (bit-bang)   PB10 = SCL, PB11 = SDA    (unchanged)
 *  KEY1 / KEY2 / KEY3    GPIO pull-up     PB1 / PA6 / PA4           (unchanged)
 *  DHT11 (temp/humidity) 1-wire GPIO      PA1 = DATA               (NEW)
 *  MAX30102 (HR/SpO2)    I2C (bit-bang)   PB6 = SCL, PB7 = SDA     (moved from PA8/PA9)
 *  HC-05 Bluetooth       USART1           PA9 = TX, PA10 = RX      (NEW)
 *                        optional:        PA8 = STATE, PA0 = EN
 *  PN532 NFC             software I2C     PA2 = SCL, PA3 = SDA    (NEW)
 *                        optional:        PA5 = RST,  PA7 = IRQ
 *
 *  SWD debug             SWDIO/SWCLK      PA13 / PA14
 *
 *  NOTE: pins previously used by the removed LED module
 *  (PA0, PA2, PB12, PB13) are freed and reused as above.
 * ============================================================================
 */

/* -------- DHT11 (temperature & humidity) -------- */
#define DHT11_GPIO_PORT         GPIOA
#define DHT11_GPIO_PIN          GPIO_Pin_1
#define DHT11_RCC               RCC_APB2Periph_GPIOA

/* -------- MPU6050 INT (抬腕/跌倒硬件中断, 需飞线) -------- */
/* 默认预留 PB12(紧挨 MPU6050 的 PB10/PB11, 便于走线)。
   若实际接到其它引脚, 改这里即可(EXTI 需为 EXTI15_10 组)。 */
#define MPU6050_INT_PORT        GPIOB
#define MPU6050_INT_PORT_SOURCE GPIO_PortSourceGPIOB
#define MPU6050_INT_PIN         GPIO_Pin_12
#define MPU6050_INT_PIN_SOURCE  GPIO_PinSource12
#define MPU6050_INT_EXTI_LINE   EXTI_Line12
#define MPU6050_INT_GPIO_RCC    RCC_APB2Periph_GPIOB
#define MPU6050_INT_IRQn        EXTI15_10_IRQn

/* -------- MAX30102 (heart-rate / SpO2, bit-bang I2C) -------- */
#define MX30102_I2C_PORT        GPIOB
#define MX30102_I2C_SCL         GPIO_Pin_6
#define MX30102_I2C_SDA         GPIO_Pin_7
#define MX30102_I2C_RCC         RCC_APB2Periph_GPIOB

/* -------- HC-05 Bluetooth (USART1) -------- */
#define HC05_USART              USART1
#define HC05_USART_RCC          RCC_APB2Periph_USART1   /* USART1 is on APB2 */
#define HC05_USART_GPIO_RCC     RCC_APB2Periph_GPIOA
#define HC05_TX_PORT            GPIOA
#define HC05_TX_PIN             GPIO_Pin_9
#define HC05_RX_PORT            GPIOA
#define HC05_RX_PIN             GPIO_Pin_10

/* HC-05 optional pins (uncomment to use) */
//#define HC05_STATE_PORT         GPIOA
//#define HC05_STATE_PIN          GPIO_Pin_8
//#define HC05_EN_PORT            GPIOA
//#define HC05_EN_PIN             GPIO_Pin_0

/* -------- PN532 NFC (software I2C) -------- */
#define PN532_I2C_PORT          GPIOA
#define PN532_I2C_RCC           RCC_APB2Periph_GPIOA
#define PN532_I2C_SCL           GPIO_Pin_2
#define PN532_I2C_SDA           GPIO_Pin_3

/* PN532 optional pins (uncomment to use) */
#define PN532_RST_PORT          GPIOA
#define PN532_RST_PIN           GPIO_Pin_5
#define PN532_IRQ_PORT          GPIOA
#define PN532_IRQ_PIN           GPIO_Pin_7

#endif /* __PINMAP_H */
