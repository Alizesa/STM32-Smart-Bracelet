#ifndef __PN532_H_
#define __PN532_H_

#include <stdint.h>

void PN532_Init(void);

void PN532_Wakeup(void);

/**
  * @brief  Read the PN532 firmware version.
  * @retval 0 success, 1 failure
  */
uint8_t PN532_GetFirmwareVersion(uint8_t *ic, uint8_t *ver,
		uint8_t *rev, uint8_t *support);

/**
  * @brief  Configure the PN532 SAM (secure access module).
  * @retval 0 success, 1 failure
  */
uint8_t PN532_SAMConfig(void);

/**
  * @brief  Poll for a passive 14443A target (NFC card) and read its UID.
  * @param  uid        buffer (>= 8 bytes) to receive the card UID
  * @param  uidLen     UID length in bytes (4 for MIFARE Classic)
  * @param  timeout_ms wait time for a card
  * @retval 0 success, 1 no card / failure
  */
uint8_t PN532_ReadPassiveTargetID(uint8_t *uid, uint8_t *uidLen,
		uint32_t timeout_ms);

/**
  * @brief  Release the current target.
  * @retval 0 success, 1 failure
  */
uint8_t PN532_InRelease(void);

/**
  * @brief  Authenticate and read a MIFARE Classic block.
  * @param  block   block number (0-63)
  * @param  keyA    factory key A (6 bytes, default FFFFFFFFFFFF)
  * @param  dataOut 16-byte output buffer
  * @retval 0 success, 1 failure
  */
uint8_t PN532_ReadMifareBlock(uint8_t block, const uint8_t *keyA,
		uint8_t *dataOut);

/**
  * @brief  Authenticate and write a MIFARE Classic block.
  * @param  block   block number (0-63)
  * @param  keyA    factory key A (6 bytes)
  * @param  dataIn  16-byte data to write
  * @retval 0 success, 1 failure
  */
uint8_t PN532_WriteMifareBlock(uint8_t block, const uint8_t *keyA,
		const uint8_t *dataIn);

uint8_t PN532_HasUartRx(void);

#endif
