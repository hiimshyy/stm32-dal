/*
 * pn532.h
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

#ifndef INC_PN532_H_
#define INC_PN532_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// PN532 I2C Address
#define PN532_I2C_ADDRESS           0x24

// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION    0x02
#define PN532_COMMAND_SAMCONFIGURATION      0x14
#define PN532_COMMAND_RFCONFIGURATION       0x32
#define PN532_COMMAND_INLISTPASSIVETARGET   0x4A
#define PN532_COMMAND_INDATAEXCHANGE        0x40
#define PN532_COMMAND_INSELECT              0x54
#define PN532_COMMAND_INAUTOPOLL            0x60

// PN532 Card Types
#define PN532_MIFARE_ISO14443A              0x00
#define PN532_MIFARE_ISO14443B              0x01
#define PN532_FELICA_212                    0x02
#define PN532_FELICA_424                    0x03

// PN532 Frame Format
#define PN532_PREAMBLE                      0x00
#define PN532_STARTCODE1                    0x00
#define PN532_STARTCODE2                    0xFF
#define PN532_POSTAMBLE                     0x00

#define PN532_HOSTTOPN532                   0xD4
#define PN532_PN532TOHOST                   0xD5

// PN532 Status
typedef enum {
    PN532_STATUS_OK = 0,
    PN532_STATUS_ERROR,
    PN532_STATUS_TIMEOUT,
    PN532_STATUS_NO_CARD,
    PN532_STATUS_COMM_ERROR
} PN532_Status_t;

// PN532 Mode
typedef enum {
    PN532_MODE_NFC = 0,
    PN532_MODE_RFID
} PN532_Mode_t;

// Card Information Structure
typedef struct {
    uint8_t card_type;
    uint8_t uid_length;
    uint8_t uid[10];
    uint8_t sak;
    uint8_t atqa[2];
    bool card_present;
} PN532_CardInfo_t;

// PN532 Handle Structure
typedef struct {
    I2C_HandleTypeDef *hi2c;
    PN532_Mode_t mode;
    PN532_CardInfo_t card_info;
    uint32_t last_read_time;
    uint32_t read_timeout;
    PN532_Status_t status;
    uint8_t error_code;
} PN532_Handle_t;

// Function Prototypes
PN532_Status_t PN532_Init(PN532_Handle_t *hpn532, I2C_HandleTypeDef *hi2c);
PN532_Status_t PN532_SetMode(PN532_Handle_t *hpn532, PN532_Mode_t mode);
PN532_Status_t PN532_GetFirmwareVersion(PN532_Handle_t *hpn532, uint32_t *version);
PN532_Status_t PN532_ConfigureSAM(PN532_Handle_t *hpn532);
PN532_Status_t PN532_ReadCard(PN532_Handle_t *hpn532);
PN532_Status_t PN532_ReadPassiveTarget(PN532_Handle_t *hpn532);
bool PN532_IsCardPresent(PN532_Handle_t *hpn532);
uint8_t* PN532_GetCardUID(PN532_Handle_t *hpn532);
uint8_t PN532_GetCardType(PN532_Handle_t *hpn532);
PN532_Status_t PN532_GetStatus(PN532_Handle_t *hpn532);
uint8_t PN532_GetErrorCode(PN532_Handle_t *hpn532);

// Low level functions
PN532_Status_t PN532_WriteCommand(PN532_Handle_t *hpn532, uint8_t *cmd, uint8_t cmd_len);
PN532_Status_t PN532_ReadResponse(PN532_Handle_t *hpn532, uint8_t *response, uint8_t *response_len);
PN532_Status_t PN532_WaitReady(PN532_Handle_t *hpn532, uint32_t timeout);
uint8_t PN532_CalculateChecksum(uint8_t *data, uint8_t len);

#endif /* INC_PN532_H_ */
