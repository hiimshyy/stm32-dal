/*
 * pn532.c
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

#include "pn532.h"
#include <string.h>

#define PN532_TIMEOUT (100) ///< Default timeout for PN532 communication
//#define PN532DEBUG    1
//#define MIFAREDEBUG 	1

uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}; ///< ACK message from PN532
uint8_t pn532response_firmwarevers[] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5}; ///< Expected firmware version message from PN532

#define PN532_BUFFER_SIZE (64)                     ///< PN532 buffer size
uint8_t pn532_packetbuffer[PN532_BUFFER_SIZE]; ///< Buffer to hold data sent to and received from PN532

int8_t _uid[7];      // ISO14443A uid
int8_t _uidLen;      // uid len
int8_t _key[6];      // Mifare Classic key
int8_t _inListedTag; // Tg number of inlisted tag.

static PN532_Config* _config = NULL;

PN532_Status_t PN532_Init(PN532_Config* config)
{
    _config = config;
    if (_config == NULL) {
        return PN532_STATUS_ERROR;
    }
#ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - Initializing...\r\n");
    }
#endif

    // Reset nếu có chân reset
    reset();
    HAL_Delay(100);

    // Wakeup
    wakeup();

    // // Verify firmware version
    // uint32_t versiondata = getFirmwareVersion();
    // if (!versiondata) {
    //     if (_config->log) {
    //         _config->log("[PN532] - Failed to get firmware version\r\n");
    //     }
    //     return PN532_STATUS_ERROR;
    // }

    // if (_config->log) {
    //     _config->log("[PN532] - Found chip PN5");
    //     _config->log("%02X\r\n", (versiondata >> 24) & 0xFF);
    //     _config->log("[PN532] - Firmware ver. %d.", (versiondata >> 16) & 0xFF);
    //     _config->log("%d\r\n", (versiondata >> 8) & 0xFF);
    // }

    return PN532_STATUS_OK;
}

void reset(void)
{
    if (_config->reset_port != NULL) {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - Hardware reset\r\n");
        }
        #endif

        HAL_GPIO_WritePin(_config->reset_port, _config->reset_pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(_config->reset_port, _config->reset_pin, GPIO_PIN_SET);
        HAL_Delay(50);  // Tăng delay sau reset
    } else {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - No reset pin configured\r\n");
        }
        #endif
    }
}

void wakeup(void)
{
	uint8_t wakeup_cmd = 0x55;
	HAL_I2C_Master_Transmit(_config->hi2c, _config->i2c_addr, &wakeup_cmd, 1, 100);
	HAL_Delay(10);
	SAMConfig();
}

static void readdata(uint8_t* buff, uint8_t n)
{
    uint8_t rbuff[n + 1]; // +1 for leading RDY byte
    HAL_I2C_Master_Receive(_config->hi2c, _config->i2c_addr, rbuff, n + 1, 100);
    for (uint8_t i = 0; i < n; i++) {
        buff[i] = rbuff[i + 1];
    }

    #ifdef PN532DEBUG
    if (_config->log) {
		_config->log("[PN532] - Read: ");
		for (uint8_t i = 0; i < n; i++) {
			_config->log(" %02X", buff[i]);
		}
		_config->log("\n");
	}
    #endif
}

static void writecommand(uint8_t *cmd, uint8_t cmdlen)
{
    uint8_t packet[8 + cmdlen];
    uint8_t LEN = cmdlen + 1;

    packet[0] = PN532_PREAMBLE;
    packet[1] = PN532_STARTCODE1;
    packet[2] = PN532_STARTCODE2;
    packet[3] = LEN;
    packet[4] = ~LEN + 1;
    packet[5] = PN532_HOSTTOPN532;

    uint8_t sum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmdlen; i++) {
        packet[6 + i] = cmd[i];
        sum += cmd[i];
    }

    packet[6 + cmdlen] = ~sum + 1;
    packet[7 + cmdlen] = PN532_POSTAMBLE;

    #ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - Write: ");
        for (uint8_t i = 0; i < 8 + cmdlen; i++) {
            _config->log("%02X ", packet[i]);
        }
        _config->log("\r\n");
    }
    #endif

    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(_config->hi2c, _config->i2c_addr, packet, 8 + cmdlen, 100);

    if (result != HAL_OK) {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - Write failed: %d\r\n", result);
        }
        #endif
    }
}

static bool isready(void)
{
    uint8_t status;
    HAL_StatusTypeDef result = HAL_I2C_Master_Receive(_config->hi2c, _config->i2c_addr, &status, 1, 100);

    if (result != HAL_OK) {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - I2C read failed: %d\r\n", result);
        }
        #endif
        return false;
    }

    #ifdef PN532DEBUG
    if (_config->log && status == PN532_I2C_READY) {
        _config->log("[PN532] - Ready! Status: 0x%02X\r\n", status);
    }
    #endif

    return (status == PN532_I2C_READY);
}

static bool waitready(uint16_t timeout)
{
    uint32_t timer = 0;
    uint32_t max_wait = (timeout == 0) ? 1000 : timeout;

    #ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - Waiting for ready (timeout=%lu ms)...\r\n", max_wait);
    }
    #endif

    while(!isready()) {
        HAL_Delay(10);
        timer += 10;
        if (timer >= max_wait) {
            #ifdef PN532DEBUG
            if (_config->log) {
                _config->log("[PN532] - Timeout after %lu ms!\r\n", timer);
            }
            #endif
            return false;
        }
    }

    #ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - Ready after %lu ms\r\n", timer);
    }
    #endif

    return true;
}

static bool readack(void)
{
	uint8_t ackbuff[6];
	readdata(ackbuff, 6);
	return (0 == memcmp((char *)ackbuff, (char *)pn532ack, 6));
}

static void PrintHex(const uint8_t *data, const uint32_t numBytes)
{
	for (uint32_t i = 0; i < numBytes; i++) {
		if (i != 0) {
			if ((i % 16) == 0) {
				if (_config->log) {
					_config->log("\n");
				}
			} else if ((i % 8) == 0) {
				if (_config->log) {
					_config->log("  ");
				}
			} else {
				if (_config->log) {
					_config->log(" ");
				}
			}
		}
		if (_config->log) {
			_config->log("%02X", data[i]);
		}
	}
	if (_config->log) {
		_config->log("\n");
	}
}

static void PrintHexChar(const uint8_t *pbtData, const uint32_t numBytes)
{
	uint8_t numCols = 16;
	for (uint32_t i = 0; i < numBytes; i++) {
		if (i != 0) {
			if ((i % numCols) == 0) {
				if (_config->log) {
					_config->log("\n");
				}
			} else if ((i % 8) == 0) {
				if (_config->log) {
					_config->log("  ");
				}
			} else {
				if (_config->log) {
					_config->log(" ");
				}
			}
		}
		if (_config->log) {
			_config->log("%02X", pbtData[i]);
		}
	}
	if (_config->log) {
		_config->log("\n");
	}

	for (uint32_t i = 0; i < numBytes; i++) {
		if (i != 0) {
			if ((i % numCols) == 0) {
				if (_config->log) {
					_config->log("\n");
				}
			} else if ((i % 8) == 0) {
				if (_config->log) {
					_config->log("  ");
				}
			} else {
				if (_config->log) {
					_config->log(" ");
				}
			}
		}
		if (pbtData[i] < 0x20 || pbtData[i] > 0x7E) {
			if (_config->log) {
				_config->log(".");
			}
		} else {
			if (_config->log) {
				_config->log("%c", pbtData[i]);
			}
		}
	}
	if (_config->log) {
		_config->log("\n");
	}
}

bool SAMConfig(void)
{
    #ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - Configuring SAM...\r\n");
    }
    #endif

    pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    pn532_packetbuffer[1] = 0x01; // normal mode
    pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
    pn532_packetbuffer[3] = 0x01; // use IRQ pin

    if (!sendCommandCheckAck(pn532_packetbuffer, 4, PN532_TIMEOUT)) {
        return false;
    }

    readdata(pn532_packetbuffer, 9);

    int offset = 6;
    bool success = (pn532_packetbuffer[offset] == 0x15);

    #ifdef PN532DEBUG
    if (_config->log) {
        if (success) {
            _config->log("[PN532] - SAM configured successfully\r\n");
        } else {
            _config->log("[PN532] - SAM config failed\r\n");
        }
    }
    #endif

    return success;
}

uint32_t getFirmwareVersion(void)
{
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

    if (!sendCommandCheckAck(pn532_packetbuffer, 1, PN532_TIMEOUT)) {
        return 0;
    }

    readdata(pn532_packetbuffer, 13);

    int offset = 6;
    response = pn532_packetbuffer[offset++];
    response <<= 8;
    response |= pn532_packetbuffer[offset++];
    response <<= 8;
    response |= pn532_packetbuffer[offset++];
    response <<= 8;
    response |= pn532_packetbuffer[offset++];

    return response;
}

bool sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout)
{
    writecommand(cmd, cmdlen);

    HAL_Delay(2);  // ✓ Thêm delay để PN532 xử lý

    if (!waitready(timeout)) {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - Timeout waiting for ready\r\n");
        }
        #endif
        return false;
    }

    if(!readack()){
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - No ACK received\r\n");
        }
        #endif
        return false;
    }

    HAL_Delay(2);  // ✓ Thêm delay trước khi đọc response

    if (!waitready(timeout)) {
        return false;
    }

    return true;
}

bool setPassiveActivationRetries(uint8_t maxRetries)
{
    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 0x05;
    pn532_packetbuffer[2] = 0xFF;
    pn532_packetbuffer[3] = 0x01;
    pn532_packetbuffer[4] = maxRetries;

    #ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - SetPassiveActivationRetries: %d\r\n", maxRetries);
    }
    #endif

    if (!sendCommandCheckAck(pn532_packetbuffer, 5, PN532_TIMEOUT)) {
        return false;
    }

    readdata(pn532_packetbuffer, 9);
    return true;
}

bool readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid, uint8_t *uidLength, uint16_t timeout)
{
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 0x01;
    pn532_packetbuffer[2] = cardbaudrate;

    #ifdef PN532DEBUG
    if (_config->log) {
        _config->log("[PN532] - Scanning for cards (timeout=%d ms)...\r\n", timeout);
    }
    #endif

    if (!sendCommandCheckAck(pn532_packetbuffer, 3, timeout > 1000 ? 1000 : timeout)) {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - Command failed\r\n");
        }
        #endif
        return false;
    }

    // PN532 cần thời gian để quét thẻ - đợi lâu hơn
    HAL_Delay(50);

    // Đợi response với timeout dài
    if (!waitready(timeout)) {
        #ifdef PN532DEBUG
        if (_config->log) {
            _config->log("[PN532] - No card detected\r\n");
        }
        #endif
        return false;
    }

    return readDetectedPassiveTargetID(uid, uidLength);
}

bool startPassiveTargetIDDetection(uint8_t cardbaudrate)
{
	pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	pn532_packetbuffer[1] = 0x01; // max 1 card at once
	pn532_packetbuffer[2] = cardbaudrate;

	return sendCommandCheckAck(pn532_packetbuffer, 3, PN532_TIMEOUT);

}

bool readDetectedPassiveTargetID(uint8_t *uid, uint8_t *uidLength)
{
    readdata(pn532_packetbuffer, 20);

    #ifdef MIFAREDEBUG
    if (_config->log) {
        _config->log("[PN532] - Found %d tag(s)\r\n", pn532_packetbuffer[7]);
    }
    #endif

    if (pn532_packetbuffer[7] != 1) {
        return false;
    }

    uint16_t sens_res = pn532_packetbuffer[9];
    sens_res <<= 8;
    sens_res |= pn532_packetbuffer[10];

    #ifdef MIFAREDEBUG
    if (_config->log) {
        _config->log("[PN532] - ATQA: 0x%04X\r\n", sens_res);
        _config->log("[PN532] - SAK: 0x%02X\r\n", pn532_packetbuffer[11]);
    }
    #endif

    *uidLength = pn532_packetbuffer[12];

    #ifdef MIFAREDEBUG
    if (_config->log) {
        _config->log("[PN532] - UID:");
    }
    #endif

    for (uint8_t i = 0; i < pn532_packetbuffer[12]; i++) {
        uid[i] = pn532_packetbuffer[13 + i];
        #ifdef MIFAREDEBUG
        if (_config->log) {
            _config->log(" %02X", uid[i]);
        }
        #endif
    }

    #ifdef MIFAREDEBUG
    if (_config->log) {
        _config->log("\r\n");
    }
    #endif

    return true;
}

bool inDataExchange(uint8_t *send, uint8_t sendLength, uint8_t *response, uint8_t *responseLength)
{
	if (sendLength > PN532_BUFFER_SIZE - 2) {
		return false;
	}
	#ifdef PN532DEBUG
		if (_config->log) {
			_config->log("[PN532] - APDU length too long for packet buffer.\n");
		}
	#endif
	uint8_t i;

	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = _inListedTag; // Card number

	memcpy(pn532_packetbuffer + 2, send, sendLength);

	if (!sendCommandCheckAck(pn532_packetbuffer, sendLength + 2, PN532_TIMEOUT*10)) {
		#ifdef PN532DEBUG
			if (_config->log) {
				_config->log("[PN532] - Could not send APDU\n");
			}
		#endif
		return false;
	}

	if (!waitready(PN532_TIMEOUT*10)) {
		#ifdef PN532DEBUG
			if (_config->log) {
				_config->log("[PN532] - Response never received for APDU\n");
			}
		#endif
		return false;
	}

	readdata(pn532_packetbuffer, PN532_BUFFER_SIZE);

	if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff){
		uint8_t length = pn532_packetbuffer[3];
		if (pn532_packetbuffer[4] != (uint8_t)(~length + 1)){
			#ifdef PN532DEBUG
				if (_config->log) {
					_config->log("[PN532] - Length check invalid for APDU\n");
					_config->log(" %02X != %02X\n", length, (uint8_t)(~length + 1));
				}
			#endif
			return false;
		}

		if (pn532_packetbuffer[5] == PN532_PN532TOHOST && pn532_packetbuffer[6] == PN532_RESPONSE_INDATAEXCHANGE){
			if (pn532_packetbuffer[7] != 0x00){
				#ifdef PN532DEBUG
					if (_config->log) {
						_config->log("[PN532] - Status code indicates an error: %02X\n", pn532_packetbuffer[7]);
					}
				#endif
				return false;
			}

			length -= 3;
			if (length > *responseLength){
				length = *responseLength; // silent truncation...
			}

			for (i = 0; i < length; i++){
				response[i] = pn532_packetbuffer[8 + i];
			}

			*responseLength = length;

			return true;
		} else {
			#ifdef PN532DEBUG
				if (_config->log) {
					_config->log("[PN532] - Don't know how to handle this command: %02X\n", pn532_packetbuffer[6]);
				}
			#endif
			return false;
		}
	} else {
		#ifdef PN532DEBUG
			if (_config->log) {
				_config->log("[PN532] - Preamble missing.\n");
			}
		#endif
		return false;
	}
}

bool inListPassiveTarget()
{
	pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	pn532_packetbuffer[1] = 1;
	pn532_packetbuffer[2] = 0;

	#ifdef PN532DEBUG
		if (_config->log) {
			_config->log("[PN532] - About to inList passive target\n");

		}
	#endif

	if (!sendCommandCheckAck(pn532_packetbuffer, 3, PN532_TIMEOUT*10)) {
		#ifdef PN532DEBUG
			if (_config->log) {
				_config->log("[PN532] - Could not send inlist message\n");
			}
		#endif
		return false;
	}

	if (!waitready(PN532_TIMEOUT*300)) {
		return false;
	}

	readdata(pn532_packetbuffer, PN532_BUFFER_SIZE);
	if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
	    uint8_t length = pn532_packetbuffer[3];
	    if (pn532_packetbuffer[4] != (uint8_t)(~length + 1)) {
	    	#ifdef PN532DEBUG
	            if (_config->log) {
	                _config->log("[PN532] - Length check invalid for inListPassiveTarget\n");
	                _config->log(" %02X != %02X\n", length, (uint8_t)(~length + 1));
	            }
	        #endif
	        return false;
	    }

	    if (pn532_packetbuffer[5] == PN532_PN532TOHOST && pn532_packetbuffer[6] == PN532_RESPONSE_INLISTPASSIVETARGET) {
	    	if (pn532_packetbuffer[7] != 1) {
	    		#ifdef PN532DEBUG
	    			_config->log("[PN532] - Unhandled number of targets inlisted\n");
	    		#endif
	        	_config->log("[PN532] - Number of tags inlisted: %d\n", pn532_packetbuffer[7]);
	            return false;
	          }

	          _inListedTag = pn532_packetbuffer[8];
	          _config->log("[PN532] - Tag number: %d\n", _inListedTag);

	          return true;
	        } else {
	    #ifdef PN532DEBUG
	        	_config->log("[PN532] - Unexpected response to inlist passive host\n");
	    #endif
	          return false;
	        }
	      } else {
	    #ifdef PN532DEBUG
	    	  _config->log("[PN532] - Preamble missing\n");
	    #endif
	        return false;
	}
	return true;
}

bool mifareclassic_IsFirstBlock(uint32_t uiBlock)
{
	if (uiBlock < 128)
		return ((uiBlock) % 4 == 0);
	else
		return ((uiBlock) % 16 == 0);
}

bool mifareclassic_IsTrailerBlock(uint32_t uiBlock)
{
	if (uiBlock < 128)
		return ((uiBlock + 1) % 4 == 0);
	else
		return ((uiBlock + 1) % 16 == 0);
}

uint8_t mifareclassic_AuthenticateBlock(uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData)
{
	// Hang on to the key and uid data
	memcpy(_key, keyData, 6);
	memcpy(_uid, uid, uidLen);
	_uidLen = uidLen;

	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Trying to authenticate card with UID: ");
		PrintHex((const uint8_t*)_uid, _uidLen);
		_config->log("[PN532] - Using authentication KEY ");
		_config->log(keyNumber ? "B" : "A");
		_config->log(": ");
		PrintHex((const uint8_t*)_key, 6);
	#endif

	// Prepare the authentication command //
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE; /* Data Exchange Header */
	pn532_packetbuffer[1] = 1; /* Max card numbers */
	pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
	pn532_packetbuffer[3] = blockNumber; /* Block Number (1K = 0..63, 4K = 0..255 */
	memcpy(pn532_packetbuffer + 4, _key, 6);
	for (uint8_t i = 0; i < _uidLen; i++) {
	  pn532_packetbuffer[10 + i] = _uid[i]; /* 4 byte card ID */
	}

	if (!sendCommandCheckAck(pn532_packetbuffer, 10 + _uidLen, PN532_TIMEOUT))
	   return 0;

	// Read the response packet
	readdata(pn532_packetbuffer, 12);

	// check if the response is valid and we are authenticated???
	// for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
	// Mifare auth error is technically byte 7: 0x14 but anything other and 0x00
	// is not good
	if (pn532_packetbuffer[7] != 0x00) {
		#ifdef PN532DEBUG
			_config->log("[PN532] - Authentification failed: ");
			PrintHexChar(pn532_packetbuffer, 12);
		#endif
	    return 0;
	}

	return 1;
}

uint8_t mifareclassic_ReadDataBlock(uint8_t blockNumber, uint8_t *data)
{
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Trying to read 16 bytes from block: %d\n", blockNumber);
	#endif

	/* Prepare the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;               /* Card number */
	pn532_packetbuffer[2] = MIFARE_CMD_READ; /* Mifare Read command = 0x30 */
	pn532_packetbuffer[3] = blockNumber; /* Block Number (0..63 for 1K, 0..255 for 4K) */

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 4, PN532_TIMEOUT)) {
		#ifdef MIFAREDEBUG
		_config->log("[PN532] - Failed to receive ACK for read command\n");
		#endif
		return 0;
	}

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);

	/* If byte 8 isn't 0x00 we probably have an error */
	if (pn532_packetbuffer[7] != 0x00) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Unexpected response:");
			PrintHexChar(pn532_packetbuffer, 26);
		#endif
		return 0;
	}

	/* Copy the 16 data bytes to the output buffer        */
	/* Block content starts at byte 9 of a valid response */
	memcpy(data, pn532_packetbuffer + 8, 16);

	/* Display data for debug if requested */
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Block: %d\n", blockNumber);
		PrintHexChar(data, 16);
	#endif

  return 1;
}

uint8_t mifareclassic_WriteDataBlock(uint8_t blockNumber, uint8_t *data)
{
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Trying to write 16 bytes to block %d\n", blockNumber);
	#endif

	/* Prepare the first part of the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;                 /* Card number */
	pn532_packetbuffer[2] = MIFARE_CMD_WRITE;  /* Mifare Write command = 0xA0 */
	pn532_packetbuffer[3] = blockNumber;   /* Block Number (0..63 for 1K, 0..255 for 4K) */

	/* Now add the 16 bytes of data to be written */
	memcpy(pn532_packetbuffer + 4, data, 16);

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 20, PN532_TIMEOUT)) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Failed to receive ACK for write command\n");
		#endif
		return 0;
	}

	HAL_Delay(10);

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);

	return 1;
}

uint8_t mifareclassic_FormatNDEF(void)
{
	uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1,
	                             0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
	uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1,
	                             0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
	uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77,
	                             0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	// Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
	// for the MAD sector in NDEF records (sector 0)

	// Write block 1 and 2 to the card
	if (!(mifareclassic_WriteDataBlock(1, sectorbuffer1)))
		return 0;
	if (!(mifareclassic_WriteDataBlock(2, sectorbuffer2)))
		return 0;
	// Write key A and access rights card
	if (!(mifareclassic_WriteDataBlock(3, sectorbuffer3)))
		return 0;

	// Seems that everything was OK (?!)
	return 1;
}

uint8_t mifareclassic_WriteNDEFURI(uint8_t sectorNumber, uint8_t uriIdentifier, const char *url)
{
	// Figure out how long the string is
	uint8_t len = strlen(url);

	// Make sure we're within a 1K limit for the sector number
	if ((sectorNumber < 1) || (sectorNumber > 15))
	    return 0;

	// Make sure the URI payload is between 1 and 38 chars
	if ((len < 1) || (len > 38))
	    return 0;

	// Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
	// in NDEF records

	// Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
	uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, (uint8_t)(len + 5), 0xD1, 0x01, (uint8_t)(len + 1), 0x55,
								uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07,
	                             0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	if (len <= 6) {
	   // Unlikely we'll get a url this short, but why not ...
	    memcpy(sectorbuffer1 + 9, url, len);
	    sectorbuffer1[len + 9] = 0xFE;
	} else if (len == 7) {
	    // 0xFE needs to be wrapped around to next block
	    memcpy(sectorbuffer1 + 9, url, len);
	    sectorbuffer2[0] = 0xFE;
	} else if ((len > 7) && (len <= 22)) {
	    // Url fits in two blocks
	    memcpy(sectorbuffer1 + 9, url, 7);
	    memcpy(sectorbuffer2, url + 7, len - 7);
	    sectorbuffer2[len - 7] = 0xFE;
	} else if (len == 23) {
	    // 0xFE needs to be wrapped around to final block
	    memcpy(sectorbuffer1 + 9, url, 7);
	    memcpy(sectorbuffer2, url + 7, len - 7);
	    sectorbuffer3[0] = 0xFE;
	} else {
	    // Url fits in three blocks
	    memcpy(sectorbuffer1 + 9, url, 7);
	    memcpy(sectorbuffer2, url + 7, 16);
	    memcpy(sectorbuffer3, url + 23, len - 24);
	    sectorbuffer3[len - 22] = 0xFE;
	}

	  // Now write all three blocks back to the card
	if (!(mifareclassic_WriteDataBlock(sectorNumber * 4, sectorbuffer1)))
	    return 0;
	if (!(mifareclassic_WriteDataBlock((sectorNumber * 4) + 1, sectorbuffer2)))
	    return 0;
	if (!(mifareclassic_WriteDataBlock((sectorNumber * 4) + 2, sectorbuffer3)))
	    return 0;
	if (!(mifareclassic_WriteDataBlock((sectorNumber * 4) + 3, sectorbuffer4)))
	    return 0;

	// Seems that everything was OK (?!)
	return 1;
}

uint8_t mifareultralight_ReadPage(uint8_t page, uint8_t *buffer)
{
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Trying to read 4 bytes from page %d\n", page);
	#endif

	/* Prepare the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;                   /* Card number */
	pn532_packetbuffer[2] = MIFARE_CMD_READ; /* Mifare Ultralight Read command = 0x30 */
	pn532_packetbuffer[3] = page; /* Page Number (0..63 for 1K, 0..255 for 4K) */

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 4, PN532_TIMEOUT)) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Failed to receive ACK for read command\n");
		#endif
		return 0;
	}

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);

	/* If byte 8 isn't 0x00 we probably have an error */
	if (pn532_packetbuffer[7] != 0x00) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Unexpected response");
			PrintHexChar(pn532_packetbuffer, 26);
		#endif
		return 0;
	}

	/* Copy the 4 data bytes to the output buffer        */
	/* Page content starts at byte 9 of a valid response */
	memcpy(buffer, pn532_packetbuffer + 8, 4);

	/* Display data for debug if requested */
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Page: %d\n", page);
		PrintHexChar(buffer, 4);
	#endif

	return 1;
}

uint8_t mifareultralight_WritePage(uint8_t page, uint8_t *data)
{
	if (page >= 64) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Page value %d out of range\n", page);
		#endif
		return 0;
	}
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Trying to write 4 bytes to page %d\n", page);
	#endif

	/* Prepare the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;                     /* Card number */
	pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE; /* Mifare Ultralight Write command = 0xA2 */
	pn532_packetbuffer[3] = page;   /* Page Number (0..63 for 1K, 0..255 for 4K) */

	/* Now add the 4 bytes of data to be written */
	memcpy(pn532_packetbuffer + 4, data, 4);

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 8, PN532_TIMEOUT)) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Failed to receive ACK for write command\n");
		#endif
		return 0;
	}

	HAL_Delay(10);

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);

	return 1;
}

uint8_t ntag2xx_ReadPage(uint8_t page, uint8_t *buffer)
{
	if (page >= 231) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Page value %d out of range\n", page);
		#endif
		return 0;
	}

	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Reading %d page.\n", page);
	#endif

	/* Prepare the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;               /* Card number */
	pn532_packetbuffer[2] = MIFARE_CMD_READ; /* Mifare Read command = 0x30 */
	pn532_packetbuffer[3] = page; /* Page Number (0..63 in most cases) */

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 4, PN532_TIMEOUT)) {
		#ifdef MIFAREDEBUG
		_config->log("[PN532] - Failed to receive ACK for write command\n");
		#endif
	    return 0;
	}

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Received: ");
		PrintHexChar(pn532_packetbuffer, 26);
	#endif

	/* If byte 8 isn't 0x00 we probably have an error */
	if (pn532_packetbuffer[7] == 0x00) {
		/* Copy the 4 data bytes to the output buffer         */
		/* Block content starts at byte 9 of a valid response */
		/* Note that the command actually reads 16 byte or 4  */
		/* pages at a time ... we simply discard the last 12  */
		/* bytes                                              */
		memcpy(buffer, pn532_packetbuffer + 8, 4);
	} else {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Unexpected response reading block: ");
		    PrintHexChar(pn532_packetbuffer, 26);
		#endif
	    return 0;
	}

	/* Display data for debug if requested */
	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Page %d: ", page);
		PrintHexChar(buffer, 4);
	#endif

	// Return OK signal
	return 1;
}

uint8_t ntag2xx_WritePage(uint8_t page, uint8_t *data)
{
	if ((page > 255) || (page < 4)) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Page value %d out of range\n", page);
		#endif
		return 0;
	}

	#ifdef MIFAREDEBUG
		_config->log("[PN532] - Writing page %d: ", page);
	#endif

	/* Prepare the command */
	pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
	pn532_packetbuffer[1] = 1;                     /* Card number */
	pn532_packetbuffer[2] = MIFARE_ULTRALIGHT_CMD_WRITE; /* Mifare Ultralight Write command = 0xA2 */
	pn532_packetbuffer[3] = page;   /* Page Number (0..63 in most cases) */

	/* Now add the 4 bytes of data to be written */
	memcpy(pn532_packetbuffer + 4, data, 4);

	/* Send the command */
	if (!sendCommandCheckAck(pn532_packetbuffer, 8, PN532_TIMEOUT)) {
		#ifdef MIFAREDEBUG
			_config->log("[PN532] - Failed to receive ACK for write command\n");
		#endif
	    return 0;
	}

	HAL_Delay(10);

	/* Read the response packet */
	readdata(pn532_packetbuffer, 26);

	return 1;
}

uint8_t ntag2xx_WriteNDEFURI(uint8_t uriIdentifier, char *url, uint8_t dataLen)
{
	uint8_t pageBuffer[4] = {0, 0, 0, 0};

	// Remove NDEF record overhead from the URI data (pageHeader below)
	uint8_t wrapperSize = 12;

	// Figure out how long the string is
	uint8_t len = strlen(url);

	// Make sure the URI payload will fit in dataLen (include 0xFE trailer)
	if ((len < 1) || (len + 1 > (dataLen - wrapperSize)))
		return 0;

	// Setup the record header
	// See NFCForum-TS-Type-2-Tag_1.1.pdf for details
	uint8_t pageHeader[12] = {
	    /* NDEF Lock Control TLV (must be first and always present) */
	    0x01, /* Tag Field (0x01 = Lock Control TLV) */
	    0x03, /* Payload Length (always 3) */
	    0xA0, /* The position inside the tag of the lock bytes (upper 4 = page address, lower 4 = byte offset) */
	    0x10, /* Size in bits of the lock area */
	    0x44, /* Size in bytes of a page and the number of bytes each lock bit can lock (4 bit + 4 bits) */
	    /* NDEF Message TLV - URI Record */
	    0x03,               /* Tag Field (0x03 = NDEF Message) */
	    (uint8_t)(len + 5), /* Payload Length (not including 0xFE trailer) */
	    0xD1, /* NDEF Record Header (TNF=0x1:Well known record + SR + ME + MB) */
	    0x01, /* Type Length for the record type indicator */
	    (uint8_t)(len + 1), /* Payload len */
	    0x55,               /* Record Type Indicator (0x55 or 'U' = URI Record) */
	    uriIdentifier       /* URI Prefix (ex. 0x01 = "http://www.") */
	};

	// Write 12 byte header (three pages of data starting at page 4)
	memcpy(pageBuffer, pageHeader, 4);
	if (!(ntag2xx_WritePage(4, pageBuffer)))
	    return 0;
	memcpy(pageBuffer, pageHeader + 4, 4);
	if (!(ntag2xx_WritePage(5, pageBuffer)))
	    return 0;
	memcpy(pageBuffer, pageHeader + 8, 4);
	if (!(ntag2xx_WritePage(6, pageBuffer)))
	    return 0;

	// Write URI (starting at page 7)
	uint8_t currentPage = 7;
	char *urlcopy = url;
	while (len) {
		if (len < 4) {
			memset(pageBuffer, 0, 4);
			memcpy(pageBuffer, urlcopy, len);
			pageBuffer[len] = 0xFE; // NDEF record footer
			if (!(ntag2xx_WritePage(currentPage, pageBuffer)))
				return 0;
			  // DONE!
			return 1;
	    } else if (len == 4) {
			memcpy(pageBuffer, urlcopy, len);
			if (!(ntag2xx_WritePage(currentPage, pageBuffer)))
				return 0;
			memset(pageBuffer, 0, 4);
			pageBuffer[0] = 0xFE; // NDEF record footer
			currentPage++;
			if (!(ntag2xx_WritePage(currentPage, pageBuffer)))
				return 0;
			  // DONE!
			return 1;
	    } else {
			// More than one page of data left
			memcpy(pageBuffer, urlcopy, 4);
			if (!(ntag2xx_WritePage(currentPage, pageBuffer)))
				return 0;
			currentPage++;
			urlcopy += 4;
			len -= 4;
	    }
	  }

	  // Seems that everything was OK (?!)
	  return 1;
}
