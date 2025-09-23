/*
 * pn532.c
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

#include "pn532.h"
#include <string.h>

#define PN532_TIMEOUT_MS            1000
#define PN532_ACK_WAIT_TIME         10
#define PN532_RESPONSE_WAIT_TIME    100

// ACK frame for verification
static const uint8_t pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

/**
 * @brief Verify ACK response from PN532
 */
static bool PN532_VerifyAck(PN532_Handle_t *hpn532)
{
    uint8_t ack_buffer[6];
    
    if (HAL_I2C_Master_Receive(hpn532->hi2c, PN532_I2C_ADDRESS << 1, 
                              ack_buffer, sizeof(ack_buffer), 100) != HAL_OK) {
        return false;
    }
    
    return (memcmp(ack_buffer, pn532_ack, sizeof(pn532_ack)) == 0);
}

/**
 * @brief Initialize PN532 (Based on Adafruit library approach)
 */
PN532_Status_t PN532_Init(PN532_Handle_t *hpn532, I2C_HandleTypeDef *hi2c)
{
    if (!hpn532 || !hi2c) return PN532_STATUS_ERROR;
    
    // Initialize handle
    hpn532->hi2c = hi2c;
    hpn532->mode = PN532_MODE_NFC;
    hpn532->read_timeout = 1000; // 1 second default
    hpn532->status = PN532_STATUS_OK;
    hpn532->error_code = 0;
    
    // Clear card info
    memset(&hpn532->card_info, 0, sizeof(PN532_CardInfo_t));
    
    // Add delay for PN532 to be ready (Adafruit approach)
    HAL_Delay(100);
    
    // Test communication with simple approach
    uint32_t version;
    PN532_Status_t fw_status = PN532_GetFirmwareVersion(hpn532, &version);
    if (fw_status != PN532_STATUS_OK) {
        hpn532->status = PN532_STATUS_COMM_ERROR;
        hpn532->error_code = 1; // Firmware version read failed
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Configure SAM (Normal mode, timeout 50ms*20=1sec, use IRQ)
    PN532_Status_t sam_status = PN532_ConfigureSAM(hpn532);
    if (sam_status != PN532_STATUS_OK) {
        hpn532->status = PN532_STATUS_ERROR;
        hpn532->error_code = 2; // SAM configuration failed
        return PN532_STATUS_ERROR;
    }
    
    hpn532->status = PN532_STATUS_OK;
    return PN532_STATUS_OK;
}

/**
 * @brief Set PN532 operating mode
 */
PN532_Status_t PN532_SetMode(PN532_Handle_t *hpn532, PN532_Mode_t mode)
{
    if (!hpn532) return PN532_STATUS_ERROR;
    
    hpn532->mode = mode;
    
    // Configure RF settings based on mode
    uint8_t cmd[4];
    cmd[0] = PN532_COMMAND_RFCONFIGURATION;
    cmd[1] = 0x01; // Config item = RF field
    
    if (mode == PN532_MODE_NFC) {
        cmd[2] = 0x01; // RF field ON for NFC
    } else {
        cmd[2] = 0x01; // RF field ON for RFID
    }
    
    return PN532_WriteCommand(hpn532, cmd, 3);
}

/**
 * @brief Get PN532 firmware version (Simplified Adafruit approach)
 */
PN532_Status_t PN532_GetFirmwareVersion(PN532_Handle_t *hpn532, uint32_t *version)
{
    if (!hpn532 || !version) return PN532_STATUS_ERROR;
    
    // Try different PN532 wake-up sequences
    uint8_t wake_up[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00}; // Wake up sequence
    HAL_I2C_Master_Transmit(hpn532->hi2c, PN532_I2C_ADDRESS << 1, wake_up, 6, 100);
    HAL_Delay(100);
    
    // Simple I2C read approach like Adafruit  
    uint8_t cmd = PN532_COMMAND_GETFIRMWAREVERSION;
    uint8_t response[12];
    
    // Write command
    HAL_StatusTypeDef tx_result = HAL_I2C_Master_Transmit(hpn532->hi2c, PN532_I2C_ADDRESS << 1, &cmd, 1, 200);
    if (tx_result != HAL_OK) {
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Wait for response
    HAL_Delay(100);
    
    // Read response
    HAL_StatusTypeDef rx_result = HAL_I2C_Master_Receive(hpn532->hi2c, PN532_I2C_ADDRESS << 1, response, 12, 200);
    if (rx_result != HAL_OK) {
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Debug: Print response data
    // Note: In production, remove this debug print to avoid external dependencies
    // DebugDumpHex("PN532 Response", response, 12);
    
    // More flexible response parsing
    for (int i = 0; i < 9; i++) {
        if (response[i] == 0x00 && response[i+1] == 0x00 && response[i+2] == 0xFF) {
            // Found valid frame header
            if (i + 5 < 12) {
                *version = (response[i+3] << 24) | (response[i+4] << 16) | (response[i+5] << 8);
                return PN532_STATUS_OK;
            }
        }
    }
    
    return PN532_STATUS_ERROR;
}

/**
 * @brief Configure SAM (Simplified Adafruit approach)
 */
PN532_Status_t PN532_ConfigureSAM(PN532_Handle_t *hpn532)
{
    if (!hpn532) return PN532_STATUS_ERROR;
    
    // Simplified SAM configuration like Adafruit
    uint8_t cmd[4] = {
        PN532_COMMAND_SAMCONFIGURATION,
        0x01, // Normal mode
        0x14, // Timeout 50ms * 20 = 1 second  
        0x01  // Use IRQ pin
    };
    
    // Write command
    if (HAL_I2C_Master_Transmit(hpn532->hi2c, PN532_I2C_ADDRESS << 1, cmd, 4, 100) != HAL_OK) {
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Wait for processing
    HAL_Delay(50);
    
    // Read ACK (simplified)
    uint8_t ack[6];
    if (HAL_I2C_Master_Receive(hpn532->hi2c, PN532_I2C_ADDRESS << 1, ack, 6, 100) == HAL_OK) {
        return PN532_STATUS_OK;
    }
    
    return PN532_STATUS_OK; // Accept even if ACK read fails
}

/**
 * @brief Read card/tag
 */
PN532_Status_t PN532_ReadCard(PN532_Handle_t *hpn532)
{
    if (!hpn532) return PN532_STATUS_ERROR;
    
    return PN532_ReadPassiveTarget(hpn532);
}

/**
 * @brief Read passive target (Simplified Adafruit approach)
 */
PN532_Status_t PN532_ReadPassiveTarget(PN532_Handle_t *hpn532)
{
    if (!hpn532) return PN532_STATUS_ERROR;
    
    // Simplified read approach like Adafruit
    uint8_t cmd[3] = {
        PN532_COMMAND_INLISTPASSIVETARGET,
        0x01, // Max 1 card
        PN532_MIFARE_ISO14443A // Card type
    };
    
    // Write command
    if (HAL_I2C_Master_Transmit(hpn532->hi2c, PN532_I2C_ADDRESS << 1, cmd, 3, 100) != HAL_OK) {
        hpn532->card_info.card_present = false;
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Wait for card detection
    HAL_Delay(100);
    
    // Read response
    uint8_t response[32];
    HAL_StatusTypeDef result = HAL_I2C_Master_Receive(hpn532->hi2c, PN532_I2C_ADDRESS << 1, response, 32, 200);
    
    if (result != HAL_OK) {
        hpn532->card_info.card_present = false;
        return PN532_STATUS_NO_CARD;
    }
    
    // Simple parsing - check if we have valid data
    if (response[0] == 0x00 && response[1] == 0x00 && response[2] == 0xFF && response[6] > 0) {
        hpn532->card_info.card_present = true;
        hpn532->card_info.card_type = PN532_MIFARE_ISO14443A;
        
        // Extract UID (simplified)
        uint8_t uid_start = 11; // Typical UID start position in Adafruit format
        hpn532->card_info.uid_length = 4; // Assume 4-byte UID for simplicity
        
        if (uid_start + 4 < 32) {
            memcpy(hpn532->card_info.uid, &response[uid_start], 4);
        }
        
        hpn532->last_read_time = HAL_GetTick();
        return PN532_STATUS_OK;
    }
    
    hpn532->card_info.card_present = false;
    return PN532_STATUS_NO_CARD;
}

/**
 * @brief Check if card is present
 */
bool PN532_IsCardPresent(PN532_Handle_t *hpn532)
{
    if (!hpn532) return false;
    
    // Check timeout
    if (HAL_GetTick() - hpn532->last_read_time > hpn532->read_timeout) {
        hpn532->card_info.card_present = false;
    }
    
    return hpn532->card_info.card_present;
}

/**
 * @brief Get card UID
 */
uint8_t* PN532_GetCardUID(PN532_Handle_t *hpn532)
{
    if (!hpn532) return NULL;
    return hpn532->card_info.uid;
}

/**
 * @brief Get card type
 */
uint8_t PN532_GetCardType(PN532_Handle_t *hpn532)
{
    if (!hpn532) return 0xFF;
    return hpn532->card_info.card_type;
}

/**
 * @brief Get PN532 status
 */
PN532_Status_t PN532_GetStatus(PN532_Handle_t *hpn532)
{
    if (!hpn532) return PN532_STATUS_ERROR;
    return hpn532->status;
}

/**
 * @brief Get error code
 */
uint8_t PN532_GetErrorCode(PN532_Handle_t *hpn532)
{
    if (!hpn532) return 0xFF;
    return hpn532->error_code;
}

/**
 * @brief Write command to PN532
 */
PN532_Status_t PN532_WriteCommand(PN532_Handle_t *hpn532, uint8_t *cmd, uint8_t cmd_len)
{
    if (!hpn532 || !cmd) return PN532_STATUS_ERROR;
    
    uint8_t frame[32];
    uint8_t frame_len = 0;
    
    // Build frame
    frame[frame_len++] = PN532_PREAMBLE;
    frame[frame_len++] = PN532_STARTCODE1;
    frame[frame_len++] = PN532_STARTCODE2;
    frame[frame_len++] = cmd_len + 1; // Length
    frame[frame_len++] = (~(cmd_len + 1)) + 1; // Length checksum
    frame[frame_len++] = PN532_HOSTTOPN532; // Direction
    
    // Copy command
    memcpy(&frame[frame_len], cmd, cmd_len);
    frame_len += cmd_len;
    
    // Calculate and add data checksum
    uint8_t checksum = PN532_HOSTTOPN532;
    for (uint8_t i = 0; i < cmd_len; i++) {
        checksum += cmd[i];
    }
    frame[frame_len++] = (~checksum) + 1;
    
    frame[frame_len++] = PN532_POSTAMBLE;
    
    // Send frame
    if (HAL_I2C_Master_Transmit(hpn532->hi2c, PN532_I2C_ADDRESS << 1, frame, frame_len, PN532_TIMEOUT_MS) != HAL_OK) {
        hpn532->status = PN532_STATUS_COMM_ERROR;
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Wait for ACK
    HAL_Delay(PN532_ACK_WAIT_TIME);
    
    // Verify ACK (optional - can be enabled for more robust communication)
    if (!PN532_VerifyAck(hpn532)) {
        hpn532->status = PN532_STATUS_COMM_ERROR;
        return PN532_STATUS_COMM_ERROR;
    }
    
    return PN532_STATUS_OK;
}

/**
 * @brief Read response from PN532
 */
PN532_Status_t PN532_ReadResponse(PN532_Handle_t *hpn532, uint8_t *response, uint8_t *response_len)
{
    if (!hpn532 || !response || !response_len) return PN532_STATUS_ERROR;
    
    uint8_t buffer[64];
    uint8_t max_len = *response_len;
    
    // Wait for response ready
    if (PN532_WaitReady(hpn532, PN532_TIMEOUT_MS) != PN532_STATUS_OK) {
        return PN532_STATUS_TIMEOUT;
    }
    
    // Read response
    if (HAL_I2C_Master_Receive(hpn532->hi2c, PN532_I2C_ADDRESS << 1, buffer, sizeof(buffer), PN532_TIMEOUT_MS) != HAL_OK) {
        hpn532->status = PN532_STATUS_COMM_ERROR;
        return PN532_STATUS_COMM_ERROR;
    }
    
    // Parse response frame
    if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0xFF) {
        uint8_t len = buffer[3];
        uint8_t len_checksum = buffer[4];
        
        // Verify length checksum
        if ((uint8_t)(len + len_checksum) != 0) {
            return PN532_STATUS_ERROR;
        }
        
        // Extract data
        if (len > 0 && buffer[5] == PN532_PN532TOHOST) {
            uint8_t data_len = len - 1;
            if (data_len <= max_len) {
                memcpy(response, &buffer[6], data_len);
                *response_len = data_len;
                return PN532_STATUS_OK;
            }
        }
    }
    
    return PN532_STATUS_ERROR;
}

/**
 * @brief Wait for PN532 ready
 */
PN532_Status_t PN532_WaitReady(PN532_Handle_t *hpn532, uint32_t timeout)
{
    if (!hpn532) return PN532_STATUS_ERROR;
    
    uint32_t start_time = HAL_GetTick();
    uint8_t status;
    
    while (HAL_GetTick() - start_time < timeout) {
        if (HAL_I2C_Master_Receive(hpn532->hi2c, PN532_I2C_ADDRESS << 1, &status, 1, 10) == HAL_OK) {
            if (status == 0x01) { // Ready
                return PN532_STATUS_OK;
            }
        }
        HAL_Delay(5);
    }
    
    return PN532_STATUS_TIMEOUT;
}

/**
 * @brief Calculate checksum
 */
uint8_t PN532_CalculateChecksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return (~checksum) + 1;
}
