/*
 * modbus_slave.c
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

#include "modbus_slave.h"
#include <string.h>

// CRC table for Modbus CRC-16 calculation
static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/**
 * @brief Initialize Modbus slave
 */
Modbus_Status_t Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, uint8_t slave_address)
{
    if (!hmodbus || !huart) return MODBUS_STATUS_ERROR;
    
    // Initialize handle
    hmodbus->huart = huart;
    hmodbus->slave_address = slave_address;
    hmodbus->baudrate = 115200; // Default
    hmodbus->parity = 0; // None
    hmodbus->stop_bits = 1;
    
    // Clear buffers
    memset(hmodbus->rx_buffer, 0, sizeof(hmodbus->rx_buffer));
    memset(hmodbus->tx_buffer, 0, sizeof(hmodbus->tx_buffer));
    hmodbus->rx_length = 0;
    hmodbus->tx_length = 0;
    
    // Clear registers
    memset(hmodbus->holding_registers, 0, sizeof(hmodbus->holding_registers));
    memset(hmodbus->input_registers, 0, sizeof(hmodbus->input_registers));
    memset(hmodbus->register_map, 0, sizeof(hmodbus->register_map));
    hmodbus->register_count = 0;
    
    // Initialize status
    hmodbus->status = MODBUS_STATUS_OK;
    hmodbus->frame_count = 0;
    hmodbus->error_count = 0;
    hmodbus->exception_count = 0;
    hmodbus->last_activity_time = 0;
    
    // Initialize communication state
    hmodbus->frame_ready = false;
    hmodbus->response_pending = false;
    hmodbus->last_char_time = 0;
    
    // Setup register mapping
    if (Modbus_RegisterMapping(hmodbus) != MODBUS_STATUS_OK) {
        return MODBUS_STATUS_ERROR;
    }
    
    // Start receiving (try interrupt mode first)
    Modbus_Status_t rx_status = Modbus_StartReceive(hmodbus);
    if (rx_status != MODBUS_STATUS_OK) {
        // If interrupt mode fails, still return OK but mark as polling mode
        hmodbus->status = MODBUS_STATUS_OK;
        return MODBUS_STATUS_OK;
    }
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief Set Modbus configuration
 */
Modbus_Status_t Modbus_SetConfig(Modbus_Handle_t *hmodbus, uint32_t baudrate, uint8_t parity, uint8_t stop_bits)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    hmodbus->baudrate = baudrate;
    hmodbus->parity = parity;
    hmodbus->stop_bits = stop_bits;
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief Setup register mapping
 */
Modbus_Status_t Modbus_RegisterMapping(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    // uint16_t reg_index = 0; // Reserved for future register indexing
    
    // IMU Input Registers (Read-only)
    Modbus_AddRegister(hmodbus, REG_ACCEL_X, &hmodbus->input_registers[REG_ACCEL_X], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_ACCEL_Y, &hmodbus->input_registers[REG_ACCEL_Y], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_ACCEL_Z, &hmodbus->input_registers[REG_ACCEL_Z], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_GYRO_X, &hmodbus->input_registers[REG_GYRO_X], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_GYRO_Y, &hmodbus->input_registers[REG_GYRO_Y], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_GYRO_Z, &hmodbus->input_registers[REG_GYRO_Z], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_MAG_X, &hmodbus->input_registers[REG_MAG_X], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_MAG_Y, &hmodbus->input_registers[REG_MAG_Y], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_MAG_Z, &hmodbus->input_registers[REG_MAG_Z], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_IMU_STATUS, &hmodbus->input_registers[REG_IMU_STATUS], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_IMU_ERROR, &hmodbus->input_registers[REG_IMU_ERROR], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    
    // Digital Input Registers (Read-only)
    Modbus_AddRegister(hmodbus, REG_DI_1, &hmodbus->input_registers[REG_DI_1], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_DI_2, &hmodbus->input_registers[REG_DI_2], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_DI_3, &hmodbus->input_registers[REG_DI_3], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_DI_4, &hmodbus->input_registers[REG_DI_4], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_DI_STATUS, &hmodbus->input_registers[REG_DI_STATUS], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_DI_ERROR, &hmodbus->input_registers[REG_DI_ERROR], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    
    // PN532 Registers (Read-only)
    Modbus_AddRegister(hmodbus, REG_PN532_DATA_LOW, &hmodbus->input_registers[REG_PN532_DATA_LOW], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_PN532_DATA_HIGH, &hmodbus->input_registers[REG_PN532_DATA_HIGH], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_PN532_STATUS, &hmodbus->input_registers[REG_PN532_STATUS], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_PN532_ERROR, &hmodbus->input_registers[REG_PN532_ERROR], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_PN532_CARD_TYPE, &hmodbus->input_registers[REG_PN532_CARD_TYPE], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_PN532_CARD_UID, &hmodbus->input_registers[REG_PN532_CARD_UID], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    
    // Configuration Holding Registers (Read/Write)
    Modbus_AddRegister(hmodbus, REG_IMU_SAMPLE_RATE_CONFIG, &hmodbus->holding_registers[REG_IMU_SAMPLE_RATE_CONFIG], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_DI_DEBOUNCE_TIME_CONFIG, &hmodbus->holding_registers[REG_DI_DEBOUNCE_TIME_CONFIG], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_NFC_READ_TIMEOUT_CONFIG, &hmodbus->holding_registers[REG_NFC_READ_TIMEOUT_CONFIG], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_DATA_VALID_STATE, &hmodbus->holding_registers[REG_DATA_VALID_STATE], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_FAULT_REPORTING, &hmodbus->holding_registers[REG_FAULT_REPORTING], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    
    // System Registers
    Modbus_AddRegister(hmodbus, REG_DEVICE_ID, &hmodbus->holding_registers[REG_DEVICE_ID], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_CONFIG_BAUDRATE, &hmodbus->holding_registers[REG_CONFIG_BAUDRATE], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_CONFIG_PARITY, &hmodbus->holding_registers[REG_CONFIG_PARITY], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_CONFIG_STOP_BITS, &hmodbus->holding_registers[REG_CONFIG_STOP_BITS], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_READ_WRITE);
    Modbus_AddRegister(hmodbus, REG_MODULE_TYPE, &hmodbus->input_registers[REG_MODULE_TYPE], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_FIRMWARE_VERSION, &hmodbus->input_registers[REG_FIRMWARE_VERSION], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_HARDWARE_VERSION, &hmodbus->input_registers[REG_HARDWARE_VERSION], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_SYSTEM_STATUS, &hmodbus->input_registers[REG_SYSTEM_STATUS], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_SYSTEM_ERROR, &hmodbus->input_registers[REG_SYSTEM_ERROR], 
                      MODBUS_REG_TYPE_INPUT, MODBUS_REG_ACCESS_READ_ONLY);
    Modbus_AddRegister(hmodbus, REG_RESET_ERROR_CMD, &hmodbus->holding_registers[REG_RESET_ERROR_CMD], 
                      MODBUS_REG_TYPE_HOLDING, MODBUS_REG_ACCESS_WRITE_ONLY);
    
    // Set default values
    hmodbus->holding_registers[REG_DEVICE_ID] = 4; // Default slave address
    hmodbus->holding_registers[REG_CONFIG_BAUDRATE] = 4; // 19200
    hmodbus->holding_registers[REG_CONFIG_PARITY] = 0; // None
    hmodbus->holding_registers[REG_CONFIG_STOP_BITS] = 1; // 1 stop bit
    hmodbus->input_registers[REG_MODULE_TYPE] = 0x0002; // Power Module
    hmodbus->input_registers[REG_FIRMWARE_VERSION] = 0x0101; // v1.01
    hmodbus->input_registers[REG_HARDWARE_VERSION] = 0x0101; // v1.01
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief Add register to register map
 */
Modbus_Status_t Modbus_AddRegister(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t *data_ptr, 
                                  Modbus_RegType_t type, Modbus_RegAccess_t access)
{
    if (!hmodbus || !data_ptr || hmodbus->register_count >= MODBUS_MAX_REGISTERS) {
        return MODBUS_STATUS_ERROR;
    }
    
    hmodbus->register_map[hmodbus->register_count].address = address;
    hmodbus->register_map[hmodbus->register_count].data_ptr = data_ptr;
    hmodbus->register_map[hmodbus->register_count].type = type;
    hmodbus->register_map[hmodbus->register_count].access = access;
    hmodbus->register_map[hmodbus->register_count].valid = true;
    hmodbus->register_count++;
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief Set register value
 */
Modbus_Status_t Modbus_SetRegisterValue(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t value)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    for (uint16_t i = 0; i < hmodbus->register_count; i++) {
        if (hmodbus->register_map[i].address == address && hmodbus->register_map[i].valid) {
            if (hmodbus->register_map[i].access != MODBUS_REG_ACCESS_READ_ONLY) {
                *hmodbus->register_map[i].data_ptr = value;
                return MODBUS_STATUS_OK;
            }
            return MODBUS_STATUS_ERROR; // Read-only register
        }
    }
    
    return MODBUS_STATUS_ERROR; // Register not found
}

/**
 * @brief Get register value
 */
Modbus_Status_t Modbus_GetRegisterValue(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t *value)
{
    if (!hmodbus || !value) return MODBUS_STATUS_ERROR;
    
    for (uint16_t i = 0; i < hmodbus->register_count; i++) {
        if (hmodbus->register_map[i].address == address && hmodbus->register_map[i].valid) {
            if (hmodbus->register_map[i].access != MODBUS_REG_ACCESS_WRITE_ONLY) {
                *value = *hmodbus->register_map[i].data_ptr;
                return MODBUS_STATUS_OK;
            }
            return MODBUS_STATUS_ERROR; // Write-only register
        }
    }
    
    return MODBUS_STATUS_ERROR; // Register not found
}

/**
 * @brief Check if register is valid
 */
bool Modbus_IsValidRegister(Modbus_Handle_t *hmodbus, uint16_t address)
{
    if (!hmodbus) return false;
    
    for (uint16_t i = 0; i < hmodbus->register_count; i++) {
        if (hmodbus->register_map[i].address == address && hmodbus->register_map[i].valid) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Process Modbus communication
 */
Modbus_Status_t Modbus_Process(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    if (hmodbus->frame_ready) {
        hmodbus->frame_ready = false;
        return Modbus_ProcessFrame(hmodbus);
    }
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief Start receiving data
 */
Modbus_Status_t Modbus_StartReceive(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    hmodbus->rx_length = 0;
    
    // Check if UART is ready or already in RX mode
    if (hmodbus->huart->gState != HAL_UART_STATE_READY && 
        hmodbus->huart->gState != HAL_UART_STATE_BUSY_RX) {
        return MODBUS_STATUS_ERROR;
    }
    
    // If already in RX mode, abort current reception
    if (hmodbus->huart->gState == HAL_UART_STATE_BUSY_RX) {
        HAL_UART_AbortReceive_IT(hmodbus->huart);
        // Small delay to ensure abort completes
        HAL_Delay(1);
    }
    
    HAL_StatusTypeDef uart_status = HAL_UART_Receive_IT(hmodbus->huart, hmodbus->rx_buffer, 1);
    if (uart_status != HAL_OK) {
        return MODBUS_STATUS_ERROR;
    }
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief UART Rx callback
 */
void Modbus_RxCallback(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // Check for inter-frame delay
    if (hmodbus->rx_length > 0 && 
        (current_time - hmodbus->last_char_time) > Modbus_CalculateTimeout(hmodbus->baudrate)) {
        // Frame timeout, process current frame
        if (hmodbus->rx_length >= MODBUS_MIN_FRAME_SIZE) {
            hmodbus->frame_ready = true;
        }
        hmodbus->rx_length = 1; // Start new frame with current byte
    } else {
        hmodbus->rx_length++;
    }
    
    hmodbus->last_char_time = current_time;
    
    // Continue receiving if buffer not full
    if (hmodbus->rx_length < MODBUS_MAX_FRAME_SIZE) {
        HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[hmodbus->rx_length], 1);
    }
}

/**
 * @brief UART Tx callback
 */
void Modbus_TxCallback(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return;
    
    hmodbus->response_pending = false;
    
    // Resume receiving
    Modbus_StartReceive(hmodbus);
}

/**
 * @brief UART Error callback
 */
void Modbus_ErrorCallback(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return;
    
    hmodbus->error_count++;
    hmodbus->status = MODBUS_STATUS_ERROR;
    
    // Restart receiving
    Modbus_StartReceive(hmodbus);
}

/**
 * @brief Process received frame
 */
Modbus_Status_t Modbus_ProcessFrame(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus || hmodbus->rx_length < MODBUS_MIN_FRAME_SIZE) {
        return MODBUS_STATUS_INVALID_FRAME;
    }
    
    // Check CRC
    if (!Modbus_VerifyCRC(hmodbus->rx_buffer, hmodbus->rx_length)) {
        hmodbus->error_count++;
        return MODBUS_STATUS_CRC_ERROR;
    }
    
    uint8_t slave_addr = hmodbus->rx_buffer[0];
    uint8_t function_code = hmodbus->rx_buffer[1];
    
    // Check if frame is for this slave
    if (slave_addr != hmodbus->slave_address && slave_addr != 0) {
        return MODBUS_STATUS_OK; // Not for us, ignore
    }
    
    hmodbus->frame_count++;
    hmodbus->last_activity_time = HAL_GetTick();
    
    // Process based on function code
    Modbus_Status_t status = MODBUS_STATUS_OK;
    
    switch (function_code) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            status = Modbus_ProcessReadHoldingRegisters(hmodbus);
            break;
            
        case MODBUS_FC_READ_INPUT_REGISTERS:
            status = Modbus_ProcessReadInputRegisters(hmodbus);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            status = Modbus_ProcessWriteSingleRegister(hmodbus);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            status = Modbus_ProcessWriteMultipleRegisters(hmodbus);
            break;
            
        default:
            Modbus_SendException(hmodbus, function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            hmodbus->exception_count++;
            status = MODBUS_STATUS_EXCEPTION;
            break;
    }
    
    return status;
}

/**
 * @brief Process read holding registers command
 */
Modbus_Status_t Modbus_ProcessReadHoldingRegisters(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus || hmodbus->rx_length < 8) return MODBUS_STATUS_INVALID_FRAME;
    
    uint16_t start_address = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t register_count = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    
    // Validate parameters
    if (register_count == 0 || register_count > MODBUS_MAX_READ_REGISTERS) {
        Modbus_SendException(hmodbus, MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_STATUS_EXCEPTION;
    }
    
    // Build response
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    hmodbus->tx_buffer[2] = register_count * 2; // Byte count
    
    uint16_t response_index = 3;
    
    for (uint16_t i = 0; i < register_count; i++) {
        uint16_t reg_address = start_address + i;
        uint16_t reg_value = 0;
        
        if (Modbus_GetRegisterValue(hmodbus, reg_address, &reg_value) != MODBUS_STATUS_OK) {
            Modbus_SendException(hmodbus, MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return MODBUS_STATUS_EXCEPTION;
        }
        
        hmodbus->tx_buffer[response_index++] = (reg_value >> 8) & 0xFF;
        hmodbus->tx_buffer[response_index++] = reg_value & 0xFF;
    }
    
    hmodbus->tx_length = response_index;
    
    return Modbus_SendResponse(hmodbus);
}

/**
 * @brief Process read input registers command
 */
Modbus_Status_t Modbus_ProcessReadInputRegisters(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus || hmodbus->rx_length < 8) return MODBUS_STATUS_INVALID_FRAME;
    
    uint16_t start_address = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t register_count = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    
    // Validate parameters
    if (register_count == 0 || register_count > MODBUS_MAX_READ_REGISTERS) {
        Modbus_SendException(hmodbus, MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_STATUS_EXCEPTION;
    }
    
    // Build response
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = MODBUS_FC_READ_INPUT_REGISTERS;
    hmodbus->tx_buffer[2] = register_count * 2; // Byte count
    
    uint16_t response_index = 3;
    
    for (uint16_t i = 0; i < register_count; i++) {
        uint16_t reg_address = start_address + i;
        uint16_t reg_value = 0;
        
        if (Modbus_GetRegisterValue(hmodbus, reg_address, &reg_value) != MODBUS_STATUS_OK) {
            Modbus_SendException(hmodbus, MODBUS_FC_READ_INPUT_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return MODBUS_STATUS_EXCEPTION;
        }
        
        hmodbus->tx_buffer[response_index++] = (reg_value >> 8) & 0xFF;
        hmodbus->tx_buffer[response_index++] = reg_value & 0xFF;
    }
    
    hmodbus->tx_length = response_index;
    
    return Modbus_SendResponse(hmodbus);
}

/**
 * @brief Process write single register command
 */
Modbus_Status_t Modbus_ProcessWriteSingleRegister(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus || hmodbus->rx_length < 8) return MODBUS_STATUS_INVALID_FRAME;
    
    uint16_t register_address = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t register_value = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    
    // Write register
    if (Modbus_SetRegisterValue(hmodbus, register_address, register_value) != MODBUS_STATUS_OK) {
        Modbus_SendException(hmodbus, MODBUS_FC_WRITE_SINGLE_REGISTER, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return MODBUS_STATUS_EXCEPTION;
    }
    
    // Echo back the request as response
    memcpy(hmodbus->tx_buffer, hmodbus->rx_buffer, 6);
    hmodbus->tx_length = 6;
    
    return Modbus_SendResponse(hmodbus);
}

/**
 * @brief Process write multiple registers command
 */
Modbus_Status_t Modbus_ProcessWriteMultipleRegisters(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus || hmodbus->rx_length < 9) return MODBUS_STATUS_INVALID_FRAME;
    
    uint16_t start_address = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t register_count = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    uint8_t byte_count = hmodbus->rx_buffer[6];
    
    // Validate parameters
    if (register_count == 0 || register_count > MODBUS_MAX_WRITE_REGISTERS || 
        byte_count != register_count * 2 || hmodbus->rx_length < (9 + byte_count)) {
        Modbus_SendException(hmodbus, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_STATUS_EXCEPTION;
    }
    
    // Write registers
    uint16_t data_index = 7;
    for (uint16_t i = 0; i < register_count; i++) {
        uint16_t reg_address = start_address + i;
        uint16_t reg_value = (hmodbus->rx_buffer[data_index] << 8) | hmodbus->rx_buffer[data_index + 1];
        data_index += 2;
        
        if (Modbus_SetRegisterValue(hmodbus, reg_address, reg_value) != MODBUS_STATUS_OK) {
            Modbus_SendException(hmodbus, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return MODBUS_STATUS_EXCEPTION;
        }
    }
    
    // Build response
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    hmodbus->tx_buffer[2] = (start_address >> 8) & 0xFF;
    hmodbus->tx_buffer[3] = start_address & 0xFF;
    hmodbus->tx_buffer[4] = (register_count >> 8) & 0xFF;
    hmodbus->tx_buffer[5] = register_count & 0xFF;
    hmodbus->tx_length = 6;
    
    return Modbus_SendResponse(hmodbus);
}

/**
 * @brief Send response
 */
Modbus_Status_t Modbus_SendResponse(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    // Add CRC
    Modbus_AddCRC(hmodbus->tx_buffer, hmodbus->tx_length);
    hmodbus->tx_length += 2;
    
    // Send response
    hmodbus->response_pending = true;
    
    if (HAL_UART_Transmit_IT(hmodbus->huart, hmodbus->tx_buffer, hmodbus->tx_length) != HAL_OK) {
        hmodbus->response_pending = false;
        return MODBUS_STATUS_ERROR;
    }
    
    return MODBUS_STATUS_OK;
}

/**
 * @brief Send exception response
 */
Modbus_Status_t Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t function_code, uint8_t exception_code)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = function_code | 0x80; // Set exception bit
    hmodbus->tx_buffer[2] = exception_code;
    hmodbus->tx_length = 3;
    
    hmodbus->exception_count++;
    
    return Modbus_SendResponse(hmodbus);
}

/**
 * @brief Calculate CRC-16 for Modbus
 */
uint16_t Modbus_CalculateCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crc_table[index];
    }
    
    return crc;
}

/**
 * @brief Verify CRC of received frame
 */
bool Modbus_VerifyCRC(uint8_t *data, uint16_t length)
{
    if (length < 3) return false;
    
    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];
    uint16_t calculated_crc = Modbus_CalculateCRC(data, length - 2);
    
    return (received_crc == calculated_crc);
}

/**
 * @brief Add CRC to frame
 */
void Modbus_AddCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = Modbus_CalculateCRC(data, length);
    data[length] = crc & 0xFF;
    data[length + 1] = (crc >> 8) & 0xFF;
}

/**
 * @brief Calculate timeout based on baudrate
 */
uint32_t Modbus_CalculateTimeout(uint32_t baudrate)
{
    // Calculate inter-frame timeout (3.5 characters)
    uint32_t char_time_us = (11 * 1000000) / baudrate; // 11 bits per character
    uint32_t timeout_ms = (char_time_us * 35) / 10000; // 3.5 characters
    
    if (timeout_ms < 2) timeout_ms = 2; // Minimum 2ms
    
    return timeout_ms;
}

// Status and debug functions
Modbus_Status_t Modbus_GetStatus(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return MODBUS_STATUS_ERROR;
    return hmodbus->status;
}

uint32_t Modbus_GetFrameCount(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return 0;
    return hmodbus->frame_count;
}

uint32_t Modbus_GetErrorCount(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return 0;
    return hmodbus->error_count;
}

uint32_t Modbus_GetExceptionCount(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return 0;
    return hmodbus->exception_count;
}

void Modbus_ResetStatistics(Modbus_Handle_t *hmodbus)
{
    if (!hmodbus) return;
    
    hmodbus->frame_count = 0;
    hmodbus->error_count = 0;
    hmodbus->exception_count = 0;
}
