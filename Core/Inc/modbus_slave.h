/*
 * modbus_slave.h
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

#ifndef INC_MODBUS_SLAVE_H_
#define INC_MODBUS_SLAVE_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "modbus_regs.h"

// Modbus function codes
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10

// Modbus exception codes
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION           0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS       0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE         0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE       0x04

// Modbus frame limits
#define MODBUS_MAX_FRAME_SIZE       256
#define MODBUS_MIN_FRAME_SIZE       4
#define MODBUS_CRC_SIZE             2
#define MODBUS_SLAVE_ADDRESS_SIZE   1
#define MODBUS_FUNCTION_CODE_SIZE   1

// Modbus timeouts
#define MODBUS_TIMEOUT_MS           1000
#define MODBUS_INTER_FRAME_DELAY    3.5f  // milliseconds
#define MODBUS_INTER_CHAR_DELAY     1.5f  // milliseconds

// Register limits
#define MODBUS_MAX_REGISTERS        256
#define MODBUS_MAX_READ_REGISTERS   125
#define MODBUS_MAX_WRITE_REGISTERS  123

// Status
typedef enum {
    MODBUS_STATUS_OK = 0,
    MODBUS_STATUS_ERROR,
    MODBUS_STATUS_TIMEOUT,
    MODBUS_STATUS_CRC_ERROR,
    MODBUS_STATUS_EXCEPTION,
    MODBUS_STATUS_INVALID_FRAME
} Modbus_Status_t;

// Register types
typedef enum {
    MODBUS_REG_TYPE_HOLDING = 0,
    MODBUS_REG_TYPE_INPUT
} Modbus_RegType_t;

// Register access permissions
typedef enum {
    MODBUS_REG_ACCESS_READ_ONLY = 0,
    MODBUS_REG_ACCESS_WRITE_ONLY,
    MODBUS_REG_ACCESS_READ_WRITE
} Modbus_RegAccess_t;

// Register definition structure
typedef struct {
    uint16_t address;
    uint16_t *data_ptr;
    Modbus_RegType_t type;
    Modbus_RegAccess_t access;
    bool valid;
} Modbus_Register_t;

// Modbus slave handle structure
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t slave_address;
    uint32_t baudrate;
    uint8_t parity;
    uint8_t stop_bits;
    
    // Communication buffers
    uint8_t rx_buffer[MODBUS_MAX_FRAME_SIZE];
    uint8_t tx_buffer[MODBUS_MAX_FRAME_SIZE];
    uint16_t rx_length;
    uint16_t tx_length;
    
    // Register storage
    uint16_t holding_registers[MODBUS_MAX_REGISTERS];
    uint16_t input_registers[MODBUS_MAX_REGISTERS];
    Modbus_Register_t register_map[MODBUS_MAX_REGISTERS];
    uint16_t register_count;
    
    // Status and statistics
    Modbus_Status_t status;
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t exception_count;
    uint32_t last_activity_time;
    
    // Communication state
    bool frame_ready;
    bool response_pending;
    uint32_t last_char_time;
    
} Modbus_Handle_t;

// Function prototypes
Modbus_Status_t Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, uint8_t slave_address);
Modbus_Status_t Modbus_SetConfig(Modbus_Handle_t *hmodbus, uint32_t baudrate, uint8_t parity, uint8_t stop_bits);
Modbus_Status_t Modbus_RegisterMapping(Modbus_Handle_t *hmodbus);

// Register management
Modbus_Status_t Modbus_AddRegister(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t *data_ptr, 
                                  Modbus_RegType_t type, Modbus_RegAccess_t access);
Modbus_Status_t Modbus_SetRegisterValue(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t value);
Modbus_Status_t Modbus_GetRegisterValue(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t *value);
bool Modbus_IsValidRegister(Modbus_Handle_t *hmodbus, uint16_t address);

// Communication functions
Modbus_Status_t Modbus_Process(Modbus_Handle_t *hmodbus);
Modbus_Status_t Modbus_StartReceive(Modbus_Handle_t *hmodbus);
void Modbus_RxCallback(Modbus_Handle_t *hmodbus);
void Modbus_TxCallback(Modbus_Handle_t *hmodbus);
void Modbus_ErrorCallback(Modbus_Handle_t *hmodbus);

// Frame processing functions
Modbus_Status_t Modbus_ProcessFrame(Modbus_Handle_t *hmodbus);
Modbus_Status_t Modbus_ProcessReadHoldingRegisters(Modbus_Handle_t *hmodbus);
Modbus_Status_t Modbus_ProcessReadInputRegisters(Modbus_Handle_t *hmodbus);
Modbus_Status_t Modbus_ProcessWriteSingleRegister(Modbus_Handle_t *hmodbus);
Modbus_Status_t Modbus_ProcessWriteMultipleRegisters(Modbus_Handle_t *hmodbus);

// Response functions
Modbus_Status_t Modbus_SendResponse(Modbus_Handle_t *hmodbus);
Modbus_Status_t Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t function_code, uint8_t exception_code);

// Utility functions
uint16_t Modbus_CalculateCRC(uint8_t *data, uint16_t length);
bool Modbus_VerifyCRC(uint8_t *data, uint16_t length);
void Modbus_AddCRC(uint8_t *data, uint16_t length);
uint32_t Modbus_CalculateTimeout(uint32_t baudrate);

// Status and debug functions
Modbus_Status_t Modbus_GetStatus(Modbus_Handle_t *hmodbus);
uint32_t Modbus_GetFrameCount(Modbus_Handle_t *hmodbus);
uint32_t Modbus_GetErrorCount(Modbus_Handle_t *hmodbus);
uint32_t Modbus_GetExceptionCount(Modbus_Handle_t *hmodbus);
void Modbus_ResetStatistics(Modbus_Handle_t *hmodbus);

#endif /* INC_MODBUS_SLAVE_H_ */
