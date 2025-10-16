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

// Modbus RTU Configuration
#define MODBUS_SLAVE_DEFAULT_ADDRESS    0x04
#define MODBUS_BUFFER_SIZE              256
#define MODBUS_TIMEOUT_MS               100
#define MODBUS_T15_TICKS                2     // 1.5 character times
#define MODBUS_T35_TICKS                4     // 3.5 character times

// Modbus Function Codes
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10

// Modbus Exception Codes
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE   0x04

// Modbus Register Map Size
#define MODBUS_REG_COUNT                256

// Modbus Baudrate Options
typedef enum {
    MODBUS_BAUD_9600    = 9600,
    MODBUS_BAUD_19200   = 19200,
    MODBUS_BAUD_38400   = 38400,
    MODBUS_BAUD_57600   = 57600,
    MODBUS_BAUD_115200  = 115200
} Modbus_Baudrate_t;

// Modbus Parity Options
typedef enum {
    MODBUS_PARITY_NONE  = 0,
    MODBUS_PARITY_EVEN  = 1,
    MODBUS_PARITY_ODD   = 2
} Modbus_Parity_t;

// Modbus Stop Bits Options
typedef enum {
    MODBUS_STOPBITS_1   = 1,
    MODBUS_STOPBITS_2   = 2
} Modbus_StopBits_t;

// Modbus State Machine
typedef enum {
    MODBUS_STATE_IDLE,
    MODBUS_STATE_RECEIVING,
    MODBUS_STATE_PROCESSING,
    MODBUS_STATE_SENDING
} Modbus_State_t;

// Modbus Handle Structure
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t slave_address;
    uint16_t registers[MODBUS_REG_COUNT];
    
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];
    uint16_t rx_index;
    uint32_t last_byte_time;
    
    uint8_t tx_buffer[MODBUS_BUFFER_SIZE];
    uint16_t tx_length;
    
    Modbus_State_t state;
    
    uint32_t baudrate;
    Modbus_Parity_t parity;
    Modbus_StopBits_t stopbits;
    
    // Statistics
    uint32_t requests_processed;
    uint32_t errors_count;
    
    // Callbacks for register access
    void (*reg_read_callback)(uint16_t addr, uint16_t *value);
    void (*reg_write_callback)(uint16_t addr, uint16_t value);
    
} Modbus_Handle_t;

// Function Prototypes

/**
 * @brief Initialize Modbus RTU Slave
 * @param hmodbus: Pointer to Modbus handle
 * @param huart: Pointer to UART handle (UART2)
 * @param slave_addr: Modbus slave address (1-247)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, uint8_t slave_addr);

/**
 * @brief Set Modbus communication parameters
 * @param hmodbus: Pointer to Modbus handle
 * @param baudrate: Baudrate value (9600, 19200, 38400, 57600, 115200)
 * @param parity: Parity setting (NONE, EVEN, ODD)
 * @param stopbits: Stop bits (1 or 2)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Modbus_SetConfig(Modbus_Handle_t *hmodbus, uint32_t baudrate, 
                                   Modbus_Parity_t parity, Modbus_StopBits_t stopbits);

/**
 * @brief Main Modbus processing function (call in main loop or task)
 * @param hmodbus: Pointer to Modbus handle
 */
void Modbus_Process(Modbus_Handle_t *hmodbus);

/**
 * @brief Write value to Modbus register
 * @param hmodbus: Pointer to Modbus handle
 * @param address: Register address
 * @param value: Value to write
 * @return true if successful
 */
bool Modbus_WriteRegister(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t value);

/**
 * @brief Read value from Modbus register
 * @param hmodbus: Pointer to Modbus handle
 * @param address: Register address
 * @param value: Pointer to store read value
 * @return true if successful
 */
bool Modbus_ReadRegister(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t *value);

/**
 * @brief Register callback for register read operations
 * @param hmodbus: Pointer to Modbus handle
 * @param callback: Callback function
 */
void Modbus_RegisterReadCallback(Modbus_Handle_t *hmodbus, 
                                  void (*callback)(uint16_t addr, uint16_t *value));

/**
 * @brief Register callback for register write operations
 * @param hmodbus: Pointer to Modbus handle
 * @param callback: Callback function
 */
void Modbus_RegisterWriteCallback(Modbus_Handle_t *hmodbus, 
                                   void (*callback)(uint16_t addr, uint16_t value));

/**
 * @brief UART Receive Interrupt Handler (call from HAL_UART_RxCpltCallback)
 * @param hmodbus: Pointer to Modbus handle
 * @param byte: Received byte
 */
void Modbus_UART_RxCallback(Modbus_Handle_t *hmodbus, uint8_t byte);

/**
 * @brief Calculate CRC16 for Modbus RTU
 * @param buffer: Data buffer
 * @param length: Buffer length
 * @return CRC16 value
 */
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length);

/**
 * @brief Get Modbus statistics
 * @param hmodbus: Pointer to Modbus handle
 * @param requests: Pointer to store request count
 * @param errors: Pointer to store error count
 */
void Modbus_GetStats(Modbus_Handle_t *hmodbus, uint32_t *requests, uint32_t *errors);

#endif /* INC_MODBUS_SLAVE_H_ */
