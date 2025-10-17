/*
 * modbus_slave.c
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

#include "modbus_slave.h"
#include "main.h"
#include <string.h>

// Private function prototypes
static void Modbus_ProcessFrame(Modbus_Handle_t *hmodbus);
static void Modbus_SendResponse(Modbus_Handle_t *hmodbus);
static void Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t function_code, uint8_t exception_code);
static void Modbus_HandleReadHoldingRegisters(Modbus_Handle_t *hmodbus);
static void Modbus_HandleReadInputRegisters(Modbus_Handle_t *hmodbus);
static void Modbus_HandleWriteSingleRegister(Modbus_Handle_t *hmodbus);
static void Modbus_HandleWriteMultipleRegisters(Modbus_Handle_t *hmodbus);
static bool Modbus_VerifyCRC(uint8_t *buffer, uint16_t length);
static HAL_StatusTypeDef Modbus_ReconfigureUART(Modbus_Handle_t *hmodbus);

/**
 * @brief Initialize Modbus RTU Slave
 */
HAL_StatusTypeDef Modbus_Init(Modbus_Handle_t *hmodbus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t slave_addr)
{
    if (hmodbus == NULL || huart == NULL || htim == NULL) {
        return HAL_ERROR;
    }
    
    // Check valid slave address (1-247)
    if (slave_addr == 0 || slave_addr > 247) {
        return HAL_ERROR;
    }
    
    // Initialize handle
    memset(hmodbus, 0, sizeof(Modbus_Handle_t));
    
    hmodbus->huart = huart;
    hmodbus->htim = htim;
    hmodbus->slave_address = slave_addr;
    hmodbus->state = MODBUS_STATE_IDLE;
    
    // Set default communication parameters
    hmodbus->baudrate = MODBUS_BAUD_115200;
    hmodbus->parity = MODBUS_PARITY_NONE;
    hmodbus->stopbits = MODBUS_STOPBITS_1;
    
    // Initialize registers to zero
    memset(hmodbus->registers, 0, sizeof(hmodbus->registers));
    
    // Initialize statistics
    hmodbus->requests_processed = 0;
    hmodbus->errors_count = 0;
    
    // Enable UART receive interrupt for single byte reception
    HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[0], 1);
    
    // Timer sẽ được start khi nhận byte đầu tiên
    
    return HAL_OK;
}

/**
 * @brief Set Modbus communication parameters
 */
HAL_StatusTypeDef Modbus_SetConfig(Modbus_Handle_t *hmodbus, uint32_t baudrate, 
                                   Modbus_Parity_t parity, Modbus_StopBits_t stopbits)
{
    if (hmodbus == NULL) {
        return HAL_ERROR;
    }
    
    hmodbus->baudrate = baudrate;
    hmodbus->parity = parity;
    hmodbus->stopbits = stopbits;
    
    // Reconfigure UART with new parameters
    return Modbus_ReconfigureUART(hmodbus);
}

/**
 * @brief Reconfigure UART with current Modbus parameters
 */
static HAL_StatusTypeDef Modbus_ReconfigureUART(Modbus_Handle_t *hmodbus)
{
    HAL_StatusTypeDef status;
    
    // Deinitialize UART
    HAL_UART_DeInit(hmodbus->huart);
    
    // Configure new parameters
    hmodbus->huart->Init.BaudRate = hmodbus->baudrate;
    hmodbus->huart->Init.WordLength = UART_WORDLENGTH_8B;
    
    // Set stop bits
    if (hmodbus->stopbits == MODBUS_STOPBITS_2) {
        hmodbus->huart->Init.StopBits = UART_STOPBITS_2;
    } else {
        hmodbus->huart->Init.StopBits = UART_STOPBITS_1;
    }
    
    // Set parity
    switch (hmodbus->parity) {
        case MODBUS_PARITY_EVEN:
            hmodbus->huart->Init.Parity = UART_PARITY_EVEN;
            hmodbus->huart->Init.WordLength = UART_WORDLENGTH_9B; // 8 data bits + parity
            break;
        case MODBUS_PARITY_ODD:
            hmodbus->huart->Init.Parity = UART_PARITY_ODD;
            hmodbus->huart->Init.WordLength = UART_WORDLENGTH_9B; // 8 data bits + parity
            break;
        default:
            hmodbus->huart->Init.Parity = UART_PARITY_NONE;
            break;
    }
    
    hmodbus->huart->Init.Mode = UART_MODE_TX_RX;
    hmodbus->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hmodbus->huart->Init.OverSampling = UART_OVERSAMPLING_16;
    
    // Reinitialize UART
    status = HAL_UART_Init(hmodbus->huart);
    
    if (status == HAL_OK) {
        // Restart receive interrupt
        HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[0], 1);
    }
    
    return status;
}

/**
 * @brief Calculate CRC16 for Modbus RTU (Polynomial 0xA001)
 */
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Verify CRC of received frame
 */
static bool Modbus_VerifyCRC(uint8_t *buffer, uint16_t length)
{
    if (length < 4) { // Minimum frame: addr + func + CRC(2)
        return false;
    }
    
    uint16_t received_crc = buffer[length - 2] | (buffer[length - 1] << 8);
    uint16_t calculated_crc = Modbus_CRC16(buffer, length - 2);
    
    return (received_crc == calculated_crc);
}

/**
 * @brief UART Receive Callback - call from HAL_UART_RxCpltCallback
 */
void Modbus_UART_RxCallback(Modbus_Handle_t *hmodbus, uint8_t byte)
{
    // Store received byte
    if (hmodbus->rx_index < MODBUS_BUFFER_SIZE - 1) {
        hmodbus->rx_buffer[hmodbus->rx_index] = byte;
        hmodbus->rx_index++;
        hmodbus->last_byte_time = HAL_GetTick();
        hmodbus->state = MODBUS_STATE_RECEIVING;
        
        // Reset và restart timer cho frame timeout (T3.5)
        __HAL_TIM_SET_COUNTER(hmodbus->htim, 0);
        if (hmodbus->rx_index == 1) {
            // Byte đầu tiên - start timer với interrupt
            HAL_TIM_Base_Start_IT(hmodbus->htim);
        }
        
        // Continue receiving next byte
        HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[hmodbus->rx_index], 1);
    } else {
        // Buffer overflow, reset
        hmodbus->rx_index = 0;
        HAL_TIM_Base_Stop_IT(hmodbus->htim);
        HAL_UART_Receive_IT(hmodbus->huart, &hmodbus->rx_buffer[0], 1);
    }
}

/**
 * @brief Main Modbus processing function
 * Khi sử dụng timer, function này chỉ cần gọi nhẹ để check state
 * Frame timeout được xử lý bởi timer interrupt
 */
void Modbus_Process(Modbus_Handle_t *hmodbus)
{
    // Với timer interrupt, chỉ cần check state SENDING
    if (hmodbus->state == MODBUS_STATE_SENDING) {
        // Wait for transmission to complete
        if (hmodbus->huart->gState == HAL_UART_STATE_READY) {
            hmodbus->state = MODBUS_STATE_IDLE;
        }
    }
    
    // Các state khác được xử lý bởi interrupt:
    // - RECEIVING: Timer sẽ trigger khi timeout
    // - PROCESSING: Xử lý trong timer callback
    // - IDLE: Không cần làm gì
}

/**
 * @brief Process received Modbus frame
 */
static void Modbus_ProcessFrame(Modbus_Handle_t *hmodbus)
{
    // Check minimum frame length (addr + func + CRC)
    if (hmodbus->rx_index < 4) {
        hmodbus->errors_count++;
        return;
    }
    
    // Verify CRC
    if (!Modbus_VerifyCRC(hmodbus->rx_buffer, hmodbus->rx_index)) {
        hmodbus->errors_count++;
        return;
    }
    
    // Check if frame is for this slave
    uint8_t received_addr = hmodbus->rx_buffer[0];
    if (received_addr != hmodbus->slave_address && received_addr != 0) {
        // Not for us, ignore
        return;
    }
    
    // Bật LED_MB khi nhận frame thành công
    HAL_GPIO_WritePin(LED_MB_GPIO_Port, LED_MB_Pin, GPIO_PIN_SET);
    
    // Extract function code
    uint8_t function_code = hmodbus->rx_buffer[1];
    
    // Process based on function code
    switch (function_code) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            Modbus_HandleReadHoldingRegisters(hmodbus);
            break;
            
        case MODBUS_FC_READ_INPUT_REGISTERS:
            Modbus_HandleReadInputRegisters(hmodbus);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            Modbus_HandleWriteSingleRegister(hmodbus);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            Modbus_HandleWriteMultipleRegisters(hmodbus);
            break;
            
        default:
            Modbus_SendException(hmodbus, function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            hmodbus->errors_count++;
            break;
    }
    
    hmodbus->requests_processed++;
}

/**
 * @brief Handle Read Holding Registers (FC 0x03)
 */
static void Modbus_HandleReadHoldingRegisters(Modbus_Handle_t *hmodbus)
{
    // Frame format: addr + func + start_addr(2) + num_regs(2) + CRC(2) = 8 bytes
    if (hmodbus->rx_index != 8) {
        Modbus_SendException(hmodbus, MODBUS_FC_READ_HOLDING_REGISTERS, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t start_addr = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t num_regs = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    
    // Validate request
    if (num_regs == 0 || num_regs > 125 || 
        start_addr >= MODBUS_REG_COUNT || 
        (start_addr + num_regs) > MODBUS_REG_COUNT) {
        Modbus_SendException(hmodbus, MODBUS_FC_READ_HOLDING_REGISTERS, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // Build response
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    hmodbus->tx_buffer[2] = num_regs * 2; // Byte count
    
    // Fill register values
    for (uint16_t i = 0; i < num_regs; i++) {
        uint16_t reg_addr = start_addr + i;
        uint16_t value = hmodbus->registers[reg_addr];
        
        // Call read callback if registered
        if (hmodbus->reg_read_callback != NULL) {
            hmodbus->reg_read_callback(reg_addr, &value);
        }
        
        hmodbus->tx_buffer[3 + i * 2] = (value >> 8) & 0xFF;
        hmodbus->tx_buffer[4 + i * 2] = value & 0xFF;
    }
    
    hmodbus->tx_length = 3 + num_regs * 2;
    Modbus_SendResponse(hmodbus);
}

/**
 * @brief Handle Read Input Registers (FC 0x04)
 */
static void Modbus_HandleReadInputRegisters(Modbus_Handle_t *hmodbus)
{
    // Same implementation as Read Holding Registers for slave
    // Frame format: addr + func + start_addr(2) + num_regs(2) + CRC(2) = 8 bytes
    if (hmodbus->rx_index != 8) {
        Modbus_SendException(hmodbus, MODBUS_FC_READ_INPUT_REGISTERS, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t start_addr = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t num_regs = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    
    // Validate request
    if (num_regs == 0 || num_regs > 125 || 
        start_addr >= MODBUS_REG_COUNT || 
        (start_addr + num_regs) > MODBUS_REG_COUNT) {
        Modbus_SendException(hmodbus, MODBUS_FC_READ_INPUT_REGISTERS, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // Build response
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = MODBUS_FC_READ_INPUT_REGISTERS;
    hmodbus->tx_buffer[2] = num_regs * 2; // Byte count
    
    // Fill register values
    for (uint16_t i = 0; i < num_regs; i++) {
        uint16_t reg_addr = start_addr + i;
        uint16_t value = hmodbus->registers[reg_addr];
        
        // Call read callback if registered
        if (hmodbus->reg_read_callback != NULL) {
            hmodbus->reg_read_callback(reg_addr, &value);
        }
        
        hmodbus->tx_buffer[3 + i * 2] = (value >> 8) & 0xFF;
        hmodbus->tx_buffer[4 + i * 2] = value & 0xFF;
    }
    
    hmodbus->tx_length = 3 + num_regs * 2;
    Modbus_SendResponse(hmodbus);
}

/**
 * @brief Handle Write Single Register (FC 0x06)
 */
static void Modbus_HandleWriteSingleRegister(Modbus_Handle_t *hmodbus)
{
    // Frame format: addr + func + reg_addr(2) + value(2) + CRC(2) = 8 bytes
    if (hmodbus->rx_index != 8) {
        Modbus_SendException(hmodbus, MODBUS_FC_WRITE_SINGLE_REGISTER, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t reg_addr = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t value = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    
    // Validate address
    if (reg_addr >= MODBUS_REG_COUNT) {
        Modbus_SendException(hmodbus, MODBUS_FC_WRITE_SINGLE_REGISTER, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // Write register
    hmodbus->registers[reg_addr] = value;
    
    // Call write callback if registered
    if (hmodbus->reg_write_callback != NULL) {
        hmodbus->reg_write_callback(reg_addr, value);
    }
    
    // Echo request as response (standard Modbus behavior)
    memcpy(hmodbus->tx_buffer, hmodbus->rx_buffer, 6);
    hmodbus->tx_length = 6;
    Modbus_SendResponse(hmodbus);
}

/**
 * @brief Handle Write Multiple Registers (FC 0x10)
 */
static void Modbus_HandleWriteMultipleRegisters(Modbus_Handle_t *hmodbus)
{
    // Minimum frame: addr + func + start_addr(2) + num_regs(2) + byte_count(1) + data(2*N) + CRC(2)
    if (hmodbus->rx_index < 11) {
        Modbus_SendException(hmodbus, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t start_addr = (hmodbus->rx_buffer[2] << 8) | hmodbus->rx_buffer[3];
    uint16_t num_regs = (hmodbus->rx_buffer[4] << 8) | hmodbus->rx_buffer[5];
    uint8_t byte_count = hmodbus->rx_buffer[6];
    
    // Validate request
    if (num_regs == 0 || num_regs > 123 || 
        byte_count != (num_regs * 2) ||
        start_addr >= MODBUS_REG_COUNT || 
        (start_addr + num_regs) > MODBUS_REG_COUNT ||
        hmodbus->rx_index != (9 + byte_count)) {
        Modbus_SendException(hmodbus, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 
                           MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // Write registers
    for (uint16_t i = 0; i < num_regs; i++) {
        uint16_t reg_addr = start_addr + i;
        uint16_t value = (hmodbus->rx_buffer[7 + i * 2] << 8) | 
                         hmodbus->rx_buffer[8 + i * 2];
        
        hmodbus->registers[reg_addr] = value;
        
        // Call write callback if registered
        if (hmodbus->reg_write_callback != NULL) {
            hmodbus->reg_write_callback(reg_addr, value);
        }
    }
    
    // Build response: addr + func + start_addr(2) + num_regs(2) + CRC(2)
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    hmodbus->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    hmodbus->tx_buffer[3] = start_addr & 0xFF;
    hmodbus->tx_buffer[4] = (num_regs >> 8) & 0xFF;
    hmodbus->tx_buffer[5] = num_regs & 0xFF;
    
    hmodbus->tx_length = 6;
    Modbus_SendResponse(hmodbus);
}

/**
 * @brief Send Modbus response
 */
static void Modbus_SendResponse(Modbus_Handle_t *hmodbus)
{
    // Calculate and append CRC
    uint16_t crc = Modbus_CRC16(hmodbus->tx_buffer, hmodbus->tx_length);
    hmodbus->tx_buffer[hmodbus->tx_length++] = crc & 0xFF;
    hmodbus->tx_buffer[hmodbus->tx_length++] = (crc >> 8) & 0xFF;
    
    // Send response
    hmodbus->state = MODBUS_STATE_SENDING;
    HAL_UART_Transmit(hmodbus->huart, hmodbus->tx_buffer, hmodbus->tx_length, 100);
    hmodbus->state = MODBUS_STATE_IDLE;
    
    // Tắt LED_MB sau khi gửi response thành công
    HAL_GPIO_WritePin(LED_MB_GPIO_Port, LED_MB_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Send Modbus exception response
 */
static void Modbus_SendException(Modbus_Handle_t *hmodbus, uint8_t function_code, uint8_t exception_code)
{
    hmodbus->tx_buffer[0] = hmodbus->slave_address;
    hmodbus->tx_buffer[1] = function_code | 0x80; // Set MSB for exception
    hmodbus->tx_buffer[2] = exception_code;
    hmodbus->tx_length = 3;
    
    Modbus_SendResponse(hmodbus);
}

/**
 * @brief Write value to Modbus register
 */
bool Modbus_WriteRegister(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t value)
{
    if (hmodbus == NULL || address >= MODBUS_REG_COUNT) {
        return false;
    }
    
    hmodbus->registers[address] = value;
    return true;
}

/**
 * @brief Read value from Modbus register
 */
bool Modbus_ReadRegister(Modbus_Handle_t *hmodbus, uint16_t address, uint16_t *value)
{
    if (hmodbus == NULL || value == NULL || address >= MODBUS_REG_COUNT) {
        return false;
    }
    
    *value = hmodbus->registers[address];
    return true;
}

/**
 * @brief Register callback for register read operations
 */
void Modbus_RegisterReadCallback(Modbus_Handle_t *hmodbus, 
                                  void (*callback)(uint16_t addr, uint16_t *value))
{
    if (hmodbus != NULL) {
        hmodbus->reg_read_callback = callback;
    }
}

/**
 * @brief Register callback for register write operations
 */
void Modbus_RegisterWriteCallback(Modbus_Handle_t *hmodbus, 
                                   void (*callback)(uint16_t addr, uint16_t value))
{
    if (hmodbus != NULL) {
        hmodbus->reg_write_callback = callback;
    }
}

/**
 * @brief Get Modbus statistics
 */
void Modbus_GetStats(Modbus_Handle_t *hmodbus, uint32_t *requests, uint32_t *errors)
{
    if (hmodbus != NULL) {
        if (requests != NULL) {
            *requests = hmodbus->requests_processed;
        }
        if (errors != NULL) {
            *errors = hmodbus->errors_count;
        }
    }
}

/**
 * @brief Timer callback for frame timeout detection
 * Gọi từ HAL_TIM_PeriodElapsedCallback khi timer overflow (T3.5 timeout)
 */
void Modbus_TIM_TimeoutCallback(Modbus_Handle_t *hmodbus)
{
    if (hmodbus == NULL) {
        return;
    }
    
    // Stop timer
    HAL_TIM_Base_Stop_IT(hmodbus->htim);
    
    // Nếu đang nhận dữ liệu và có ít nhất 1 byte
    if (hmodbus->state == MODBUS_STATE_RECEIVING && hmodbus->rx_index > 0) {
        // Timeout xảy ra -> xử lý frame
        hmodbus->state = MODBUS_STATE_PROCESSING;
        Modbus_ProcessFrame(hmodbus);
        
        // Reset buffer
        hmodbus->rx_index = 0;
        hmodbus->state = MODBUS_STATE_IDLE;
    } else {
        // Timeout mà không có dữ liệu hoặc đang idle
        hmodbus->rx_index = 0;
        hmodbus->state = MODBUS_STATE_IDLE;
    }
}
