/*
 * bno055.c
 *
 *  Created on: Sep 22, 2025
 *      Author: tiensy
 */

 #include "bno055.h"
 #include <string.h>
 
 #define BNO055_TIMEOUT_MS       1000
 #define BNO055_MODE_SWITCH_DELAY 30
 
 /**
  * @brief Initialize BNO055 sensor
  */
 BNO055_Status_t BNO055_Init(BNO055_Handle_t *hbno055, I2C_HandleTypeDef *hi2c)
 {
     if (!hbno055 || !hi2c) return BNO055_STATUS_ERROR;
     
     // Initialize handle
     hbno055->hi2c = hi2c;
     hbno055->address = BNO055_I2C_ADDR;
     hbno055->initialized = false;
     hbno055->status = BNO055_STATUS_OK;
     hbno055->error_code = 0;
     hbno055->operation_mode = BNO055_OPERATION_MODE_CONFIG;
     
     // Clear sensor data
     memset(&hbno055->accel, 0, sizeof(BNO055_Vector_t));
     memset(&hbno055->gyro, 0, sizeof(BNO055_Vector_t));
     memset(&hbno055->mag, 0, sizeof(BNO055_Vector_t));
     memset(&hbno055->euler, 0, sizeof(BNO055_Euler_t));
     memset(&hbno055->quaternion, 0, sizeof(BNO055_Quaternion_t));
     memset(&hbno055->linear_accel, 0, sizeof(BNO055_Vector_t));
     memset(&hbno055->calib_status, 0, sizeof(BNO055_CalibStatus_t));
     
     // Check communication with retry
     uint8_t chip_id;
     uint8_t retry_count = 3;
     BNO055_Status_t id_status = BNO055_STATUS_ERROR;
     
     while (retry_count > 0 && id_status != BNO055_STATUS_OK) {
         id_status = BNO055_ReadID(hbno055, &chip_id);
         if (id_status != BNO055_STATUS_OK) {
             HAL_Delay(50);
             retry_count--;
         } else {
             // Successfully read chip ID, break early
             break;
         }
     }
     
     if (id_status != BNO055_STATUS_OK) {
         hbno055->status = BNO055_STATUS_COMM_ERROR;
         hbno055->error_code = 0xFF; // Communication failed
         return BNO055_STATUS_COMM_ERROR;
     }
     
     // Debug: show what chip ID we actually read
     if (chip_id != BNO055_ID) {
         hbno055->status = BNO055_STATUS_ERROR;
         hbno055->error_code = chip_id; // Store actual chip ID for debug
         return BNO055_STATUS_ERROR;
     }
     
     // Set to config mode
     if (BNO055_SetOperationMode(hbno055, BNO055_OPERATION_MODE_CONFIG) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     // Reset system
     if (BNO055_WriteRegister(hbno055, BNO055_SYS_TRIGGER_ADDR, 0x20) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     // Wait for reset
     HAL_Delay(650);
     
     // Wait for chip to be ready
     uint8_t sys_stat, sys_err;
     uint32_t timeout = HAL_GetTick() + 1000;
     do {
         if (BNO055_ReadSystemStatus(hbno055, &sys_stat, &sys_err) != BNO055_STATUS_OK) {
             return BNO055_STATUS_ERROR;
         }
         HAL_Delay(10);
     } while (sys_stat != 0x01 && HAL_GetTick() < timeout);
     
     if (sys_stat != 0x01) {
         hbno055->status = BNO055_STATUS_TIMEOUT;
         return BNO055_STATUS_TIMEOUT;
     }
     
     // Set normal power mode
     if (BNO055_SetPowerMode(hbno055, BNO055_POWER_MODE_NORMAL) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     // Set units (m/s², °/s, °C, degrees)
     if (BNO055_WriteRegister(hbno055, BNO055_UNIT_SEL_ADDR, 0x00) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     // Set to NDOF mode (9DOF fusion)
     if (BNO055_SetOperationMode(hbno055, BNO055_OPERATION_MODE_NDOF) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->initialized = true;
     hbno055->status = BNO055_STATUS_OK;
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Set operation mode
  */
 BNO055_Status_t BNO055_SetOperationMode(BNO055_Handle_t *hbno055, uint8_t mode)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     if (BNO055_WriteRegister(hbno055, BNO055_OPR_MODE_ADDR, mode) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->operation_mode = mode;
     HAL_Delay(BNO055_MODE_SWITCH_DELAY);
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Set power mode
  */
 BNO055_Status_t BNO055_SetPowerMode(BNO055_Handle_t *hbno055, uint8_t mode)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     return BNO055_WriteRegister(hbno055, BNO055_PWR_MODE_ADDR, mode);
 }
 
 /**
  * @brief Read chip ID
  */
 BNO055_Status_t BNO055_ReadID(BNO055_Handle_t *hbno055, uint8_t *id)
 {
     if (!hbno055 || !id) return BNO055_STATUS_ERROR;
     
     return BNO055_ReadRegister(hbno055, BNO055_CHIP_ID_ADDR, id);
 }
 
 /**
  * @brief Read revision information
  */
 BNO055_Status_t BNO055_ReadRevInfo(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[6];
     if (BNO055_ReadRegisters(hbno055, BNO055_ACCEL_REV_ID_ADDR, data, 6) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read accelerometer data
  */
 BNO055_Status_t BNO055_ReadAccel(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[6];
     if (BNO055_ReadRegisters(hbno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 6) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->accel.x = (int16_t)((data[1] << 8) | data[0]);
     hbno055->accel.y = (int16_t)((data[3] << 8) | data[2]);
     hbno055->accel.z = (int16_t)((data[5] << 8) | data[4]);
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read gyroscope data
  */
 BNO055_Status_t BNO055_ReadGyro(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[6];
     if (BNO055_ReadRegisters(hbno055, BNO055_GYRO_DATA_X_LSB_ADDR, data, 6) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->gyro.x = (int16_t)((data[1] << 8) | data[0]);
     hbno055->gyro.y = (int16_t)((data[3] << 8) | data[2]);
     hbno055->gyro.z = (int16_t)((data[5] << 8) | data[4]);
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read magnetometer data
  */
 BNO055_Status_t BNO055_ReadMag(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[6];
     if (BNO055_ReadRegisters(hbno055, BNO055_MAG_DATA_X_LSB_ADDR, data, 6) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->mag.x = (int16_t)((data[1] << 8) | data[0]);
     hbno055->mag.y = (int16_t)((data[3] << 8) | data[2]);
     hbno055->mag.z = (int16_t)((data[5] << 8) | data[4]);
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read Euler angles
  */
 BNO055_Status_t BNO055_ReadEuler(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[6];
     if (BNO055_ReadRegisters(hbno055, BNO055_EULER_H_LSB_ADDR, data, 6) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     int16_t heading = (int16_t)((data[1] << 8) | data[0]);
     int16_t roll = (int16_t)((data[3] << 8) | data[2]);
     int16_t pitch = (int16_t)((data[5] << 8) | data[4]);
     
     // Convert to degrees (1 degree = 16 LSB)
     hbno055->euler.heading = heading / 16.0f;
     hbno055->euler.roll = roll / 16.0f;
     hbno055->euler.pitch = pitch / 16.0f;
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read quaternion data
  */
 BNO055_Status_t BNO055_ReadQuaternion(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[8];
     if (BNO055_ReadRegisters(hbno055, BNO055_QUATERNION_DATA_W_LSB_ADDR, data, 8) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     int16_t w = (int16_t)((data[1] << 8) | data[0]);
     int16_t x = (int16_t)((data[3] << 8) | data[2]);
     int16_t y = (int16_t)((data[5] << 8) | data[4]);
     int16_t z = (int16_t)((data[7] << 8) | data[6]);
     
     // Convert to float (1 quaternion = 2^14 LSB)
     const float scale = 1.0f / (1 << 14);
     hbno055->quaternion.w = w * scale;
     hbno055->quaternion.x = x * scale;
     hbno055->quaternion.y = y * scale;
     hbno055->quaternion.z = z * scale;
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read linear acceleration data
  */
 BNO055_Status_t BNO055_ReadLinearAccel(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[6];
     if (BNO055_ReadRegisters(hbno055, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, data, 6) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->linear_accel.x = (int16_t)((data[1] << 8) | data[0]);
     hbno055->linear_accel.y = (int16_t)((data[3] << 8) | data[2]);
     hbno055->linear_accel.z = (int16_t)((data[5] << 8) | data[4]);
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read all sensor data
  */
 BNO055_Status_t BNO055_ReadAllSensors(BNO055_Handle_t *hbno055)
 {
     if (!hbno055 || !hbno055->initialized) return BNO055_STATUS_NOT_INITIALIZED;
     
     BNO055_Status_t status = BNO055_STATUS_OK;
     
     // Read all sensor data
     if (BNO055_ReadAccel(hbno055) != BNO055_STATUS_OK) status = BNO055_STATUS_ERROR;
     if (BNO055_ReadGyro(hbno055) != BNO055_STATUS_OK) status = BNO055_STATUS_ERROR;
     if (BNO055_ReadMag(hbno055) != BNO055_STATUS_OK) status = BNO055_STATUS_ERROR;
     if (BNO055_ReadEuler(hbno055) != BNO055_STATUS_OK) status = BNO055_STATUS_ERROR;
     if (BNO055_ReadQuaternion(hbno055) != BNO055_STATUS_OK) status = BNO055_STATUS_ERROR;
     if (BNO055_ReadLinearAccel(hbno055) != BNO055_STATUS_OK) status = BNO055_STATUS_ERROR;
     
     // Read calibration status
     BNO055_ReadCalibStatus(hbno055);
     
     // Read temperature
     BNO055_ReadTemperature(hbno055);
     
     hbno055->last_read_time = HAL_GetTick();
     hbno055->status = status;
     
     return status;
 }
 
 /**
  * @brief Read calibration status
  */
 BNO055_Status_t BNO055_ReadCalibStatus(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t calib_data;
     if (BNO055_ReadRegister(hbno055, BNO055_CALIB_STAT_ADDR, &calib_data) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->calib_status.system = (calib_data >> 6) & 0x03;
     hbno055->calib_status.gyro = (calib_data >> 4) & 0x03;
     hbno055->calib_status.accel = (calib_data >> 2) & 0x03;
     hbno055->calib_status.mag = calib_data & 0x03;
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read temperature
  */
 BNO055_Status_t BNO055_ReadTemperature(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t temp_data;
     if (BNO055_ReadRegister(hbno055, BNO055_TEMP_ADDR, &temp_data) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     hbno055->temperature = (int8_t)temp_data;
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read system status
  */
 BNO055_Status_t BNO055_ReadSystemStatus(BNO055_Handle_t *hbno055, uint8_t *sys_stat, uint8_t *sys_err)
 {
     if (!hbno055 || !sys_stat || !sys_err) return BNO055_STATUS_ERROR;
     
     if (BNO055_ReadRegister(hbno055, BNO055_SYS_STAT_ADDR, sys_stat) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     if (BNO055_ReadRegister(hbno055, BNO055_SYS_ERR_ADDR, sys_err) != BNO055_STATUS_OK) {
         return BNO055_STATUS_ERROR;
     }
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Check if sensor is calibrated
  */
 bool BNO055_IsCalibrated(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return false;
     
     return (hbno055->calib_status.system >= 3 && 
             hbno055->calib_status.gyro >= 3 && 
             hbno055->calib_status.accel >= 3 && 
             hbno055->calib_status.mag >= 3);
 }
 
 // Data access functions
 BNO055_Vector_t* BNO055_GetAccel(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->accel;
 }
 
 BNO055_Vector_t* BNO055_GetGyro(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->gyro;
 }
 
 BNO055_Vector_t* BNO055_GetMag(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->mag;
 }
 
 BNO055_Euler_t* BNO055_GetEuler(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->euler;
 }
 
 BNO055_Quaternion_t* BNO055_GetQuaternion(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->quaternion;
 }
 
 BNO055_Vector_t* BNO055_GetLinearAccel(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->linear_accel;
 }
 
 BNO055_CalibStatus_t* BNO055_GetCalibStatus(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return NULL;
     return &hbno055->calib_status;
 }
 
 int8_t BNO055_GetTemperature(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return 0;
     return hbno055->temperature;
 }
 
 BNO055_Status_t BNO055_GetStatus(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     return hbno055->status;
 }
 
 uint8_t BNO055_GetErrorCode(BNO055_Handle_t *hbno055)
 {
     if (!hbno055) return 0xFF;
     return hbno055->error_code;
 }
 
 /**
  * @brief Write register
  */
 BNO055_Status_t BNO055_WriteRegister(BNO055_Handle_t *hbno055, uint8_t reg, uint8_t value)
 {
     if (!hbno055) return BNO055_STATUS_ERROR;
     
     uint8_t data[2] = {reg, value};
     
     if (HAL_I2C_Master_Transmit(hbno055->hi2c, hbno055->address << 1, data, 2, BNO055_TIMEOUT_MS) != HAL_OK) {
         hbno055->status = BNO055_STATUS_COMM_ERROR;
         return BNO055_STATUS_COMM_ERROR;
     }
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read single register
  */
 BNO055_Status_t BNO055_ReadRegister(BNO055_Handle_t *hbno055, uint8_t reg, uint8_t *value)
 {
     if (!hbno055 || !value) return BNO055_STATUS_ERROR;
     
     // Use combined transaction for better reliability
     HAL_StatusTypeDef result = HAL_I2C_Mem_Read(hbno055->hi2c, hbno055->address << 1, 
                                                reg, I2C_MEMADD_SIZE_8BIT, 
                                                value, 1, BNO055_TIMEOUT_MS);
     
     if (result != HAL_OK) {
         hbno055->status = BNO055_STATUS_COMM_ERROR;
         return BNO055_STATUS_COMM_ERROR;
     }
     
     return BNO055_STATUS_OK;
 }
 
 /**
  * @brief Read multiple registers
  */
 BNO055_Status_t BNO055_ReadRegisters(BNO055_Handle_t *hbno055, uint8_t reg, uint8_t *data, uint8_t len)
 {
     if (!hbno055 || !data || len == 0) return BNO055_STATUS_ERROR;
     
     // Use combined transaction for better reliability
     HAL_StatusTypeDef result = HAL_I2C_Mem_Read(hbno055->hi2c, hbno055->address << 1, 
                                                reg, I2C_MEMADD_SIZE_8BIT, 
                                                data, len, BNO055_TIMEOUT_MS);
     
     if (result != HAL_OK) {
         hbno055->status = BNO055_STATUS_COMM_ERROR;
         return BNO055_STATUS_COMM_ERROR;
     }
     
     return BNO055_STATUS_OK;
 }
 