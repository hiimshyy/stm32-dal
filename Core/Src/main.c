/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for modbusTask */
osThreadId_t modbusTaskHandle;
const osThreadAttr_t modbusTask_attributes = {
  .name = "modbusTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for nfcTask */
osThreadId_t nfcTaskHandle;
const osThreadAttr_t nfcTask_attributes = {
  .name = "nfcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// Module handles
PN532_Handle_t hpn532;
BNO055_Handle_t hbno055;
Modbus_Handle_t hmodbus;

// Data buffers
uint16_t sensor_data[32];

// Status variables
uint8_t system_status = 0;
uint8_t system_error = 0;
bool sensors_initialized = false;

// Debug buffer
char debug_buffer[128];

// RTOS synchronization objects
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = {
  .name = "dataMutex"
};

osSemaphoreId_t sensorReadySemHandle;
const osSemaphoreAttr_t sensorReadySem_attributes = {
  .name = "sensorReadySem"
};

osEventFlagsId_t systemEventsHandle;
const osEventFlagsAttr_t systemEvents_attributes = {
  .name = "systemEvents"
};

// Event flags
#define EVENT_SENSOR_DATA_READY    (1UL << 0)
#define EVENT_NFC_DATA_READY       (1UL << 1)
#define EVENT_MODBUS_REQUEST       (1UL << 2)
#define EVENT_SYSTEM_ERROR         (1UL << 3)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartModbusTask(void *argument);
void StartSensorTask(void *argument);
void StartNfcTask(void *argument);

/* USER CODE BEGIN PFP */
void SystemInit_Modules(void);
void UpdateSensorData(void);
void UpdateNFCData(void);
void ProcessModbusCommands(void);
void DebugPrint(const char* format, ...);
void I2C_Scanner(void);
void DebugDumpHex(const char* label, uint8_t* data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize modules
  SystemInit_Modules();
  
  DebugPrint("DAL Module Started\r\n");
  DebugPrint("Firmware Version: v1.01\r\n");
  DebugPrint("Hardware Version: v1.01\r\n");
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Create the mutex(es) */
  dataMutexHandle = osMutexNew(&dataMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Create the semaphore(s) */
  sensorReadySemHandle = osSemaphoreNew(1, 0, &sensorReadySem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of modbusTask */
  modbusTaskHandle = osThreadNew(StartModbusTask, NULL, &modbusTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of nfcTask */
  nfcTaskHandle = osThreadNew(StartNfcTask, NULL, &nfcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Create additional threads */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  nfcTaskHandle = osThreadNew(StartNfcTask, NULL, &nfcTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* Create the event flags */
  systemEventsHandle = osEventFlagsNew(&systemEvents_attributes);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // This code should not be reached when using RTOS
    // All functionality is now handled by RTOS tasks
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize all modules
 */
void SystemInit_Modules(void)
{
    // Clear status and errors at start
    system_status = 0x00;
    system_error = 0x00;
    
    // Add delay for I2C stabilization
    HAL_Delay(100);
    
    // Scan I2C bus for devices
    DebugPrint("Scanning I2C bus...\r\n");
    I2C_Scanner();
    
    // Initialize BNO055 IMU
    DebugPrint("Initializing BNO055...\r\n");
    BNO055_Status_t bno_status = BNO055_Init(&hbno055, &hi2c1);
    if (bno_status == BNO055_STATUS_OK) {
        DebugPrint("BNO055 initialized successfully\r\n");
        system_status |= 0x01; // IMU OK
    } else {
        uint8_t error_code = BNO055_GetErrorCode(&hbno055);
        DebugPrint("BNO055 initialization failed with error: 0x%02X\r\n", error_code);
        if (error_code == 0xFF) {
            DebugPrint("  -> Communication failed - check I2C connections\r\n");
        } else if (error_code == 0x00) {
            DebugPrint("  -> Chip ID is 0x00 - sensor may be in reset or wrong address\r\n");
        } else {
            DebugPrint("  -> Wrong chip ID: 0x%02X (expected 0x%02X)\r\n", error_code, BNO055_ID);
        }
        system_error |= 0x01; // IMU Error
    }
    
    // Initialize PN532 NFC/RFID
    DebugPrint("Initializing PN532...\r\n");
    PN532_Status_t pn532_status = PN532_Init(&hpn532, &hi2c1);
    if (pn532_status == PN532_STATUS_OK) {
        PN532_SetMode(&hpn532, PN532_MODE_NFC);
        DebugPrint("PN532 initialized successfully\r\n");
        system_status |= 0x02; // NFC OK
    } else {
        uint8_t error_code = PN532_GetErrorCode(&hpn532);
        DebugPrint("PN532 initialization failed with error: %d\r\n", error_code);
        if (error_code == 1) {
            DebugPrint("  -> Firmware version read failed - check PN532 wiring/power\r\n");
        } else if (error_code == 2) {
            DebugPrint("  -> SAM configuration failed\r\n");
        } else {
            DebugPrint("  -> Unknown error\r\n");
        }
        system_error |= 0x02; // NFC Error
    }
    
    // Initialize Modbus slave
    DebugPrint("Initializing Modbus slave...\r\n");
    
    // Check UART2 state first
    DebugPrint("UART2 State: %d ", huart2.gState);
    switch(huart2.gState) {
        case HAL_UART_STATE_RESET:     DebugPrint("(RESET)\r\n"); break;
        case HAL_UART_STATE_READY:     DebugPrint("(READY)\r\n"); break;
        case HAL_UART_STATE_BUSY:      DebugPrint("(BUSY)\r\n"); break;
        case HAL_UART_STATE_BUSY_TX:   DebugPrint("(BUSY_TX)\r\n"); break;
        case HAL_UART_STATE_BUSY_RX:   DebugPrint("(BUSY_RX)\r\n"); break;
        case HAL_UART_STATE_BUSY_TX_RX: DebugPrint("(BUSY_TX_RX)\r\n"); break;
        case HAL_UART_STATE_TIMEOUT:   DebugPrint("(TIMEOUT)\r\n"); break;
        case HAL_UART_STATE_ERROR:     DebugPrint("(ERROR)\r\n"); break;
        default: DebugPrint("(UNKNOWN)\r\n"); break;
    }
    
    Modbus_Status_t modbus_status = Modbus_Init(&hmodbus, &huart2, 4);
    if (modbus_status == MODBUS_STATUS_OK) {
        DebugPrint("Modbus slave initialized successfully\r\n");
        system_status |= 0x04; // Modbus OK
    } else {
        DebugPrint("Modbus slave initialization failed with status: %d\r\n", modbus_status);
        system_error |= 0x04; // Modbus Error
    }
    
    sensors_initialized = true;
    DebugPrint("Module initialization completed\r\n");
    DebugPrint("System Status: 0x%02X, System Error: 0x%02X\r\n", system_status, system_error);
}

/**
 * @brief Update sensor data from BNO055
 */
void UpdateSensorData(void)
{
    if (!sensors_initialized) return;
    
    // Read all sensor data from BNO055
    if (BNO055_ReadAllSensors(&hbno055) == BNO055_STATUS_OK) {
        BNO055_Vector_t *accel = BNO055_GetAccel(&hbno055);
        BNO055_Vector_t *gyro = BNO055_GetGyro(&hbno055);
        BNO055_Vector_t *mag = BNO055_GetMag(&hbno055);
        
        // Update Modbus registers
        Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_X, (uint16_t)accel->x);
        Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_Y, (uint16_t)accel->y);
        Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_Z, (uint16_t)accel->z);
        
        Modbus_SetRegisterValue(&hmodbus, REG_GYRO_X, (uint16_t)gyro->x);
        Modbus_SetRegisterValue(&hmodbus, REG_GYRO_Y, (uint16_t)gyro->y);
        Modbus_SetRegisterValue(&hmodbus, REG_GYRO_Z, (uint16_t)gyro->z);
        
        Modbus_SetRegisterValue(&hmodbus, REG_MAG_X, (uint16_t)mag->x);
        Modbus_SetRegisterValue(&hmodbus, REG_MAG_Y, (uint16_t)mag->y);
        Modbus_SetRegisterValue(&hmodbus, REG_MAG_Z, (uint16_t)mag->z);
        
        // Update IMU status
        BNO055_CalibStatus_t *calib = BNO055_GetCalibStatus(&hbno055);
        uint16_t imu_status = (calib->system << 6) | (calib->gyro << 4) | 
                             (calib->accel << 2) | calib->mag;
        Modbus_SetRegisterValue(&hmodbus, REG_IMU_STATUS, imu_status);
        
        // Update error status
        uint8_t error_code = BNO055_GetErrorCode(&hbno055);
        Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, error_code);
        
    } else {
        system_error |= 0x01; // IMU communication error
        Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, 0x01);
    }
}

/**
 * @brief Update NFC/RFID data from PN532
 */
void UpdateNFCData(void)
{
    if (!sensors_initialized) return;
    
    // Try to read card/tag
    PN532_Status_t status = PN532_ReadCard(&hpn532);
    
    if (status == PN532_STATUS_OK && PN532_IsCardPresent(&hpn532)) {
        // Card detected
        uint8_t *uid = PN532_GetCardUID(&hpn532);
        uint8_t card_type = PN532_GetCardType(&hpn532);
        
        // Update Modbus registers
        // Store first 4 bytes of UID as two 16-bit registers
        uint16_t uid_low = (uid[1] << 8) | uid[0];
        uint16_t uid_high = (uid[3] << 8) | uid[2];
        
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_LOW, uid_low);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_HIGH, uid_high);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_TYPE, card_type);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x01); // Card present
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0x00); // No error
        
        // Store full UID in card UID register (first 2 bytes)
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_UID, uid_low);
        
    } else if (status == PN532_STATUS_NO_CARD) {
        // No card detected
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_LOW, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_HIGH, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_TYPE, 0x00);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x00); // No card
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0x00); // No error
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_UID, 0x0000);
        
    } else {
        // Communication error
        system_error |= 0x02; // NFC communication error
        uint8_t error_code = PN532_GetErrorCode(&hpn532);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x00); // No card
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, error_code);
    }
}

/**
 * @brief Process Modbus communication
 */
void ProcessModbusCommands(void)
{
    if (!sensors_initialized) return;
    
    // Update system registers
    Modbus_SetRegisterValue(&hmodbus, REG_SYSTEM_STATUS, system_status);
    Modbus_SetRegisterValue(&hmodbus, REG_SYSTEM_ERROR, system_error);
    
    // Process Modbus communication
    Modbus_Process(&hmodbus);
    
    // Handle special commands
    uint16_t reset_cmd;
    if (Modbus_GetRegisterValue(&hmodbus, REG_RESET_ERROR_CMD, &reset_cmd) == MODBUS_STATUS_OK) {
        if (reset_cmd == 0x0001) {
            // Reset all error flags
            system_error = 0;
            Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, 0);
            Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0);
            Modbus_SetRegisterValue(&hmodbus, REG_SYSTEM_ERROR, 0);
            Modbus_SetRegisterValue(&hmodbus, REG_RESET_ERROR_CMD, 0); // Clear command
            
            DebugPrint("Error flags reset\r\n");
        }
    }
}

/**
 * @brief Debug print function
 */
void DebugPrint(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);
    
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
}

/**
 * @brief UART2 Rx Complete callback (for Modbus)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        Modbus_RxCallback(&hmodbus);
        // Signal Modbus task that data is available
        osEventFlagsSet(systemEventsHandle, EVENT_MODBUS_REQUEST);
    }
}

/**
 * @brief UART2 Tx Complete callback (for Modbus)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        Modbus_TxCallback(&hmodbus);
    }
}

/**
 * @brief UART Error callback
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        Modbus_ErrorCallback(&hmodbus);
        // Signal system error
        osEventFlagsSet(systemEventsHandle, EVENT_SYSTEM_ERROR);
    }
}

/**
 * @brief Debug hex dump function
 */
void DebugDumpHex(const char* label, uint8_t* data, uint8_t len)
{
    DebugPrint("%s: ", label);
    for (uint8_t i = 0; i < len; i++) {
        DebugPrint("%02X ", data[i]);
        if ((i + 1) % 16 == 0) DebugPrint("\r\n");
    }
    if (len % 16 != 0) DebugPrint("\r\n");
}

/**
 * @brief I2C Scanner to detect devices on the bus
 */
void I2C_Scanner(void)
{
    uint8_t devices_found = 0;
    DebugPrint("I2C Scanner starting...\r\n");
    DebugPrint("Expected devices: BNO055 (0x%02X), PN532 (0x%02X)\r\n", BNO055_I2C_ADDR, PN532_I2C_ADDRESS);
    
    // Check I2C peripheral state
    DebugPrint("I2C1 State: %d ", hi2c1.State);
    switch(hi2c1.State) {
        case HAL_I2C_STATE_RESET:   DebugPrint("(RESET)\r\n"); break;
        case HAL_I2C_STATE_READY:   DebugPrint("(READY)\r\n"); break;
        case HAL_I2C_STATE_BUSY:    DebugPrint("(BUSY)\r\n"); break;
        case HAL_I2C_STATE_BUSY_TX: DebugPrint("(BUSY_TX)\r\n"); break;
        case HAL_I2C_STATE_BUSY_RX: DebugPrint("(BUSY_RX)\r\n"); break;
        case HAL_I2C_STATE_LISTEN:  DebugPrint("(LISTEN)\r\n"); break;
        case HAL_I2C_STATE_BUSY_TX_LISTEN: DebugPrint("(BUSY_TX_LISTEN)\r\n"); break;
        case HAL_I2C_STATE_BUSY_RX_LISTEN: DebugPrint("(BUSY_RX_LISTEN)\r\n"); break;
        case HAL_I2C_STATE_ABORT:   DebugPrint("(ABORT)\r\n"); break;
        case HAL_I2C_STATE_TIMEOUT: DebugPrint("(TIMEOUT)\r\n"); break;
        case HAL_I2C_STATE_ERROR:   DebugPrint("(ERROR)\r\n"); break;
        default: DebugPrint("(UNKNOWN)\r\n"); break;
    }
    
    // Try to reset I2C if it's not in ready state
    if (hi2c1.State != HAL_I2C_STATE_READY) {
        DebugPrint("I2C1 not ready, attempting reset...\r\n");
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(10);
        MX_I2C1_Init();
        HAL_Delay(50);
        DebugPrint("I2C1 reset completed, new state: %d\r\n", hi2c1.State);
    }
    
    for (uint8_t address = 1; address < 128; address++) {
        // Try to communicate with device at this address
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, 100);
        if (result == HAL_OK) {
            DebugPrint("I2C device found at address 0x%02X\r\n", address);
            devices_found++;
            
            // Check if it's our known devices
            if (address == BNO055_I2C_ADDR) {
                DebugPrint("  -> BNO055 IMU detected\r\n");
            } else if (address == PN532_I2C_ADDRESS) {
                DebugPrint("  -> PN532 NFC/RFID detected\r\n");
            } else {
                DebugPrint("  -> Unknown device (possibly EEPROM, RTC, or other I2C device)\r\n");
            }
        } else if (result != HAL_TIMEOUT) {
            // Log other errors (not timeout which is expected for empty addresses)
            if (address == BNO055_I2C_ADDR || address == PN532_I2C_ADDRESS) {
                DebugPrint("I2C error at expected address 0x%02X: %d\r\n", address, result);
            }
        }
    }
    
    if (devices_found == 0) {
        DebugPrint("No I2C devices found! Possible issues:\r\n");
        DebugPrint("  - Check SDA/SCL connections (PB7/PB6)\r\n");
        DebugPrint("  - Check pull-up resistors (4.7kΩ)\r\n");
        DebugPrint("  - Check power supply to sensors\r\n");
        DebugPrint("  - Verify I2C clock speed (currently 100kHz)\r\n");
    } else {
        DebugPrint("I2C scan completed. Found %d device(s)\r\n", devices_found);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Initialize modules
  SystemInit_Modules();
  
  DebugPrint("DAL Module Started (RTOS)\r\n");
  DebugPrint("Firmware Version: v1.01\r\n");
  DebugPrint("Hardware Version: v1.01\r\n");
  
  // Signal that initialization is complete
  osEventFlagsSet(systemEventsHandle, EVENT_SENSOR_DATA_READY);
  
  /* Infinite loop */
  for(;;)
  {
    // Monitor system status and handle errors
    uint32_t events = osEventFlagsWait(systemEventsHandle, 
                                      EVENT_SYSTEM_ERROR, 
                                      osFlagsWaitAny, 
                                      1000);
    
    if (events & EVENT_SYSTEM_ERROR) {
      DebugPrint("System error detected, attempting recovery...\r\n");
      
      // Clear error flag
      osEventFlagsClear(systemEventsHandle, EVENT_SYSTEM_ERROR);
      
      // Attempt to reinitialize failed modules (only if not hardware issue)
      if (system_error & 0x01) {
        // Reinitialize IMU
        if (BNO055_Init(&hbno055, &hi2c1) == BNO055_STATUS_OK) {
          system_error &= ~0x01;
          system_status |= 0x01;
          DebugPrint("IMU recovery successful\r\n");
        } else {
          DebugPrint("IMU recovery failed - hardware issue\r\n");
        }
      }
      
      if (system_error & 0x02) {
        // Reinitialize NFC
        if (PN532_Init(&hpn532, &hi2c1) == PN532_STATUS_OK) {
          system_error &= ~0x02;
          system_status |= 0x02;
          DebugPrint("NFC recovery successful\r\n");
        } else {
          DebugPrint("NFC recovery failed - hardware issue\r\n");
        }
      }
      
      // If hardware issues persist, reduce recovery attempts
      if ((system_error & 0x03) == 0x03) {
        DebugPrint("Hardware sensors not available - running in Modbus-only mode\r\n");
        // Don't attempt recovery for hardware issues
        osEventFlagsClear(systemEventsHandle, EVENT_SYSTEM_ERROR);
      }
    }
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartModbusTask */
/**
* @brief Function implementing the modbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbusTask */
void StartModbusTask(void *argument)
{
  /* USER CODE BEGIN StartModbusTask */
  // Wait for system initialization
  osEventFlagsWait(systemEventsHandle, EVENT_SENSOR_DATA_READY, osFlagsWaitAny, osWaitForever);
  
  DebugPrint("Modbus Task Started\r\n");
  
  /* Infinite loop */
  for(;;)
  {
    // Wait for Modbus request or timeout
    uint32_t events = osEventFlagsWait(systemEventsHandle, 
                                      EVENT_MODBUS_REQUEST, 
                                      osFlagsWaitAny, 
                                      10); // 10ms timeout
    
    // Check if Modbus request event was received
    if (events & EVENT_MODBUS_REQUEST) {
      // Clear the event flag
      osEventFlagsClear(systemEventsHandle, EVENT_MODBUS_REQUEST);
    }
    
    // Process Modbus communication
    if (sensors_initialized) {
      // Acquire data mutex before updating system registers
      if (osMutexAcquire(dataMutexHandle, 10) == osOK) {
        // Update system registers
        Modbus_SetRegisterValue(&hmodbus, REG_SYSTEM_STATUS, system_status);
        Modbus_SetRegisterValue(&hmodbus, REG_SYSTEM_ERROR, system_error);
        
        // Process Modbus communication
        Modbus_Process(&hmodbus);
        
        // Handle special commands
        uint16_t reset_cmd;
        if (Modbus_GetRegisterValue(&hmodbus, REG_RESET_ERROR_CMD, &reset_cmd) == MODBUS_STATUS_OK) {
          if (reset_cmd == 0x0001) {
            // Reset all error flags
            system_error = 0;
            Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, 0);
            Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0);
            Modbus_SetRegisterValue(&hmodbus, REG_SYSTEM_ERROR, 0);
            Modbus_SetRegisterValue(&hmodbus, REG_RESET_ERROR_CMD, 0); // Clear command
            
            DebugPrint("Error flags reset\r\n");
          }
        }
        
        osMutexRelease(dataMutexHandle);
      }
    }
    
    osDelay(1); // Small delay to prevent busy waiting
  }
  /* USER CODE END StartModbusTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  // Wait for system initialization
  osEventFlagsWait(systemEventsHandle, EVENT_SENSOR_DATA_READY, osFlagsWaitAny, osWaitForever);
  
  DebugPrint("Sensor Task Started\r\n");
  
  /* Infinite loop */
  for(;;)
  {
    if (sensors_initialized && (system_status & 0x01)) {
      // BNO055 is available, read real data
      if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
        if (BNO055_ReadAllSensors(&hbno055) == BNO055_STATUS_OK) {
          BNO055_Vector_t *accel = BNO055_GetAccel(&hbno055);
          BNO055_Vector_t *gyro = BNO055_GetGyro(&hbno055);
          BNO055_Vector_t *mag = BNO055_GetMag(&hbno055);
          
          // Update Modbus registers
          Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_X, (uint16_t)accel->x);
          Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_Y, (uint16_t)accel->y);
          Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_Z, (uint16_t)accel->z);
          
          Modbus_SetRegisterValue(&hmodbus, REG_GYRO_X, (uint16_t)gyro->x);
          Modbus_SetRegisterValue(&hmodbus, REG_GYRO_Y, (uint16_t)gyro->y);
          Modbus_SetRegisterValue(&hmodbus, REG_GYRO_Z, (uint16_t)gyro->z);
          
          Modbus_SetRegisterValue(&hmodbus, REG_MAG_X, (uint16_t)mag->x);
          Modbus_SetRegisterValue(&hmodbus, REG_MAG_Y, (uint16_t)mag->y);
          Modbus_SetRegisterValue(&hmodbus, REG_MAG_Z, (uint16_t)mag->z);
          
          // Update IMU status
          BNO055_CalibStatus_t *calib = BNO055_GetCalibStatus(&hbno055);
          uint16_t imu_status = (calib->system << 6) | (calib->gyro << 4) | 
                               (calib->accel << 2) | calib->mag;
          Modbus_SetRegisterValue(&hmodbus, REG_IMU_STATUS, imu_status);
          
          uint8_t error_code = BNO055_GetErrorCode(&hbno055);
          Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, error_code);
          
          system_error &= ~0x01; // Clear IMU error
        } else {
          system_error |= 0x01; // Set IMU error
          Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, 0x01);
          osEventFlagsSet(systemEventsHandle, EVENT_SYSTEM_ERROR);
        }
        osMutexRelease(dataMutexHandle);
      }
    } else {
      // BNO055 not available, provide dummy/default data
      if (osMutexAcquire(dataMutexHandle, 10) == osOK) {
        Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_X, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_Y, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_ACCEL_Z, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_GYRO_X, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_GYRO_Y, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_GYRO_Z, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_MAG_X, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_MAG_Y, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_MAG_Z, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_IMU_STATUS, 0x00);
        Modbus_SetRegisterValue(&hmodbus, REG_IMU_ERROR, 0xFF); // Sensor not connected
        osMutexRelease(dataMutexHandle);
      }
    }
    
    // Read sensor data every 100ms
    osDelay(100);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartNfcTask */
/**
* @brief Function implementing the nfcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNfcTask */
void StartNfcTask(void *argument)
{
  /* USER CODE BEGIN StartNfcTask */
  // Wait for system initialization
  osEventFlagsWait(systemEventsHandle, EVENT_SENSOR_DATA_READY, osFlagsWaitAny, osWaitForever);
  
  DebugPrint("NFC Task Started\r\n");
  
  /* Infinite loop */
  for(;;)
  {
    if (sensors_initialized && (system_status & 0x02)) {
      // PN532 is available, read real data
      if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
        PN532_Status_t status = PN532_ReadCard(&hpn532);
        
        if (status == PN532_STATUS_OK && PN532_IsCardPresent(&hpn532)) {
          // Card detected
          uint8_t *uid = PN532_GetCardUID(&hpn532);
          uint8_t card_type = PN532_GetCardType(&hpn532);
          
          uint16_t uid_low = (uid[1] << 8) | uid[0];
          uint16_t uid_high = (uid[3] << 8) | uid[2];
          
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_LOW, uid_low);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_HIGH, uid_high);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_TYPE, card_type);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x01); // Card present
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0x00);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_UID, uid_low);
          
          system_error &= ~0x02; // Clear NFC error
          osEventFlagsSet(systemEventsHandle, EVENT_NFC_DATA_READY);
          
        } else if (status == PN532_STATUS_NO_CARD) {
          // No card detected
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_LOW, 0x0000);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_HIGH, 0x0000);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_TYPE, 0x00);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x00);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0x00);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_UID, 0x0000);
          
          system_error &= ~0x02; // Clear NFC error
        } else {
          // Communication error
          system_error |= 0x02;
          uint8_t error_code = PN532_GetErrorCode(&hpn532);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x00);
          Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, error_code);
          osEventFlagsSet(systemEventsHandle, EVENT_SYSTEM_ERROR);
        }
        osMutexRelease(dataMutexHandle);
      }
    } else {
      // PN532 not available, provide default data
      if (osMutexAcquire(dataMutexHandle, 10) == osOK) {
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_LOW, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_DATA_HIGH, 0x0000);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_TYPE, 0x00);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_STATUS, 0x00);
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_ERROR, 0xFF); // Sensor not connected
        Modbus_SetRegisterValue(&hmodbus, REG_PN532_CARD_UID, 0x0000);
        osMutexRelease(dataMutexHandle);
      }
    }
    
    // Read NFC data every 500ms
    osDelay(500);
  }
  /* USER CODE END StartNfcTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
