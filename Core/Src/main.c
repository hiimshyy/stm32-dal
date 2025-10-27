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
#include <math.h>
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

TIM_HandleTypeDef htim2;

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
PN532_Config pn532_config;
BNO055_Handle_t hbno055;
Modbus_Handle_t hmodbus;

// Data buffers
uint16_t sensor_data[32];

// NFC/RFID data
uint8_t nfc_card_uid[7];
uint8_t nfc_card_uid_length = 0;
uint8_t nfc_card_type = 0;
bool nfc_card_present = false;
uint32_t nfc_last_card_uid = 0; // For detecting card changes

// Status variables
uint8_t system_status = 0;
uint8_t system_error = 0;
bool sensors_initialized = false;

// IMU velocity and heading data
float current_velocity = 0.0f;  // m/s
float current_heading = 0.0f;   // degrees

// Debug buffer
char debug_buffer[128];

// IMU fail counter for error handling
uint32_t imu_fail_counter = 0;

BNO055_Vector_t *_g_lin_accel;
BNO055_Vector_t *_g_gyro_data;
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
#define EVENT_SYSTEM_ERROR         (1UL << 3)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartModbusTask(void *argument);
void StartSensorTask(void *argument);
void StartNfcTask(void *argument);

/* USER CODE BEGIN PFP */
void SystemInit_Modules(void);
void DebugPrint(const char* format, ...);
void I2C_Scanner(void);
void DebugDumpHex(const char* label, uint8_t* data, uint8_t len);
void Modbus_OnRegisterRead(uint16_t addr, uint16_t *value);
void Modbus_OnRegisterWrite(uint16_t addr, uint16_t value);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize modules
  SystemInit_Modules();
  
  // Initialize Modbus RTU Slave với TIM2 cho frame timeout
  if (Modbus_Init(&hmodbus, &huart2, &htim2, MODBUS_SLAVE_DEFAULT_ADDRESS) == HAL_OK) {
    DebugPrint("Modbus RTU initialized on UART2 with TIM2\r\n");
    DebugPrint("  Slave Address: %d\r\n", MODBUS_SLAVE_DEFAULT_ADDRESS);
    DebugPrint("  Baudrate: 115200\r\n");
    DebugPrint("  Frame timeout: 5ms (T3.5)\r\n");
    
    // Register callbacks
    Modbus_RegisterReadCallback(&hmodbus, Modbus_OnRegisterRead);
    Modbus_RegisterWriteCallback(&hmodbus, Modbus_OnRegisterWrite);
    
    // Initialize system registers with default values
    Modbus_WriteRegister(&hmodbus, REG_DEVICE_ID, MODBUS_SLAVE_DEFAULT_ADDRESS);
    Modbus_WriteRegister(&hmodbus, REG_MODULE_TYPE, 0x0004);
    Modbus_WriteRegister(&hmodbus, REG_FIRMWARE_VERSION, 0x0101);
    Modbus_WriteRegister(&hmodbus, REG_HARDWARE_VERSION, 0x0101);
    Modbus_WriteRegister(&hmodbus, REG_CONFIG_BAUDRATE, 5); // 115200
    Modbus_WriteRegister(&hmodbus, REG_CONFIG_PARITY, 0);   // None
    Modbus_WriteRegister(&hmodbus, REG_CONFIG_STOP_BITS, 1); // 1 stop bit
  } else {
    DebugPrint("Modbus initialization failed!\r\n");
  }
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

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of nfcTask */
  nfcTaskHandle = osThreadNew(StartNfcTask, NULL, &nfcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* creation of modbusTask */
  modbusTaskHandle = osThreadNew(StartModbusTask, NULL, &modbusTask_attributes);
  if (modbusTaskHandle == NULL) {
    DebugPrint("Failed to create modbusTask\r\n");
  }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;  // 16MHz / 16 = 1MHz (1us per tick)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;  // 5000us = 5ms (T3.5 cho baudrate 115200)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, LED_Pin|LED_MB_Pin|LED_FAULT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RL_FAULT_GPIO_Port, RL_FAULT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LED_MB_Pin LED_FAULT_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED_MB_Pin|LED_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_4_Pin IN_3_Pin IN_2_Pin IN_1_Pin */
  GPIO_InitStruct.Pin = IN_4_Pin|IN_3_Pin|IN_2_Pin|IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RL_FAULT_Pin */
  GPIO_InitStruct.Pin = RL_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RL_FAULT_GPIO_Port, &GPIO_InitStruct);

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
    
    // Scan I2C bus for devices
    I2C_Scanner();
    
    // Initialize BNO055 IMU with retry mechanism
    DebugPrint("Initializing BNO055...\r\n");
    
    BNO055_Status_t bno_status = BNO055_STATUS_ERROR;
    for (int retry = 0; retry < 3 && bno_status != BNO055_STATUS_OK; retry++) {
        if (retry > 0) {
            DebugPrint("BNO055 retry attempt %d/3\r\n", retry + 1);
            HAL_Delay(500); // Longer delay for retry
        } else {
            HAL_Delay(100); // Initial startup delay
        }
        
        bno_status = BNO055_Init(&hbno055, &hi2c1);
    }
    
    if (bno_status == BNO055_STATUS_OK) {
        DebugPrint("BNO055 initialized successfully\r\n");
        
        // Check BNO055 operational status
        uint8_t sys_stat, sys_err;
        if (BNO055_ReadSystemStatus(&hbno055, &sys_stat, &sys_err) == BNO055_STATUS_OK) {
            DebugPrint("  System Status: 0x%02X, System Error: 0x%02X\r\n", sys_stat, sys_err);
        }
        
        // Check operation mode
        DebugPrint("  Operation Mode: 0x%02X\r\n", hbno055.operation_mode);
        
        system_status |= 0x01; // IMU OK
    } else {
        uint8_t error_code = BNO055_GetErrorCode(&hbno055);
        DebugPrint("BNO055 initialization failed after 3 attempts with error: 0x%02X\r\n", error_code);
        if (error_code == 0xFF) {
            DebugPrint("  -> Communication failed - check I2C connections\r\n");
        } else if (error_code == 0x00) {
            DebugPrint("  -> Chip ID is 0x00 - sensor may be in reset or wrong address\r\n");
        } else {
            DebugPrint("  -> Wrong chip ID: 0x%02X (expected 0x%02X)\r\n", error_code, BNO055_ID);
        }
        system_error |= 0x01; // IMU Error
        HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
    }
    
    // Initialize PN532 NFC/RFID with retry mechanism
    DebugPrint("Initializing PN532...\r\n");
    
    // Configure PN532
    pn532_config.hi2c = &hi2c1;
    pn532_config.i2c_addr = PN532_I2C_ADDRESS << 1; // HAL expects 8-bit address
    pn532_config.reset_port = NULL; // No reset pin configured
    pn532_config.reset_pin = 0;
    pn532_config.log = DebugPrint; // Use our debug print function
    
    PN532_Status_t pn532_status = PN532_STATUS_ERROR;
    for (int retry = 0; retry < 3 && pn532_status != PN532_STATUS_OK; retry++) {
        if (retry > 0) {
            DebugPrint("PN532 retry attempt %d/3\r\n", retry + 1);
            HAL_Delay(500); // Longer delay for retry
        } else {
            HAL_Delay(100); // Delay between sensor initializations
        }
        
        pn532_status = PN532_Init(&pn532_config);
    }
    
    if (pn532_status == PN532_STATUS_OK) {
        DebugPrint("PN532 initialized successfully\r\n");
        system_status |= 0x02; // NFC OK
    } else {
        DebugPrint("PN532 initialization failed\n");
    }
    
    
    sensors_initialized = true;
    DebugPrint("Module initialization completed\r\n");
    DebugPrint("System Status: 0x%02X, System Error: 0x%02X\r\n", system_status, system_error);
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
 * @brief I2C Scanner to detect devices on the bus
 */
void I2C_Scanner(void)
{
    uint8_t devices_found = 0;
    DebugPrint("I2C Scanner starting...\r\n");
    DebugPrint("Expected devices: BNO055 (0x%02X), PN532 (0x%02X)\r\n", BNO055_I2C_ADDR, PN532_I2C_ADDRESS);
    DebugPrint("I2C Clock Speed: %lu Hz\r\n", hi2c1.Init.ClockSpeed);
    
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
}

void Modbus_OnRegisterRead(uint16_t addr, uint16_t *value)
{
    // Cập nhật các thanh ghi từ dữ liệu thực tế
    switch (addr) {
        // IMU Registers - lấy từ sensor_data[]
        case REG_ACCEL_X:
            *value = (uint16_t)_g_lin_accel->x;
            break;
        case REG_ACCEL_Y:
            *value = (uint16_t)_g_lin_accel->y;
            break;
        case REG_ACCEL_Z:
            *value = (uint16_t)_g_lin_accel->z;
            break;
        case REG_GYRO_X:
            *value = (uint16_t)_g_gyro_data->x;
            break;
        case REG_GYRO_Y:
            *value = (uint16_t)_g_gyro_data->y;
            break;
        case REG_GYRO_Z:
            *value = (uint16_t)_g_gyro_data->z;
            break;

        case REG_VELOCITY:
            // Velocity in m/s, scale by 100 (0.01 m/s resolution)
            // Ví dụ: 1.23 m/s = 123
            {
                int16_t velocity_scaled = (int16_t)(current_velocity * 100.0f);
                *value = (uint16_t)velocity_scaled;
            }
            break;

        case REG_HEADING:
            // Heading in degrees, scale by 10 (0.1 degree resolution)
            // Ví dụ: 123.4 degrees = 1234
            {
                int16_t heading_scaled = (int16_t)(current_heading * 10.0f);
                *value = (uint16_t)heading_scaled;
            }
            break;

        case REG_IMU_STATUS:
            *value = (system_status & 0x01) ? 1 : 0;
            break;

        case REG_IMU_ERROR:
            *value = (system_error & 0x01) ? 1 : 0;
            break;

        // Digital Input Registers
        case REG_DI_1:
            *value = HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin);
            break;
        case REG_DI_2:
            *value = HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin);
            break;
        case REG_DI_3:
            *value = HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin);
            break;
        case REG_DI_4:
            *value = HAL_GPIO_ReadPin(IN_4_GPIO_Port, IN_4_Pin);
            break;

        case REG_DI_STATUS:
            *value = (HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin) << 0) |
                     (HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin) << 1) |
                     (HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin) << 2) |
                     (HAL_GPIO_ReadPin(IN_4_GPIO_Port, IN_4_Pin) << 3);
            break;

        // PN532 NFC Registers
        case REG_PN532_DATA_LOW:
        case REG_PN532_DATA_HIGH:
            // Không sử dụng - để trống
            *value = 0;
            break;

        case REG_PN532_STATUS:
            *value = (system_status & 0x02) ? 1 : 0;
            break;

        case REG_PN532_ERROR:
            *value = (system_error & 0x02) ? 1 : 0;
            break;

        case REG_PN532_CARD_TYPE:
            *value = nfc_card_present ? nfc_card_type : 0;
            break;

        case REG_PN532_CARD_UID_HIGH:
            // 2 byte cao của UID (Big-Endian)
            // Byte 0 (MSB) | Byte 1
            if (nfc_card_present && nfc_card_uid_length >= 2) {
                *value = (nfc_card_uid[0] << 8) | nfc_card_uid[1];
            } else {
                *value = 0;
            }
            break;

        case REG_PN532_CARD_UID_LOW:
            // 2 byte thấp của UID (Big-Endian)
            // Byte 2 | Byte 3 (LSB)
            if (nfc_card_present && nfc_card_uid_length >= 4) {
                *value = (nfc_card_uid[2] << 8) | nfc_card_uid[3];
            } else {
                *value = 0;
            }
            break;

        // System Registers
        case REG_DEVICE_ID:
            *value = hmodbus.slave_address;
            break;
        case REG_CONFIG_BAUDRATE:
            switch (hmodbus.baudrate) {
                case MODBUS_BAUD_9600:   *value = 1; break;
                case MODBUS_BAUD_19200:  *value = 2; break;
                case MODBUS_BAUD_38400:  *value = 3; break;
                case MODBUS_BAUD_57600:  *value = 4; break;
                case MODBUS_BAUD_115200: *value = 5; break;
                default:                 *value = 5; break;
            }
            break;
        case REG_CONFIG_PARITY:
            *value = (uint16_t)hmodbus.parity;
            break;
        case REG_CONFIG_STOP_BITS:
            *value = (uint16_t)hmodbus.stopbits;
            break;
        case REG_MODULE_TYPE:
            *value = 0x0004; // DAL Module
            break;
        case REG_FIRMWARE_VERSION:
            *value = 0x0101; // v1.01
            break;
        case REG_HARDWARE_VERSION:
            *value = 0x0101; // v1.01
            break;
        case REG_SYSTEM_STATUS:
            *value = system_status;
            break;
        case REG_SYSTEM_ERROR:
            *value = system_error;
            break;

        default:
            // Giữ nguyên giá trị đã lưu trong thanh ghi
            break;
    }
}

/**
 * @brief Callback được gọi khi Master ghi thanh ghi
 */
void Modbus_OnRegisterWrite(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case REG_DEVICE_ID:
            if (value > 0 && value <= 247) {
                hmodbus.slave_address = (uint8_t)value;
                DebugPrint("Modbus slave address changed to: %d\r\n", value);
            }
            break;

        case REG_CONFIG_BAUDRATE:
            {
                uint32_t new_baudrate;
                switch (value) {
                    case 1: new_baudrate = MODBUS_BAUD_9600; break;
                    case 2: new_baudrate = MODBUS_BAUD_19200; break;
                    case 3: new_baudrate = MODBUS_BAUD_38400; break;
                    case 4: new_baudrate = MODBUS_BAUD_57600; break;
                    case 5: new_baudrate = MODBUS_BAUD_115200; break;
                    default: new_baudrate = MODBUS_BAUD_115200; break;
                }
                if (Modbus_SetConfig(&hmodbus, new_baudrate, hmodbus.parity, hmodbus.stopbits) == HAL_OK) {
                    DebugPrint("Modbus baudrate changed to: %lu\r\n", new_baudrate);
                }
            }
            break;

        case REG_CONFIG_PARITY:
            if (value <= 2) {
                if (Modbus_SetConfig(&hmodbus, hmodbus.baudrate, (Modbus_Parity_t)value, hmodbus.stopbits) == HAL_OK) {
                    DebugPrint("Modbus parity changed to: %d\r\n", value);
                }
            }
            break;

        case REG_CONFIG_STOP_BITS:
            // Thay đổi stop bits
            if (value == 1 || value == 2) {
                if (Modbus_SetConfig(&hmodbus, hmodbus.baudrate, hmodbus.parity, (Modbus_StopBits_t)value) == HAL_OK) {
                    DebugPrint("Modbus stop bits changed to: %d\r\n", value);
                }
            }
            break;

        case REG_RESET_ERROR_CMD:
            // Reset error flags
            if (value == 1) {
                system_error = 0;
                DebugPrint("System errors reset\r\n");
                // Tắt LED_FAULT khi reset error
                HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
            }
            break;

        default:
            // Lưu giá trị vào thanh ghi mặc định
            break;
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
  DebugPrint("------Default Task Started------\r\n");
  
  // Signal that initialization is complete
  osEventFlagsSet(systemEventsHandle, EVENT_SENSOR_DATA_READY);

  uint32_t recovery_cooldown = 0;
  
  /* Infinite loop */
  for(;;)
  {
    uint32_t events = osEventFlagsWait(systemEventsHandle, 
                                      EVENT_SYSTEM_ERROR, 
                                      osFlagsWaitAny, 
                                      1000);
    
    if (events & EVENT_SYSTEM_ERROR) {
        // Check cooldown
        if (HAL_GetTick() - recovery_cooldown < 10000) { // 10 second cooldown
            osEventFlagsClear(systemEventsHandle, EVENT_SYSTEM_ERROR);
            osDelay(1000);
            continue;
        }

        DebugPrint("System error detected, attempting recovery...\r\n");
        recovery_cooldown = HAL_GetTick();

        // Clear error flag
        osEventFlagsClear(systemEventsHandle, EVENT_SYSTEM_ERROR);

        // Try to reinitialize modules
        if (system_error & 0x01) {
            DebugPrint("Resetting IMU error flag\r\n");
            system_error &= ~0x01;
        }

        if (system_error & 0x02) {
            DebugPrint("Resetting NFC error flag\r\n");
            system_error &= ~0x02;
        }
    }
    
    // Cập nhật LED_FAULT dựa trên system_error
    if (system_error != 0) {
        // Có lỗi -> Bật LED_FAULT
        HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
    } else {
        // Không có lỗi -> Tắt LED_FAULT
        HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(500); // Slower blink
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
  DebugPrint("------Modbus Task Started------\r\n");
  
  /* Infinite loop */
  for(;;)
  {
    // Process Modbus communication
    Modbus_Process(&hmodbus);
    
    osDelay(1); // 1ms polling rate
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
  
  DebugPrint("------Sensor Task Started------\n");
  
  uint32_t consecutive_failures = 0;
  const uint32_t MAX_CONSECUTIVE_FAILURES = 5;

  /* Infinite loop */
  for(;;)
  {
    if (sensors_initialized && (system_status & 0x01)) {
      // BNO055 is available, read real data
      if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
        BNO055_Status_t read_status = BNO055_ReadAllSensors(&hbno055);
        
        if (read_status == BNO055_STATUS_OK) {
            // static states
            static float v = 0.0f;                 // vận tốc (m/s)
            static float last_a_forward = 0.0f;    // a_{k-1}
            static float a_forward_lp = 0.0f;      // low-pass state (EMA)
            static uint32_t last_tick = 0;
            static MAFilter_t accFilter;
            static uint8_t zh_counter = 0;

            MAFilter_Init(&accFilter);

//          BNO055_Vector_t *accel = BNO055_GetAccel(&hbno055);
            _g_lin_accel = BNO055_GetLinearAccel(&hbno055);
            BNO055_Quaternion_t *quat = BNO055_GetQuaternion(&hbno055);
            BNO055_Euler_t *euler = BNO055_GetEuler(&hbno055);
            _g_gyro_data = BNO055_GetGyro(&hbno055);

//          DebugPrint("========IMU Data========\r\n");
//          DebugPrint("Acceleration: %.2f, %.2f, %.2f\r\n", accel->x * 0.01f, accel->y * 0.01f, accel->z * 0.01f);
//          DebugPrint("Linear Acceleration: %.2f, %.2f, %.2f m/s*s\r\n", lin_accel->x * 0.01f, lin_accel->y * 0.01f, lin_accel->z * 0.01f);
//          DebugPrint("Quaternion: %.2f, %.2f, %.2f, %.2f\r\n", quat->w, quat->x, quat->y, quat->z);
//          DebugPrint("Euler: %.2f, %.2f, %.2f\r\n", euler->heading, euler->roll, euler->pitch);
//          DebugPrint("Gyro: %.2f, %.2f, %.2f\r\n", gyro->x * 0.01f, gyro->y * 0.01f, gyro->z * 0.01f);

          if (!_g_lin_accel || !quat || !euler || !_g_gyro_data) return;

          uint32_t now = HAL_GetTick();
          float dt = (last_tick == 0) ? SAMPLE_DT_DEFAULT : (now - last_tick) / 1000.0f;
          if (dt <= 0) dt = SAMPLE_DT_DEFAULT;
          last_tick = now;

          // quaternion -> rotation matrix
          float w = quat->w, x = quat->x, y = quat->y, z = quat->z;
          float norm = sqrtf(w*w + x*x + y*y + z*z);
          if (norm < 1e-6f) return;
          w/=norm; x/=norm; y/=norm; z/=norm;

          float R00 = 1 - 2*y*y - 2*z*z;
          float R01 = 2*x*y - 2*z*w;
          float R10 = 2*x*y + 2*z*w;
          float R11 = 1 - 2*x*x - 2*z*z;

          // scale LSB -> m/s^2 (BNO in m/s2 mode => 1 LSB = 0.01 m/s^2)
          float ax = _g_lin_accel->x * 0.01f;
          float ay = _g_lin_accel->z * 0.01f;
          // world frame (only x,y needed)
          float ax_w = R00*ax + R01*ay;
          float ay_w = R10*ax + R11*ay;

          // heading in radians
          float heading_rad = euler->heading * PI_F / 180.0f;

          // projection onto forward axis (vehicle forward = heading)
          float a_forward = ax_w * cosf(heading_rad) + ay_w * sinf(heading_rad);
          float a_forward_filtered = MAFilter_Update(&accFilter, a_forward);


          // ----- EMA low-pass for a_forward -----
          // alpha = dt / (tau + dt), tau = 1/(2*pi*fc)
          float tau = 1.0f / (2.0f * PI_F * FC_CUTOFF);
          float alpha = dt / (tau + dt);
          // initialize lp on first run
          static bool lp_init = false;
          if (!lp_init) {
              a_forward_lp = a_forward_filtered;
              lp_init = true;
          } else {
              a_forward_lp = alpha * a_forward_filtered + (1.0f - alpha) * a_forward_lp;
          }

          // ----- trapezoidal integration -----
          float v_delta = 0.5f * (a_forward_lp + last_a_forward) * dt;
          v += v_delta;
          last_a_forward = a_forward_lp;

          // ----- ZUPT: nếu gần như đứng yên (acceleration nhỏ và gyro nhỏ) -----
          // thresholds có thể tinh chỉnh
          const float ACC_THRESHOLD = 0.05f;   // m/s^2
          const float GYRO_THRESHOLD = 2.0f;   // deg/s (raw unit from BNO maybe LSB -> cần tùy)
          // Note: gyro in your code is raw LSB; you may convert to deg/s if needed.
          if (fabsf(a_forward_lp) < ACC_THRESHOLD &&
              fabsf(_g_gyro_data->x) < GYRO_THRESHOLD && fabsf(_g_gyro_data->y) < GYRO_THRESHOLD && fabsf(_g_gyro_data ->z) < GYRO_THRESHOLD) {
              // optionally require this condition persist N cycles before zeroing to avoid flicker
              zh_counter++;
              if (zh_counter >= 3) { // 3 cycles stable -> zero velocity
                  v = 0.0f;
              }
          } else {
              zh_counter = 0;
          }

          // optional: limit v to reasonable bounds (e.g., -50..+50 m/s)
          if (v > 50.0f) v = 50.0f;
          if (v < -50.0f) v = -50.0f;
          
          // Cập nhật velocity và heading vào biến global
          current_velocity = v;
          current_heading = euler->heading;

          // Debug print
//          DebugPrint("a_fwd: %.3f (lp %.3f) dt: %.3f => v: %.3f m/s | heading: %.2f\r\n",
//                    a_forward, a_forward_lp, dt, v, euler->heading);
//          DebugPrint("v: %.3f m/s | heading: %.2f\r\n", v, euler->heading);
          
          // Reset fail counter on success
          extern uint32_t imu_fail_counter; // Declare extern
          imu_fail_counter = 0;
          
          system_error &= ~0x01; // Clear IMU error
          consecutive_failures = 0;
        } else {
        	consecutive_failures++;
        	DebugPrint("IMU read failed! Status: %d, Consecutive failures: %lu\r\n", read_status, consecutive_failures);

        	// Chỉ báo lỗi hệ thống sau nhiều lần fail liên tiếp
        	if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
			system_error |= 0x01; // Set IMU error
			HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
			osEventFlagsSet(systemEventsHandle, EVENT_SYSTEM_ERROR);
			consecutive_failures = 0; // Reset counter
        	}
        }
        osMutexRelease(dataMutexHandle);
      }
    } else {
    	osDelay(100);
    }
    // Read sensor data every 50ms
    osDelay(50);
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
	DebugPrint("------NFC Task Starting------\r\n");
  // Wait for system initialization with timeout
  osEventFlagsWait(systemEventsHandle, EVENT_SENSOR_DATA_READY, osFlagsWaitAny, 10000);

  uint32_t firmware_version = getFirmwareVersion();
  if (firmware_version) {
    DebugPrint("PN532 firmware version: %d\r\n", firmware_version);
  } else {
    DebugPrint("Failed to get firmware version\r\n");
  }

  if (setPassiveActivationRetries(0xFF)) {
    DebugPrint("PN532 passive retries configured\r\n");
  } else {
    DebugPrint("Failed to configure passive retries\r\n");
  }

  // SAMConfig
  if (!SAMConfig()) {
      DebugPrint("PN532 SAMConfig failed\r\n");
  } else {
      DebugPrint("PN532 SAMConfig successful\r\n");
  }
  /* Infinite loop */
  for(;;)
  {
    if (sensors_initialized && (system_status & 0x02)) {
      // PN532 is available, try to read RFID card
      if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
        
        uint8_t uid[7];
        uint8_t uid_length = 0;
        
        bool card_detected = readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uid_length, 2000);
        
        if (card_detected && uid_length > 0) {
          // Card detected successfully
          DebugPrint("========NFC Card Detected========\r\n");
          DebugPrint("UID: ");
          for (uint8_t i = 0; i < uid_length; i++) {
            DebugPrint("%02X ", uid[i]);
          }
          DebugPrint("\r\n");
          
          // Calculate 32-bit UID for comparison (use first 4 bytes)
          uint32_t current_uid = 0;
          for (uint8_t i = 0; i < uid_length && i < 4; i++) {
            current_uid |= ((uint32_t)uid[i]) << (i * 8);
          }
          
          // Check if this is a new card
          if (current_uid != nfc_last_card_uid || !nfc_card_present) {
            DebugPrint("New card detected! UID: 0x%08lX\r\n", current_uid);
            
            // Update card information
            memcpy(nfc_card_uid, uid, uid_length);
            nfc_card_uid_length = uid_length;
            nfc_card_present = true;
            nfc_last_card_uid = current_uid;
            
            // Determine card type based on UID length and other characteristics
            if (uid_length == 4) {
              nfc_card_type = 1; // Mifare Classic 1K/4K or compatible
              DebugPrint("Card Type: Mifare Classic (4-byte UID)\r\n");
            } else if (uid_length == 7) {
              nfc_card_type = 2; // Mifare Ultralight or 7-byte UID card
              DebugPrint("Card Type: Mifare Ultralight (7-byte UID)\r\n");
            } else {
              nfc_card_type = 0; // Unknown
              DebugPrint("Card Type: Unknown (%d-byte UID)\r\n", uid_length);
            }            
            // Signal that new NFC data is available
            osEventFlagsSet(systemEventsHandle, EVENT_NFC_DATA_READY);
          }
          
          system_error &= ~0x02; // Clear NFC error
          
        } else {
          // No card detected or read error
          if (nfc_card_present) {
            DebugPrint("Card removed\r\n");
            
            // Clear card information
            nfc_card_present = false;
            nfc_card_uid_length = 0;
            nfc_card_type = 0;
            nfc_last_card_uid = 0;
            memset(nfc_card_uid, 0, sizeof(nfc_card_uid));
            
          }
          
          system_error &= ~0x02; // Clear NFC error (no card is not an error)
        }
        
        osMutexRelease(dataMutexHandle);
      }
    } else {
      // PN532 not available, provide default data
      static uint32_t pn532_debug_counter = 0;
      if (++pn532_debug_counter >= 50) { // Less frequent for this message
        DebugPrint("PN532 not available - sensors_initialized: %d, system_status: 0x%02X\r\n", 
                   sensors_initialized, system_status);
        pn532_debug_counter = 0;
      }
      
      if (osMutexAcquire(dataMutexHandle, 10) == osOK) {
        // Clear card data when PN532 is not available
        nfc_card_present = false;
        nfc_card_uid_length = 0;
        nfc_card_type = 0;
        nfc_last_card_uid = 0;
        
        
        osMutexRelease(dataMutexHandle);
      }
    }
    // Read NFC data every 500ms (faster response for card detection)
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
