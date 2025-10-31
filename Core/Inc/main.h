/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pn532.h"
#include "bno055.h"
#include "filter.h"
#include "modbus_slave.h"
#include "modbus_regs.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PI_F 3.14159265f

static const float SAMPLE_DT_DEFAULT = 0.02f; // mặc định 50Hz (update dựa trên HAL_GetTick)
static const float FC_CUTOFF = 3.0f;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LED_MB_Pin GPIO_PIN_14
#define LED_MB_GPIO_Port GPIOC
#define LED_FAULT_Pin GPIO_PIN_15
#define LED_FAULT_GPIO_Port GPIOC
#define IN_4_Pin GPIO_PIN_4
#define IN_4_GPIO_Port GPIOA
#define IN_3_Pin GPIO_PIN_5
#define IN_3_GPIO_Port GPIOA
#define IN_2_Pin GPIO_PIN_6
#define IN_2_GPIO_Port GPIOA
#define IN_1_Pin GPIO_PIN_7
#define IN_1_GPIO_Port GPIOA
#define RL_FAULT_Pin GPIO_PIN_8
#define RL_FAULT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
typedef enum
{
	DOCK_STATUS_UNDOCKED = 0,
	DOCK_STATUS_PREPARED = 1,
	DOCK_STATUS_DOCKED = 2,
} Dock_Status_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
