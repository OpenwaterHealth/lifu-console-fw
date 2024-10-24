/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LDAC_Pin GPIO_PIN_14
#define LDAC_GPIO_Port GPIOC
#define V12_ENABLE_Pin GPIO_PIN_13
#define V12_ENABLE_GPIO_Port GPIOC
#define V12_PWR_G_Pin GPIO_PIN_3
#define V12_PWR_G_GPIO_Port GPIOB
#define SYS_RDY_Pin GPIO_PIN_2
#define SYS_RDY_GPIO_Port GPIOD
#define V5B_PWR_G_Pin GPIO_PIN_5
#define V5B_PWR_G_GPIO_Port GPIOB
#define SMBCLK_Pin GPIO_PIN_8
#define SMBCLK_GPIO_Port GPIOA
#define HV_SHUTDOWN_Pin GPIO_PIN_9
#define HV_SHUTDOWN_GPIO_Port GPIOC
#define SMBDAT_Pin GPIO_PIN_1
#define SMBDAT_GPIO_Port GPIOC
#define HV_ON_Pin GPIO_PIN_0
#define HV_ON_GPIO_Port GPIOC
#define HV_REG_DISABLE_Pin GPIO_PIN_8
#define HV_REG_DISABLE_GPIO_Port GPIOC
#define TEMP1_Pin GPIO_PIN_2
#define TEMP1_GPIO_Port GPIOA
#define CLR_Pin GPIO_PIN_0
#define CLR_GPIO_Port GPIOB
#define HV_P_Pin GPIO_PIN_0
#define HV_P_GPIO_Port GPIOA
#define TEMP2_Pin GPIO_PIN_3
#define TEMP2_GPIO_Port GPIOA
#define V5A_PWR_G_Pin GPIO_PIN_1
#define V5A_PWR_G_GPIO_Port GPIOB
#define PWR_G_Pin GPIO_PIN_2
#define PWR_G_GPIO_Port GPIOB
#define HB_LED_Pin GPIO_PIN_1
#define HB_LED_GPIO_Port GPIOA
#define SYNC_Pin GPIO_PIN_4
#define SYNC_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_12
#define GPIO1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// #define SEMI_HOSTING_ENABLED

extern CRC_HandleTypeDef hcrc;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
