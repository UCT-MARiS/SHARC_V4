/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "hal_interface.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
// Function declarations
void printmsg(char *format,...);
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define STM32_LED2_Pin GPIO_PIN_13
#define STM32_LED2_GPIO_Port GPIOC
#define BATT_SDA_Pin GPIO_PIN_0
#define BATT_SDA_GPIO_Port GPIOF
#define BATT_SCL_Pin GPIO_PIN_1
#define BATT_SCL_GPIO_Port GPIOF
#define IRID_ONOFF_Pin GPIO_PIN_1
#define IRID_ONOFF_GPIO_Port GPIOC
#define GPS_INT_Pin GPIO_PIN_3
#define GPS_INT_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define IMU_CLKIN_Pin GPIO_PIN_2
#define IMU_CLKIN_GPIO_Port GPIOA
#define GPS_PPS_Pin GPIO_PIN_3
#define GPS_PPS_GPIO_Port GPIOA
#define ENV_CS_Pin GPIO_PIN_4
#define ENV_CS_GPIO_Port GPIOA
#define ENV_SCK_Pin GPIO_PIN_5
#define ENV_SCK_GPIO_Port GPIOA
#define ENV_MISO_Pin GPIO_PIN_6
#define ENV_MISO_GPIO_Port GPIOA
#define ENV_MOSI_Pin GPIO_PIN_7
#define ENV_MOSI_GPIO_Port GPIOA
#define IRID_TX_Pin GPIO_PIN_4
#define IRID_TX_GPIO_Port GPIOC
#define IRID_RX_Pin GPIO_PIN_5
#define IRID_RX_GPIO_Port GPIOC
#define IMU2_CS_Pin GPIO_PIN_12
#define IMU2_CS_GPIO_Port GPIOB
#define IMU2_SCK_Pin GPIO_PIN_13
#define IMU2_SCK_GPIO_Port GPIOB
#define IMU2_MISO_Pin GPIO_PIN_14
#define IMU2_MISO_GPIO_Port GPIOB
#define IMU2_MOSI_Pin GPIO_PIN_15
#define IMU2_MOSI_GPIO_Port GPIOB
#define IMU2_INT1_Pin GPIO_PIN_8
#define IMU2_INT1_GPIO_Port GPIOD
#define STM32_LED3_Pin GPIO_PIN_9
#define STM32_LED3_GPIO_Port GPIOD
#define STM32_LED1_Pin GPIO_PIN_7
#define STM32_LED1_GPIO_Port GPIOC
#define EXTRA_TX_Pin GPIO_PIN_5
#define EXTRA_TX_GPIO_Port GPIOD
#define EXTRA_RX_Pin GPIO_PIN_6
#define EXTRA_RX_GPIO_Port GPIOD
#define STM32_PB_Pin GPIO_PIN_7
#define STM32_PB_GPIO_Port GPIOB
#define IMU1_SCL_Pin GPIO_PIN_8
#define IMU1_SCL_GPIO_Port GPIOB
#define IMU1_SDA_Pin GPIO_PIN_9
#define IMU1_SDA_GPIO_Port GPIOB
#define IMU1_INT1_Pin GPIO_PIN_0
#define IMU1_INT1_GPIO_Port GPIOE
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
