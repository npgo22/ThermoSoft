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

/* Define to prevent recursive inclusion
 * -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes
 * ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types
 * ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants
 * --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro
 * ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes
 * ---------------------------------------------*/
void Error_Handler(void);
void MX_ETH_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines
 * -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

// IP link status
#define LED_LINK_UP_GPIO_Port GPIOC
#define LED_LINK_UP_Pin       GPIO_PIN_7

// Ethernet PHY Reset
#define ETH_PHY_RESET_Port GPIOA
#define ETH_PHY_RESET_Pin  GPIO_PIN_5

// MAX31856 #1
#define NDRDY1_Port  GPIOC
#define NDRDY1_Pin   GPIO_PIN_10
#define NFAULT1_Port GPIOA
#define NFAULT1_Pin  GPIO_PIN_10
#define CS1_Port     GPIOA
#define CS1_Pin      GPIO_PIN_15

// MAX31856 #2
#define NDRDY2_Port  GPIOD
#define NDRDY2_Pin   GPIO_PIN_2
#define NFAULT2_Port GPIOC
#define NFAULT2_Pin  GPIO_PIN_11
#define CS2_Port     GPIOC
#define CS2_Pin      GPIO_PIN_12

// MAX31856 #3
#define NDRDY3_Port  GPIOC
#define NDRDY3_Pin   GPIO_PIN_15
#define NFAULT3_Port GPIOC
#define NFAULT3_Pin  GPIO_PIN_13
#define CS3_Port     GPIOC
#define CS3_Pin      GPIO_PIN_14

// MAX31856 #4
#define NDRDY4_Port  GPIOC
#define NDRDY4_Pin   GPIO_PIN_3
#define NFAULT4_Port GPIOC
#define NFAULT4_Pin  GPIO_PIN_0
#define CS4_Port     GPIOC
#define CS4_Pin      GPIO_PIN_2

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
