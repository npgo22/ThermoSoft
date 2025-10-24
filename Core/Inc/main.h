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

// Packet batching configuration
#define SENSOR_COUNT 4  // Number of thermocouple sensors
#define BATCH_SIZE   10 // Number of reading sets per packet

// Packed structure for individual thermocouple reading (legacy)
typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  float tc_temp;
} thermocouple_reading_t;

// Packed structure for batched sensor data packet
typedef struct __attribute__((packed)) {
  uint32_t packetTag;          // Packet identifier
  float tc1_temps[BATCH_SIZE]; // Thermocouple 1 temperature batch
  float tc2_temps[BATCH_SIZE]; // Thermocouple 2 temperature batch
  float tc3_temps[BATCH_SIZE]; // Thermocouple 3 temperature batch
  float tc4_temps[BATCH_SIZE]; // Thermocouple 4 temperature batch
  uint32_t packetTime;         // Timestamp when packet was sent
} sensor_data_packet_t;

// Packet identifier for COSMOS/receiver
#define PACKET_ID 7

// IP link status
#define LED_LINK_UP_GPIO_Port GPIOC
#define LED_LINK_UP_Pin       GPIO_PIN_7

// Data send status (as in, turns on when stuff is being sent)
#define DATA_SEND_GPIO_Port GPIOC
#define DATA_SEND_GPIO_Pin  GPIO_PIN_8

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

// MAX31856 Common Configuration

/*
Bit 0
50Hz/60Hz Noise Rejection Filter Selection
0= Selects rejection of 60Hz and its harmonics (default)
1= Selects rejection of 50Hz and its harmonics
Note: Change the notch frequency only while in the “Normally Off” mode – not in the Automatic
Conversion mode.
*/
#define MAX31856_NOISE_FILTER CR0_FILTER_OUT_60Hz

// Bits 1 and 2 are for faults.

/*
Bit 3
Cold-Junction Sensor Disable
0 = Cold-junction temperature sensor enabled (default)
1 = Cold-junction temperature sensor disabled. Data from an external temperature sensor may be
written to the cold-junction temperature register. When this bit changes from 0 to 1, the most
recent cold-junction temperature value will remain in the cold-junction temperature register until
the internal sensor is enabled or until a new value is written to the register. The overall
temperature conversion time is reduced by 25ms (typ) when this bit is set to 1.
*/
#define MAX31856_CJ_ENABLE CR0_CJ_ENABLED

/*
Bits 4 and 5 enable/disable open-circuit fault detection and select fault detection timing.
This is set by resistance of the leads and is as follows:
 Bits 5:4 | Fault Test | Input Network
=======================================
    00    | Disabled   | N/A
    01    | Enabled    | R_S < 5k
    10    | Enabled    | 40k > R_S > 5k, Time const < 2ms
    11    | Enabled    | 40k > R_S > 5k, Time const > 2ms
*/
#define MAX31856_OC_FAULT_DETECT CR0_OC_DETECT_ENABLED_TC_LESS_2ms

// Bit 6 sets oneshot mode.

/*
Bit 7
Conversion Mode
0 = Normally Off mode (default)
1 = Automatic Conversion mode. Conversions occur continuously every 100ms (nominal).
*/
#define MAX31856_CONVERSION_MODE CR0_CONV_CONTINUOUS

/*
Bits 3:0
Thermocouple Type
0000 = B Type
0001 = E Type
0010 = J Type
0011 = K Type (default)
0100 = N Type
0101 = R Type
0110 = S Type
0111 = T Type
10xx = Voltage Mode, Gain = 8. Code = 8 x 1.6 x 217 x VIN
11xx = Voltage Mode, Gain = 32. Code = 32 x 1.6 x 217 x VIN
Where Code is 19 bit signed number from TC registers and VIN is thermocouple input voltage
*/
#define MAX31856_TC_TYPE CR1_TC_TYPE_K

/*
Bits 6:4
Thermocouple Voltage Conversion Averaging Mode
000 = 1 sample (default)
001 = 2 samples averaged
010 = 4 samples averaged
011 = 8 samples averaged
1xx = 16 samples averaged
Adding samples increases the conversion time and reduces noise.
Typical conversion times:
1-shot or first conversion in Auto mode:
= tCONV + (samples -1) x 33.33mS (60Hz rejection)
= tCONV + (samples -1) x 40mS (50Hz rejection)
2 thru n conversions in Auto mode
= tCONV + (samples -1) x 16.67mS (60Hz rejection)
= tCONV + (samples -1) x 20mS (50Hz rejection)
The Thermocouple Voltage Conversion Averaging Mode settings should not be changed while
conversions are taking place.
*/
#define MAX31856_AVG_SAMPLES CR1_AVG_TC_SAMPLES_4

// UDP Configuration
#define UDP_DEST_IP_ADDR0    192
#define UDP_DEST_IP_ADDR1    168
#define UDP_DEST_IP_ADDR2    1
#define UDP_DEST_IP_ADDR3    100
#define UDP_DEST_PORT        5000
#define UDP_SEND_INTERVAL_MS 100

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
