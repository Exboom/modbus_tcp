/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rtc.h"
#include "stm32f4xx_ll_gpio.h"

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
#define PC14_OSC32_IN_Pin LL_GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin LL_GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define SCS_Pin LL_GPIO_PIN_6
#define SCS_GPIO_Port GPIOF
#define SCLK_Pin LL_GPIO_PIN_7
#define SCLK_GPIO_Port GPIOF
#define MISO_Pin LL_GPIO_PIN_8
#define MISO_GPIO_Port GPIOF
#define MOSI_Pin LL_GPIO_PIN_9
#define MOSI_GPIO_Port GPIOF
#define PH0_OSC_IN_Pin LL_GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin LL_GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin LL_GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define BOOT1_Pin LL_GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define TE_Pin LL_GPIO_PIN_11
#define TE_GPIO_Port GPIOD
#define RDX_Pin LL_GPIO_PIN_12
#define RDX_GPIO_Port GPIOD
#define WRX_DCX_Pin LL_GPIO_PIN_13
#define WRX_DCX_GPIO_Port GPIOD
#define STLINK_RX_Pin LL_GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define STLINK_TX_Pin LL_GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TP_INT1_Pin LL_GPIO_PIN_15
#define TP_INT1_GPIO_Port GPIOA
#define LD3_Pin LL_GPIO_PIN_13
#define LD3_GPIO_Port GPIOG
#define LD4_Pin LL_GPIO_PIN_14
#define LD4_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
