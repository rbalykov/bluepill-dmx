/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ModemCTS_Pin GPIO_PIN_0
#define ModemCTS_GPIO_Port GPIOA
#define ModemRTS_Pin GPIO_PIN_1
#define ModemRTS_GPIO_Port GPIOA
#define ModemTX_Pin GPIO_PIN_2
#define ModemTX_GPIO_Port GPIOA
#define ModemRX_Pin GPIO_PIN_3
#define ModemRX_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOB
#define CTS_EMU_Pin GPIO_PIN_1
#define CTS_EMU_GPIO_Port GPIOB
#define RTS_EMU_Pin GPIO_PIN_2
#define RTS_EMU_GPIO_Port GPIOB
#define DMX_TX_Pin GPIO_PIN_10
#define DMX_TX_GPIO_Port GPIOB
#define DMX_RX_Pin GPIO_PIN_11
#define DMX_RX_GPIO_Port GPIOB
#define USB_ENABLE_Pin GPIO_PIN_8
#define USB_ENABLE_GPIO_Port GPIOA
#define STDOUT_TX_Pin GPIO_PIN_9
#define STDOUT_TX_GPIO_Port GPIOA
#define STDIN_RX_Pin GPIO_PIN_10
#define STDIN_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ETH_RST_Pin GPIO_PIN_15
#define ETH_RST_GPIO_Port GPIOA
#define NHOLD_Pin GPIO_PIN_3
#define NHOLD_GPIO_Port GPIOB
#define NWP_Pin GPIO_PIN_4
#define NWP_GPIO_Port GPIOB
#define SC_FLASH_Pin GPIO_PIN_5
#define SC_FLASH_GPIO_Port GPIOB
#define TTL1_Pin GPIO_PIN_6
#define TTL1_GPIO_Port GPIOB
#define TTL2_Pin GPIO_PIN_7
#define TTL2_GPIO_Port GPIOB
#define TTL3_Pin GPIO_PIN_8
#define TTL3_GPIO_Port GPIOB
#define TTL4_Pin GPIO_PIN_9
#define TTL4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
