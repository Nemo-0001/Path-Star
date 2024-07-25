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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "uartRingBuffer.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "string.h"
#include "stdint.h"

#include "../../ECUAL/Inc/DC_MOTOR.h"
#include "../../ECUAL/Inc/HCSR04.h"
#include "../../ECUAL/Inc/Buzzer.h"
#include "../../ECUAL/Inc/NMEA.h"

#include "../../util/util.h"
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
void SysTick_CallBack(void);

void PIR_Response(void);
void Ultraonic_Response(float distance1);

void Read_Buttons(void);
char Translate_Braille(uint8_t braillePattern, bool *isNumber);
void Store_Character(void);
void Send_Sentence(void);
void Send_Braille(void);

void Check_Password(void);

void Send_GPS_Data(void);

void UART_Receiving_IT_Init(void);
void UART_SendString(UART_HandleTypeDef *huart, char *string);
void UART_SendFloat(UART_HandleTypeDef *huart, float num);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L298_ENA_Pin GPIO_PIN_1
#define L298_ENA_GPIO_Port GPIOA
#define L298_IN1_Pin GPIO_PIN_0
#define L298_IN1_GPIO_Port GPIOB
#define L298_IN2_Pin GPIO_PIN_1
#define L298_IN2_GPIO_Port GPIOB
#define But2_Pin GPIO_PIN_10
#define But2_GPIO_Port GPIOB
#define But3_Pin GPIO_PIN_11
#define But3_GPIO_Port GPIOB
#define But4_Pin GPIO_PIN_12
#define But4_GPIO_Port GPIOB
#define But5_Pin GPIO_PIN_13
#define But5_GPIO_Port GPIOB
#define But6_Pin GPIO_PIN_14
#define But6_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOB
#define Trigger1_Pin GPIO_PIN_11
#define Trigger1_GPIO_Port GPIOA
#define Store_But_Pin GPIO_PIN_4
#define Store_But_GPIO_Port GPIOB
#define Send_But_Pin GPIO_PIN_5
#define Send_But_GPIO_Port GPIOB
#define Echo1_Pin GPIO_PIN_6
#define Echo1_GPIO_Port GPIOB
#define PIR_Pin GPIO_PIN_8
#define PIR_GPIO_Port GPIOB
#define But1_Pin GPIO_PIN_9
#define But1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
