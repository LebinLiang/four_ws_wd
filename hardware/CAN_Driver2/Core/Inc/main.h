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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define M1_DIR_Pin GPIO_PIN_12
#define M1_DIR_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_13
#define M2_DIR_GPIO_Port GPIOB
#define S2_DIR1_Pin GPIO_PIN_14
#define S2_DIR1_GPIO_Port GPIOB
#define S2_DIR2_Pin GPIO_PIN_15
#define S2_DIR2_GPIO_Port GPIOB
#define M1_PWM1_Pin GPIO_PIN_8
#define M1_PWM1_GPIO_Port GPIOA
#define M1_PWM2_Pin GPIO_PIN_9
#define M1_PWM2_GPIO_Port GPIOA
#define M2_PWM1_Pin GPIO_PIN_10
#define M2_PWM1_GPIO_Port GPIOA
#define M2_PWM2_Pin GPIO_PIN_11
#define M2_PWM2_GPIO_Port GPIOA
#define S1_DIR1_Pin GPIO_PIN_3
#define S1_DIR1_GPIO_Port GPIOB
#define S1_DIR2_Pin GPIO_PIN_4
#define S1_DIR2_GPIO_Port GPIOB
#define S1_PWM_Pin GPIO_PIN_6
#define S1_PWM_GPIO_Port GPIOB
#define S2_PWM_Pin GPIO_PIN_7
#define S2_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define CAN_TX_ID 0x102
#define CAN_RX_ID1 0x203
#define CAN_RX_ID2 0x204

typedef struct
{
	float vx;  //电机速度
	float wz;  //舵机角度
} ms_ctrl_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
