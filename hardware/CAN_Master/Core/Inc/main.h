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
#define CAN_TX_ID1 0x101
#define CAN_TX_ID2 0x102	
#define CAN_RX_ID1 0x201
#define CAN_RX_ID2 0x202
#define CAN_RX_ID3 0x203
#define CAN_RX_ID4 0x204
		
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
	
		typedef  struct
{
  uint16_t ChannelHighTime;
  uint16_t ChannelPeriod;
  uint8_t  ChannelEdge;
  uint16_t ChannelPercent;
	uint16_t ChannelPercentTemp[3];
	uint8_t  ChannelTempCount;
	uint16_t ChannelRisingTimeLast;
	uint16_t ChannelRisingTimeNow;
	uint16_t ChannelFallingTime;
	
} pwm_capture_struct_t;
	
typedef struct
{
	float wheel_speed[4];
	float wheel_angle[4];
	uint8_t brake;
	
} chassic_ctrl_t;

typedef struct 
{
	float S1_angle;
	float S2_angle;
	float S3_angle;
	float S4_angle;
	
} chassic_odom_t;

	
typedef struct
{
	
	uint16_t PWM0; //无
	uint16_t PWM1; //无
	uint16_t PWM2; //前进后退
	uint16_t PWM3; //舵机转向
	uint16_t PWM4; //喷药开关1
	uint16_t PWM5; //喷药开关2
	uint16_t PWM6; //舵机转换  分别为前轮旋转 四轮旋转 自动挡
	uint16_t PWM7; //刹车  		保护模式
	
}remote_ctrl_t;

typedef struct
{
	
	uint8_t pump1_en;//喷药水泵1
	uint8_t pump2_en;//喷药水泵2
	
} pump_ctrl_t;

typedef enum
{
  PROTECT_MODE,   //保护模式，刹车停止控制    	     
  REMOTER_MODE,   //遥控模式，遥控器控制
	AUTO_MODE,      //自动模式，由上位机控制
	
} mode_state_e;


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
#define PUMP1_Pin GPIO_PIN_12
#define PUMP1_GPIO_Port GPIOB
#define PUMP2_Pin GPIO_PIN_13
#define PUMP2_GPIO_Port GPIOB
#define BRAKE_Pin GPIO_PIN_8
#define BRAKE_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
