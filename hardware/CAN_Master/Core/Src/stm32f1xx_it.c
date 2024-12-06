/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
pwm_capture_struct_t pwmin1;
pwm_capture_struct_t pwmin2;
pwm_capture_struct_t pwmin3;
pwm_capture_struct_t pwmin4;
pwm_capture_struct_t pwmin5;
pwm_capture_struct_t pwmin6;
pwm_capture_struct_t pwmin7;
pwm_capture_struct_t pwmin8;

extern remote_ctrl_t Remote_Ctrl_info;
extern uint8_t rc_state ;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Flash global interrupt.
  */
void FLASH_IRQHandler(void)
{
  /* USER CODE BEGIN FLASH_IRQn 0 */

  /* USER CODE END FLASH_IRQn 0 */
  HAL_FLASH_IRQHandler();
  /* USER CODE BEGIN FLASH_IRQn 1 */

  /* USER CODE END FLASH_IRQn 1 */
}

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles USB high priority or CAN TX interrupts.
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 0 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 1 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

	
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_11)
  {
			rc_state =0;
		
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
  }
 
  }



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*htim)
{

	UNUSED(htim);
	
	if(htim->Instance==TIM2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if(pwmin1.ChannelEdge == 0)
			{
			pwmin1.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
			pwmin1.ChannelEdge = 1;
			if(pwmin1.ChannelRisingTimeLast == 0)
			{
				pwmin1.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin1.ChannelRisingTimeNow > pwmin1.ChannelRisingTimeLast)
				{
					pwmin1.ChannelPeriod = pwmin1.ChannelRisingTimeNow - pwmin1.ChannelRisingTimeLast;
				}
				else
				{
					pwmin1.ChannelPeriod = pwmin1.ChannelRisingTimeNow + 0xffff - pwmin1.ChannelRisingTimeLast + 1;
				}
			}
			pwmin1.ChannelRisingTimeLast = pwmin1.ChannelRisingTimeNow;
			}
			else if(pwmin1.ChannelEdge == 1)
			{
			pwmin1.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
			
			if(pwmin1.ChannelFallingTime < pwmin1.ChannelRisingTimeNow)
			{
				pwmin1.ChannelHighTime = pwmin1.ChannelFallingTime + 0xffff - pwmin1.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin1.ChannelHighTime = pwmin1.ChannelFallingTime - pwmin1.ChannelRisingTimeNow;
			}
			if(pwmin1.ChannelPeriod != 0)
			{
				pwmin1.ChannelPercent = (uint8_t)(((float)pwmin1.ChannelHighTime / pwmin1.ChannelPeriod) * 1000);
			}
			pwmin1.ChannelEdge = 0;
			Remote_Ctrl_info.PWM0 = 2000 - pwmin1.ChannelHighTime;
			}
		}
		//-----------------------------------//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(pwmin2.ChannelEdge == 0)
			{
			pwmin2.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
			pwmin2.ChannelEdge = 1;
			if(pwmin2.ChannelRisingTimeLast == 0)
			{
				pwmin2.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin2.ChannelRisingTimeNow > pwmin2.ChannelRisingTimeLast)
				{
					pwmin2.ChannelPeriod = pwmin2.ChannelRisingTimeNow - pwmin2.ChannelRisingTimeLast;
				}
				else
				{
					pwmin2.ChannelPeriod = pwmin2.ChannelRisingTimeNow + 0xffff - pwmin2.ChannelRisingTimeLast + 1;
				}
			}
			pwmin2.ChannelRisingTimeLast = pwmin2.ChannelRisingTimeNow;
			}
			else if(pwmin2.ChannelEdge == 1)
			{
			pwmin2.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
			
			if(pwmin2.ChannelFallingTime < pwmin2.ChannelRisingTimeNow)
			{
				pwmin2.ChannelHighTime = pwmin2.ChannelFallingTime + 0xffff - pwmin2.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin2.ChannelHighTime = pwmin2.ChannelFallingTime - pwmin2.ChannelRisingTimeNow;
			}
			if(pwmin2.ChannelPeriod != 0)
			{
				pwmin2.ChannelPercent = (uint8_t)(((float)pwmin2.ChannelHighTime / pwmin2.ChannelPeriod) * 1000);
			}
			pwmin2.ChannelEdge = 0;
			Remote_Ctrl_info.PWM1 =2000 - pwmin2.ChannelHighTime;
			}
		}
			//-----------------------------------//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(pwmin3.ChannelEdge == 0)
			{
			pwmin3.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
			pwmin3.ChannelEdge = 1;
			if(pwmin3.ChannelRisingTimeLast == 0)
			{
				pwmin3.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin3.ChannelRisingTimeNow > pwmin3.ChannelRisingTimeLast)
				{
					pwmin3.ChannelPeriod = pwmin3.ChannelRisingTimeNow - pwmin3.ChannelRisingTimeLast;
				}
				else
				{
					pwmin3.ChannelPeriod = pwmin3.ChannelRisingTimeNow + 0xffff - pwmin3.ChannelRisingTimeLast + 1;
				}
			}
			pwmin3.ChannelRisingTimeLast = pwmin3.ChannelRisingTimeNow;
			}
			else if(pwmin3.ChannelEdge == 1)
			{
			pwmin3.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
			
			if(pwmin3.ChannelFallingTime < pwmin3.ChannelRisingTimeNow)
			{
				pwmin3.ChannelHighTime = pwmin3.ChannelFallingTime + 0xffff - pwmin3.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin3.ChannelHighTime = pwmin3.ChannelFallingTime - pwmin3.ChannelRisingTimeNow;
			}
			if(pwmin3.ChannelPeriod != 0)
			{
				pwmin3.ChannelPercent = (uint8_t)(((float)pwmin3.ChannelHighTime / pwmin3.ChannelPeriod) * 1000);
			}
			pwmin3.ChannelEdge = 0;
			Remote_Ctrl_info.PWM2 =2000 - pwmin3.ChannelHighTime;
			}
		}
			//-----------------------------------//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(pwmin4.ChannelEdge == 0)
			{
			pwmin4.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
			pwmin4.ChannelEdge = 1;
			if(pwmin4.ChannelRisingTimeLast == 0)
			{
				pwmin4.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin4.ChannelRisingTimeNow > pwmin4.ChannelRisingTimeLast)
				{
					pwmin4.ChannelPeriod = pwmin4.ChannelRisingTimeNow - pwmin4.ChannelRisingTimeLast;
				}
				else
				{
					pwmin4.ChannelPeriod = pwmin4.ChannelRisingTimeNow + 0xffff - pwmin4.ChannelRisingTimeLast + 1;
				}
			}
			pwmin4.ChannelRisingTimeLast = pwmin4.ChannelRisingTimeNow;
			}
			else if(pwmin4.ChannelEdge == 1)
			{
			pwmin4.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
			
			if(pwmin4.ChannelFallingTime < pwmin4.ChannelRisingTimeNow)
			{
				pwmin4.ChannelHighTime = pwmin4.ChannelFallingTime + 0xffff - pwmin4.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin4.ChannelHighTime = pwmin4.ChannelFallingTime - pwmin4.ChannelRisingTimeNow;
			}
			if(pwmin4.ChannelPeriod != 0)
			{
				pwmin4.ChannelPercent = (uint8_t)(((float)pwmin4.ChannelHighTime / pwmin4.ChannelPeriod) * 1000);
			}
			pwmin4.ChannelEdge = 0;
			Remote_Ctrl_info.PWM3 = 2000 - pwmin4.ChannelHighTime;
			}
		}
	}
	//-----------------------------------//
	if(htim->Instance==TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if(pwmin5.ChannelEdge == 0)
			{
			pwmin5.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
			pwmin5.ChannelEdge = 1;
			if(pwmin5.ChannelRisingTimeLast == 0)
			{
				pwmin5.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin5.ChannelRisingTimeNow > pwmin5.ChannelRisingTimeLast)
				{
					pwmin5.ChannelPeriod = pwmin5.ChannelRisingTimeNow - pwmin5.ChannelRisingTimeLast;
				}
				else
				{
					pwmin5.ChannelPeriod = pwmin5.ChannelRisingTimeNow + 0xffff - pwmin5.ChannelRisingTimeLast + 1;
				}
			}
			pwmin5.ChannelRisingTimeLast = pwmin5.ChannelRisingTimeNow;
			}
			else if(pwmin5.ChannelEdge == 1)
			{
			pwmin5.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
			
			if(pwmin5.ChannelFallingTime < pwmin5.ChannelRisingTimeNow)
			{
				pwmin5.ChannelHighTime = pwmin5.ChannelFallingTime + 0xffff - pwmin5.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin5.ChannelHighTime = pwmin5.ChannelFallingTime - pwmin5.ChannelRisingTimeNow;
			}
			if(pwmin5.ChannelPeriod != 0)
			{
				pwmin5.ChannelPercent = (uint8_t)(((float)pwmin5.ChannelHighTime / pwmin5.ChannelPeriod) * 1000);
			}
			pwmin5.ChannelEdge = 0;
			Remote_Ctrl_info.PWM4 = 2000 - pwmin5.ChannelHighTime;
			}
		}
		//-----------------------------------//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(pwmin6.ChannelEdge == 0)
			{
			pwmin6.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
			pwmin6.ChannelEdge = 1;
			if(pwmin6.ChannelRisingTimeLast == 0)
			{
				pwmin6.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin6.ChannelRisingTimeNow > pwmin6.ChannelRisingTimeLast)
				{
					pwmin6.ChannelPeriod = pwmin6.ChannelRisingTimeNow - pwmin6.ChannelRisingTimeLast;
				}
				else
				{
					pwmin6.ChannelPeriod = pwmin6.ChannelRisingTimeNow + 0xffff - pwmin6.ChannelRisingTimeLast + 1;
				}
			}
			pwmin6.ChannelRisingTimeLast = pwmin6.ChannelRisingTimeNow;
			}
			else if(pwmin6.ChannelEdge == 1)
			{
			pwmin6.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
			
			if(pwmin6.ChannelFallingTime < pwmin6.ChannelRisingTimeNow)
			{
				pwmin6.ChannelHighTime = pwmin6.ChannelFallingTime + 0xffff - pwmin6.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin6.ChannelHighTime = pwmin6.ChannelFallingTime - pwmin6.ChannelRisingTimeNow;
			}
			if(pwmin6.ChannelPeriod != 0)
			{
				pwmin6.ChannelPercent = (uint8_t)(((float)pwmin6.ChannelHighTime / pwmin6.ChannelPeriod) * 1000);
			}
			pwmin6.ChannelEdge = 0;
			
		Remote_Ctrl_info.PWM5 = 2000 - pwmin6.ChannelHighTime;
			}
		}
		//-----------------------------------//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(pwmin7.ChannelEdge == 0)
			{
			pwmin7.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
			pwmin7.ChannelEdge = 1;
			if(pwmin7.ChannelRisingTimeLast == 0)
			{
				pwmin7.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin7.ChannelRisingTimeNow > pwmin7.ChannelRisingTimeLast)
				{
					pwmin7.ChannelPeriod = pwmin7.ChannelRisingTimeNow - pwmin7.ChannelRisingTimeLast;
				}
				else
				{
					pwmin7.ChannelPeriod = pwmin7.ChannelRisingTimeNow + 0xffff - pwmin7.ChannelRisingTimeLast + 1;
				}
			}
			pwmin7.ChannelRisingTimeLast = pwmin7.ChannelRisingTimeNow;
			}
			else if(pwmin7.ChannelEdge == 1)
			{
			pwmin7.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
			
			if(pwmin7.ChannelFallingTime < pwmin7.ChannelRisingTimeNow)
			{
				pwmin7.ChannelHighTime = pwmin7.ChannelFallingTime + 0xffff - pwmin7.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin7.ChannelHighTime = pwmin7.ChannelFallingTime - pwmin7.ChannelRisingTimeNow;
			}
			if(pwmin7.ChannelPeriod != 0)
			{
				pwmin7.ChannelPercent = (uint8_t)(((float)pwmin7.ChannelHighTime / pwmin7.ChannelPeriod) * 1000);
			}
			pwmin7.ChannelEdge = 0;
			Remote_Ctrl_info.PWM6 = 2000 - pwmin7.ChannelHighTime;
			}
		}
		//-----------------------------------//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(pwmin8.ChannelEdge == 0)
			{
			pwmin8.ChannelRisingTimeNow = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);//获取上升沿时间点
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);//切换捕获极性
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
			pwmin8.ChannelEdge = 1;
			if(pwmin8.ChannelRisingTimeLast == 0)
			{
				pwmin8.ChannelPeriod = 0;
			}
			else
			{
				if(pwmin8.ChannelRisingTimeNow > pwmin8.ChannelRisingTimeLast)
				{
					pwmin8.ChannelPeriod = pwmin8.ChannelRisingTimeNow - pwmin8.ChannelRisingTimeLast;
				}
				else
				{
					pwmin8.ChannelPeriod = pwmin8.ChannelRisingTimeNow + 0xffff - pwmin8.ChannelRisingTimeLast + 1;
				}
			}
			pwmin8.ChannelRisingTimeLast = pwmin8.ChannelRisingTimeNow;
			}
			else if(pwmin8.ChannelEdge == 1)
			{
			pwmin8.ChannelFallingTime = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
			
			if(pwmin8.ChannelFallingTime < pwmin8.ChannelRisingTimeNow)
			{
				pwmin8.ChannelHighTime = pwmin8.ChannelFallingTime + 0xffff - pwmin8.ChannelRisingTimeNow + 1;
			}
			else
			{
				pwmin8.ChannelHighTime = pwmin8.ChannelFallingTime - pwmin8.ChannelRisingTimeNow;
			}
			if(pwmin8.ChannelPeriod != 0)
			{
				pwmin8.ChannelPercent = (uint8_t)(((float)pwmin8.ChannelHighTime / pwmin8.ChannelPeriod) * 1000);
			}
			pwmin8.ChannelEdge = 0;
			Remote_Ctrl_info.PWM7 = 2000 - pwmin8.ChannelHighTime;
			}
		}
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
