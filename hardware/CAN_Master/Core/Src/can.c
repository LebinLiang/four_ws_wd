/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
extern chassic_odom_t Chassic_Odom;
uint8_t Rx_Data[8];
uint8_t state=0;


  void CAN1_Config(void)
{
	CAN_FilterTypeDef CAN_FilterType;
	CAN_FilterType.FilterBank=0;
	CAN_FilterType.FilterIdHigh=0x0000;
	CAN_FilterType.FilterIdLow=0x0000;
	CAN_FilterType.FilterMaskIdHigh=0x0000;
	CAN_FilterType.FilterMaskIdLow=0x0000;
	CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterType.FilterActivation=ENABLE;
	CAN_FilterType.SlaveStartFilterBank=14;
	if(HAL_CAN_ConfigFilter(&hcan,&CAN_FilterType)!=HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan)!=HAL_OK)
	{
		Error_Handler();
	}
}


/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
  /* USER CODE BEGIN CAN1:USB_HP_CAN1_TX_IRQn disable */
    /**
    * Uncomment the line below to disable the "USB_HP_CAN1_TX_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn); */
  /* USER CODE END CAN1:USB_HP_CAN1_TX_IRQn disable */

  /* USER CODE BEGIN CAN1:USB_LP_CAN1_RX0_IRQn disable */
    /**
    * Uncomment the line below to disable the "USB_LP_CAN1_RX0_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn); */
  /* USER CODE END CAN1:USB_LP_CAN1_RX0_IRQn disable */

    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
    	CAN_RxHeaderTypeDef CAN_RxHeader;
    	HAL_StatusTypeDef HAL_Retval;
    	
    	//uint8_t Data_Len=0;
    	uint32_t ID=0;
    
    	HAL_Retval = HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN_RxHeader,Rx_Data);
    	if(HAL_Retval == HAL_OK)
    	{
    		//Data_Len = CAN_RxHeader.DLC;
    		if(CAN_RxHeader.IDE)
    			ID = CAN_RxHeader.ExtId;
    		else
    			ID = CAN_RxHeader.StdId;
						if (state==0)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				state = 1;
				}
				else
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					state = 0;
				}
				
				if(ID==CAN_TX_ID1)
				{	
					unsigned char *p1 = (unsigned char*)&Chassic_Odom.S1_angle;
						for(uint8_t i=0 ;i<4;i++)
					{
						p1[i] = Rx_Data[i];
					}
					unsigned char *p2 = (unsigned char*)&Chassic_Odom.S2_angle;
						for(uint8_t i=0 ;i<4;i++)
					{
						p2[i] = Rx_Data[i+4];
					}	
				}
				else if(ID==CAN_TX_ID2)
				{	
				 	unsigned char *p3 = (unsigned char*)&Chassic_Odom.S3_angle;
						for(uint8_t i=0 ;i<4;i++)
					{
						p3[i] = Rx_Data[i];
					}
					unsigned char *p4 = (unsigned char*)&Chassic_Odom.S4_angle;
						for(uint8_t i=0 ;i<4;i++)
					{
						p4[i] = Rx_Data[i+4];
					}	
				}
    	}
    }


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
