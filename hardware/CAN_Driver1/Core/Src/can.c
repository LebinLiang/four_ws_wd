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
uint8_t Rx_Data[8];
uint8_t state=0;
extern uint8_t CAN_check_count ;
extern ms_ctrl_t ms1;
extern ms_ctrl_t ms2;
extern void Can_unpack_msg(uint8_t* buf,ms_ctrl_t *ms);

  void CAN1_Config(void)
{
	CAN_FilterTypeDef CAN_FilterType;
	CAN_FilterType.FilterBank=0;
	CAN_FilterType.FilterIdHigh=(CAN_RX_ID1 << 5);
	CAN_FilterType.FilterIdLow=0x0000;
	CAN_FilterType.FilterMaskIdHigh=0x0000;
	CAN_FilterType.FilterMaskIdLow=0x0000;
	CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN_FilterType.FilterMode=CAN_FILTERMODE_IDLIST;
	CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterType.FilterActivation=ENABLE;
	CAN_FilterType.SlaveStartFilterBank=14;
	
	CAN_FilterTypeDef CAN_FilterType2;
	CAN_FilterType2.FilterBank=1;
	CAN_FilterType2.FilterIdHigh=(CAN_RX_ID2 << 5);
	CAN_FilterType2.FilterIdLow=0x0000;
	CAN_FilterType2.FilterMaskIdHigh=0x0000;
	CAN_FilterType2.FilterMaskIdLow=0x0000;
	CAN_FilterType2.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN_FilterType2.FilterMode=CAN_FILTERMODE_IDLIST;
	CAN_FilterType2.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterType2.FilterActivation=ENABLE;
	CAN_FilterType2.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan,&CAN_FilterType2)!=HAL_OK)
	{
		Error_Handler();
	}
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
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
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
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
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
			
				CAN_check_count++;
				if(ID==CAN_RX_ID1)
				{	
					Can_unpack_msg(Rx_Data,&ms1);
				}
				if(ID==CAN_RX_ID2)
				{	
					Can_unpack_msg(Rx_Data,&ms2);
				}
    		
    	}
    }


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
