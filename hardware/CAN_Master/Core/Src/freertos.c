/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "math_process.h"
#include "referee.h"
#include "protocol.h"
#include "math.h"
#include "tim.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define PWM_MID_MAX 2000-148  //0x95 - 0x99
#define PWM_MID_MIN 2000-152

#define PWM_HIGH_MAX 2000-190 //0xc7 - 0xc9
#define PWM_HIGH_MIN 2000-208 

#define PWM_LOW_MAX 2000-90 //0x63 -0x66
#define PWM_LOW_MIN 2000-105

void Can_Pack_msg(float speed,float angle,uint8_t* data);
uint8_t RC_Watchdog(void);
uint8_t Can_TxMessage(uint8_t ide,uint32_t id,uint8_t len,uint8_t *data);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t data[8]={5,2,0,1,3,1,4,0}; 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
remote_ctrl_t Remote_Ctrl_info;
chassic_ctrl_t Chassic_Ctrl; 
chassic_ctrl_t CAN_Ctrl;
chassic_odom_t Chassic_Odom;
pump_ctrl_t pump_ctrl;
mode_state_e Mode_State;
uint8_t error_flag = 0;
float Vx_speed = 0;
float Vy_speed = 0;
float theth = 0;
float Sum_speed = 0;
//extern uint8_t Rx_Buf[256];
uint8_t CAN_count = 0;
uint8_t can_tx[8];
int8_t sign = 1;
uint8_t rc_state =1;
uint8_t sw_state = 1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId can_taskHandle;
osThreadId rc_taskHandle;
osThreadId uart_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void usb_fifo_init(void);
/* USER CODE END FunctionPrototypes */

void CAN_Task(void const * argument);
void RemoteControl_Task(void const * argument);
void Uart_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of can_task */
  osThreadDef(can_task, CAN_Task, osPriorityHigh, 0, 128);
  can_taskHandle = osThreadCreate(osThread(can_task), NULL);

  /* definition and creation of rc_task */
  osThreadDef(rc_task, RemoteControl_Task, osPriorityHigh, 0, 128);
  rc_taskHandle = osThreadCreate(osThread(rc_task), NULL);

  /* definition and creation of uart_task */
  osThreadDef(uart_task, Uart_Task, osPriorityHigh, 0, 128);
  uart_taskHandle = osThreadCreate(osThread(uart_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_CAN_Task */
/**
  * @brief  Function implementing the can_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CAN_Task */
void CAN_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {

		for(int i=0 ;i<4;i++)
		{
			Can_Pack_msg(CAN_Ctrl.wheel_speed[i],CAN_Ctrl.wheel_angle[i],can_tx);
			Can_TxMessage(0,CAN_RX_ID1+i,8,can_tx);
		  osDelay(1);
		}
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_RemoteControl_Task */
/**
* @brief Function implementing the rc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteControl_Task */
void RemoteControl_Task(void const * argument)
{
  /* USER CODE BEGIN RemoteControl_Task */
	
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	
  /* Infinite loop */
  for(;;)
  {
		
		
		referee_send_data(CHASSIS_ODOM_CMD_ID,&Chassic_Odom,sizeof(Chassic_Odom));
		
		if (RC_Watchdog() )
	{
			
			if(Remote_Ctrl_info.PWM6>=PWM_MID_MIN && Remote_Ctrl_info.PWM6<=PWM_MID_MAX)
			{
				Mode_State = PROTECT_MODE;
				error_flag = 0;
				rc_state =1;
			}
			if (rc_state)
			{
				 if(Remote_Ctrl_info.PWM6>=PWM_LOW_MIN && Remote_Ctrl_info.PWM6<=PWM_LOW_MAX)
	 	  	{
				   if(error_flag==0)
					 {
						 Mode_State = REMOTER_MODE;
					   sw_state =1;
				   }
						 else
					   Mode_State = PROTECT_MODE;
			  }	
			 else if(Remote_Ctrl_info.PWM6>=PWM_HIGH_MIN && Remote_Ctrl_info.PWM6<=PWM_HIGH_MAX)
			 {
				    if(error_flag==0)
						{
					    Mode_State = AUTO_MODE;
				     sw_state =1;
						}
							else
					  Mode_State = PROTECT_MODE;
			 }
		  }
			else
			{
				Mode_State = PROTECT_MODE;
			}
			}
		else
		{
			Mode_State = PROTECT_MODE;
			error_flag = 1;
		}
    switch(Mode_State)
		{
			case PROTECT_MODE: //刹车 加上 停止其他控制
			{
				CAN_Ctrl.wheel_speed[0]=0;
				CAN_Ctrl.wheel_speed[1]=0;
				CAN_Ctrl.wheel_speed[2]=0;
				CAN_Ctrl.wheel_speed[3]=0;
				CAN_Ctrl.wheel_angle[0]=0;
				CAN_Ctrl.wheel_angle[1]=0;
				CAN_Ctrl.wheel_angle[2]=0;
				CAN_Ctrl.wheel_angle[3]=0;
				CAN_Ctrl.brake=1;
				break;
			}
			
			case REMOTER_MODE:
			{
			if(Remote_Ctrl_info.PWM3>=PWM_MID_MIN && Remote_Ctrl_info.PWM3<=PWM_MID_MAX)
				{
					Vx_speed = -Map(1800,1900,-1,1,Remote_Ctrl_info.PWM2);
					Vy_speed = -Map(PWM_HIGH_MIN,PWM_LOW_MAX,-1,1,Remote_Ctrl_info.PWM0);
					if(ABS(Vy_speed)<0.08)
							Vy_speed = 0;
					if(ABS(Vx_speed)<0.08)
							Vx_speed = 0;
					theth = atan2(Vy_speed,Vx_speed)*57.1;
					if(theth>90)
					{
						theth = theth-180;
						sign = -1;
					}
					else if(theth >=0 && theth<90)
						sign = 1;
					else if(theth<-90)
					{
						theth = theth+180;
						sign = -1;
					}	
					else if (theth>-90 && theth<0)
						sign =1;
				
			Sum_speed = sqrt(pow(Vx_speed,2)+pow(Vy_speed,2));
			
		//	if(Vx_speed<0&&Vy_speed>0)
		//		sign =-1;
					
			if(Remote_Ctrl_info.PWM7>=PWM_MID_MIN && Remote_Ctrl_info.PWM7<=PWM_MID_MAX)
				{
					CAN_Ctrl.wheel_speed[0]=sign*Sum_speed;
					CAN_Ctrl.wheel_speed[1]=sign*Sum_speed;
					CAN_Ctrl.wheel_speed[2]=sign*Sum_speed;
					CAN_Ctrl.wheel_speed[3]=sign*Sum_speed;
					CAN_Ctrl.wheel_angle[0]=-1*theth;
					CAN_Ctrl.wheel_angle[1]=-1*theth;
					CAN_Ctrl.wheel_angle[2]=-1*theth;
					CAN_Ctrl.wheel_angle[3]=-1*theth;
					CAN_Ctrl.brake=0;
				}
				else
				{
					CAN_Ctrl.wheel_speed[0]=sign*Sum_speed;
					CAN_Ctrl.wheel_speed[1]=sign*Sum_speed;
					CAN_Ctrl.wheel_speed[2]=sign*Sum_speed;
					CAN_Ctrl.wheel_speed[3]=sign*Sum_speed;
					CAN_Ctrl.wheel_angle[0]=-1*theth*0.5;
					CAN_Ctrl.wheel_angle[1]=-1*theth*0.5;
					CAN_Ctrl.wheel_angle[2]=0;
					CAN_Ctrl.wheel_angle[3]=0;
					CAN_Ctrl.brake=0;
				}
		  }
			else
			{
			CAN_Ctrl.wheel_angle[0]=50.267;
			CAN_Ctrl.wheel_angle[1]=-50.267;
			CAN_Ctrl.wheel_angle[2]=50.267;
			CAN_Ctrl.wheel_angle[3]=-50.267;		
			CAN_Ctrl.brake=0;
				
				Sum_speed = -Map(PWM_HIGH_MIN,PWM_LOW_MAX,-1,1,Remote_Ctrl_info.PWM3);
				
				if(Sum_speed>0 )
				{
				CAN_Ctrl.wheel_speed[0]=ABS(Sum_speed);
				CAN_Ctrl.wheel_speed[1]=-ABS(Sum_speed);
				CAN_Ctrl.wheel_speed[2]=-ABS(Sum_speed);
				CAN_Ctrl.wheel_speed[3]=ABS(Sum_speed);
				}
				else 
				{
				CAN_Ctrl.wheel_speed[0]=-ABS(Sum_speed);
				CAN_Ctrl.wheel_speed[1]=ABS(Sum_speed);
				CAN_Ctrl.wheel_speed[2]=ABS(Sum_speed);
				CAN_Ctrl.wheel_speed[3]=-ABS(Sum_speed);				
				}
			}
			
		if(Remote_Ctrl_info.PWM4>=PWM_MID_MIN && Remote_Ctrl_info.PWM4<=PWM_MID_MAX)
				pump_ctrl.pump1_en = 0;
		else
				pump_ctrl.pump1_en = 1;
		if(Remote_Ctrl_info.PWM5>=PWM_MID_MIN && Remote_Ctrl_info.PWM5<=PWM_MID_MAX)
				pump_ctrl.pump2_en = 0;
		else
				pump_ctrl.pump2_en = 1;
		
				break;
			}
			
			case AUTO_MODE:
			{
				CAN_Ctrl.wheel_speed[0]=Chassic_Ctrl.wheel_speed[0];
				CAN_Ctrl.wheel_speed[1]=Chassic_Ctrl.wheel_speed[1];
				CAN_Ctrl.wheel_speed[2]=Chassic_Ctrl.wheel_speed[2];
				CAN_Ctrl.wheel_speed[3]=Chassic_Ctrl.wheel_speed[3];
				CAN_Ctrl.wheel_angle[0]=Chassic_Ctrl.wheel_angle[0];
				CAN_Ctrl.wheel_angle[1]=Chassic_Ctrl.wheel_angle[1];
				CAN_Ctrl.wheel_angle[2]=Chassic_Ctrl.wheel_angle[2];
				CAN_Ctrl.wheel_angle[3]=Chassic_Ctrl.wheel_angle[3];
				CAN_Ctrl.brake= Chassic_Ctrl.brake;
				
				break;
			}
			
			default:
			{}
		}
		 if(CAN_Ctrl.brake == 1)
		 {
		TIM1->CCR1 = 1000;
		 }
		 else
		 {
		  TIM1->CCR1 =1334;
		 }
			if(pump_ctrl.pump1_en == 1)
				 HAL_GPIO_WritePin(GPIOB, PUMP1_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, PUMP1_Pin, GPIO_PIN_RESET);
			if(pump_ctrl.pump2_en == 1)
				HAL_GPIO_WritePin(GPIOB, PUMP2_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, PUMP2_Pin, GPIO_PIN_RESET);
		
	
    osDelay(10);
  }
  /* USER CODE END RemoteControl_Task */
}

/* USER CODE BEGIN Header_Uart_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart_Task */
void Uart_Task(void const * argument)
{
  /* USER CODE BEGIN Uart_Task */
		usb_fifo_init();
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)Rx_Buf, 128);
  /* Infinite loop */
  for(;;)
  {
			referee_unpack_fifo_data();
		
    osDelay(5);
  }
  /* USER CODE END Uart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN 4 */

/***********************************************
函数功能：RC_Watchdog 检测RC遥控器数据是否在正常值范围内
入口参数：	
返回值：1  信道正常
       0   信道异常
************************************************/
uint8_t RC_Watchdog()
{
	uint8_t error=0;
	if (Remote_Ctrl_info.PWM0>PWM_HIGH_MIN && Remote_Ctrl_info.PWM0<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM1>PWM_HIGH_MIN && Remote_Ctrl_info.PWM1<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM2>PWM_HIGH_MIN && Remote_Ctrl_info.PWM2<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM3>PWM_HIGH_MIN && Remote_Ctrl_info.PWM3<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM4>PWM_HIGH_MIN && Remote_Ctrl_info.PWM4<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM5>PWM_HIGH_MIN && Remote_Ctrl_info.PWM5<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM6>PWM_HIGH_MIN && Remote_Ctrl_info.PWM6<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (Remote_Ctrl_info.PWM7>PWM_HIGH_MIN && Remote_Ctrl_info.PWM7<PWM_LOW_MAX)
		error +=0;
	else
		error +=1;
	if (error!=0)
		return 0;
	else
    return 1;
}


/***********************************************
函数功能：Can_Pack_Send_msg 将陀螺仪数据打包进数组并发送CAN
入口参数：			
			 speed(float)电机速度[-1,1]相对
			 angle2(float)舵机转角[-90°,90°]绝对
返回值：null
************************************************/
void Can_Pack_msg(float speed,float angle,uint8_t* data)
{
	memcpy(data,(void*)&speed,sizeof(speed));		//数据
	memcpy(data+4,(void*)&angle,sizeof(angle));		//数据
}

/***********************************************
函数功能：can发送数据
入口参数：
			ide：	0：标准帧
					1：扩展帧
			id：	帧ID
			len：	数据长度
			data：	数据
返回值：0：成功。1：失败
************************************************/

uint8_t Can_TxMessage(uint8_t ide,uint32_t id,uint8_t len,uint8_t *data)
{
	uint32_t   TxMailbox;
	CAN_TxHeaderTypeDef CAN_TxHeader;
	HAL_StatusTypeDef   HAL_RetVal; 
	uint16_t i=0;
	if(ide == 0)
	{
		CAN_TxHeader.IDE = CAN_ID_STD;	//标准帧
		CAN_TxHeader.StdId = id;
	}
	else 
	{
		CAN_TxHeader.IDE = CAN_ID_EXT;			//扩展帧
		CAN_TxHeader.ExtId = id;
	}
	CAN_TxHeader.DLC = len;
	CAN_TxHeader.RTR = CAN_RTR_DATA;//数据帧,CAN_RTR_REMOTE遥控帧
	CAN_TxHeader.TransmitGlobalTime = DISABLE;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
		i++;
		if(i>0xfffe)
			return 1;
	}
	HAL_Delay(1);
	HAL_RetVal = HAL_CAN_AddTxMessage(&hcan,&CAN_TxHeader,data,&TxMailbox);
	if(HAL_RetVal != HAL_OK)
		return 1;
	return 0;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
