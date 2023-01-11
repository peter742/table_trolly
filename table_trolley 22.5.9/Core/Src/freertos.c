/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "usart.h"
//网络协议层
#include "onenet.h"

//网络设备
#include "esp8266.h"
//C库
#include <string.h>
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern unsigned char esp8266_buf[128];
extern unsigned short esp8266_cnt, esp8266_cntPre;
extern char *req_payload ;
extern uint8_t recv_end_flag;

extern unsigned int MotorSpeed_left;  // 电机当前速度数值，从编码器中获取
extern unsigned int MotorSpeed_right;  // 电机当前速度数值，从编码器中获取
extern int MotorOutput_left;		  // 电机输出
extern int MotorOutput_right;		  // 电机输出
	
//unsigned char *dataPtr = NULL;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern 	const char *topics[];
extern	const char toptopic[];
extern	unsigned short timeCount ;	//发送间隔变量
	
extern	unsigned char *dataPtr ;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for high_task */
osThreadId_t high_taskHandle;
const osThreadAttr_t high_task_attributes = {
  .name = "high_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for middle_task */
osThreadId_t middle_taskHandle;
const osThreadAttr_t middle_task_attributes = {
  .name = "middle_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tim2_Binary */
osSemaphoreId_t tim2_BinaryHandle;
const osSemaphoreAttr_t tim2_Binary_attributes = {
  .name = "tim2_Binary"
};
/* Definitions for usart2_Binary */
osSemaphoreId_t usart2_BinaryHandle;
const osSemaphoreAttr_t usart2_Binary_attributes = {
  .name = "usart2_Binary"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Task_high(void *argument);
void Task_middle(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the semaphores(s) */
  /* creation of tim2_Binary */
  tim2_BinaryHandle = osSemaphoreNew(1, 1, &tim2_Binary_attributes);

  /* creation of usart2_Binary */
  usart2_BinaryHandle = osSemaphoreNew(1, 1, &usart2_Binary_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of high_task */
  high_taskHandle = osThreadNew(Task_high, NULL, &high_task_attributes);

  /* creation of middle_task */
  middle_taskHandle = osThreadNew(Task_middle, NULL, &middle_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	BaseType_t xHigherPriorityTaskWoken;
  BaseType_t err;
  /* Infinite loop */
	
		for(;;)
  {
		
		if(tim2_BinaryHandle!=NULL)
        {
						err = xSemaphoreTakeFromISR( tim2_BinaryHandle, &xHigherPriorityTaskWoken );  // 中断与任务同步
						 if(err == pdTRUE)
						 {
//								UsartPrintf(USART_DEBUG,"Encoder left = %d moto left= %d \r\n",MotorSpeed_left,MotorOutput_left);
//								UsartPrintf(USART_DEBUG,"Encoder right = %d moto right = %d \r\n",MotorSpeed_right,MotorOutput_right);
								
						 }
				}		
				 osDelay(1);
		
  }
		

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Task_high */
/**
* @brief Function implementing the high_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_high */
void Task_high(void *argument)
{
  /* USER CODE BEGIN Task_high */
  /* Infinite loop */
  for(;;)
  {
		if(++timeCount >= 200)									//发送间隔5s
		{
			UsartPrintf(USART_DEBUG, "OneNet_Publish\r\n");
			OneNet_Publish(toptopic,"nice\r\n");
			UsartPrintf(USART_DEBUG,"connected\r\n");
			timeCount = 0;
			ESP8266_Clear();
		}
		
		dataPtr = ESP8266_GetIPD(0);
		if(dataPtr != NULL)
			OneNet_RevPro(dataPtr);
		osDelay(10);
  }
  /* USER CODE END Task_high */
}

/* USER CODE BEGIN Header_Task_middle */
/**
* @brief Function implementing the middle_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_middle */
void Task_middle(void *argument)
{
  /* USER CODE BEGIN Task_middle */
  /* Infinite loop */
  for(;;)
  {
//		UsartPrintf(USART_DEBUG, "Task_middle\r\n");

    osDelay(200);
  }
  /* USER CODE END Task_middle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

