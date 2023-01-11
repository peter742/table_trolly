/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"

//����Э���
#include "onenet.h"

//�����豸
#include "esp8266.h"
//C��
#include <string.h>

#include "stdio.h"
#include "math.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define DelayXms	 osDelay
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

unsigned int MotorSpeed_left;  // �����ǰ�ٶ���ֵ���ӱ������л�ȡ
unsigned int MotorSpeed_right;  // �����ǰ�ٶ���ֵ���ӱ������л�ȡ
int MotorOutput_left;		  // ������
int MotorOutput_right;		  // ������

unsigned int LastSpeed_left;
unsigned int LastSpeed_right;

extern osSemaphoreId_t tim2_BinaryHandle;

	const char *topics[] = {"/tabletrolley/pub"};
	const char toptopic[] = {"table/trolley/sub"};
	unsigned short timeCount = 0;	//���ͼ������
	
	unsigned char *dataPtr = NULL;

uint16_t  PWM_left=1140;
uint16_t PWM_right=1000;
	

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//ʹ�ܴ���1 IDLE�ж� 
	HAL_UART_Receive_DMA(&huart2,esp8266_buf,128);//ʹ�ܽ���
	
		ESP8266_Init();					//��ʼ��ESP8266
		while(OneNet_DevLink())			//����OneNET
		DelayXms(500);
		UsartPrintf(USART_DEBUG,"connected\r\n");
		DelayXms(250);
		UsartPrintf(USART_DEBUG,"sucess\r\n");
		OneNet_Subscribe(topics, 1);

	       
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	    // TIM1_CH1(pwm)
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);	    // TIM1_CH4(pwm)
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // ����������A
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // ����������B	
  HAL_TIM_Base_Start_IT(&htim2);                // ʹ�ܶ�ʱ��2�ж�

		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1000);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,1000);
		DirLA_ON;
		DirLB_OFF;
		DirRA_OFF;
		DirRB_ON;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

//  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if(++timeCount >= 200)									//���ͼ��5s
		{
			UsartPrintf(USART_DEBUG, "OneNet_Publish\r\n");
			OneNet_Publish(toptopic,"TEST11111\r\n");
			UsartPrintf(USART_DEBUG,"connected\r\n");
			timeCount = 0;
			ESP8266_Clear();
		}
		
		dataPtr = ESP8266_GetIPD(0);
		if(dataPtr != NULL)
			OneNet_RevPro(dataPtr);
		HAL_Delay(10);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	 static unsigned char i = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (htim == (&htim2))
    {
        //1.��ȡ����ٶ�
				LastSpeed_left= (short)(__HAL_TIM_GET_COUNTER(&htim3));   
				LastSpeed_right = (short)(__HAL_TIM_GET_COUNTER(&htim4));  
        // TIM4��������õ�����壬�õ����10ms����������/18��Ϊʵ��ת�ٵ�rpm
        __HAL_TIM_SET_COUNTER(&htim4,0);  // ����������
				__HAL_TIM_SET_COUNTER(&htim3,0);  // ����������
			MotorSpeed_left=MotorSpeed_left+LastSpeed_left;
			MotorSpeed_right=LastSpeed_right+MotorSpeed_right;
			
       

//				xSemaphoreGiveFromISR( tim2_BinaryHandle, &xHigherPriorityTaskWoken ); // �����ж϶�ֵ�ź��� �ж�������ͬ��
//				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        //2.��ռ�ձȵ�����������ƺ���
//        MotorOutput=3600; // 3600��Ϊ50%��ռ�ձ�
//        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorOutput);
//        MotorOutput=MotorOutput*100/7200; 
        // ռ�ձȣ������100%���㣬���ڴ�ӡ��
        i++;
        if(i>5)
        {
          // ͨ���۲�С���������ж��Ƿ���ȷ���붨ʱ���ж�
          HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
          // ��ӡ��ʱ��4�ļ���ֵ��short��-32768����32767��
					xSemaphoreGiveFromISR( tim2_BinaryHandle, &xHigherPriorityTaskWoken ); // �����ж϶�ֵ�ź��� �ж�������ͬ��
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					UsartPrintf(USART_DEBUG,"Encoder left = %d moto left= %d \r\n",MotorSpeed_left,MotorOutput_left);
					UsartPrintf(USART_DEBUG,"Encoder right = %d moto right = %d \r\n",MotorSpeed_right,MotorOutput_right);
					UsartPrintf(USART_DEBUG,"PWM_left = %d PWM_right = %d \r\n",PWM_left,PWM_right);
					MotorSpeed_left=0;
					MotorSpeed_right=0;
          i=0;
        }
    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
