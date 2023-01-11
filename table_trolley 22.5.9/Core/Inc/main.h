/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DelayXms	HAL_Delay
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Dir_rightA_Pin GPIO_PIN_4
#define Dir_rightA_GPIO_Port GPIOA
#define Dir_rightB_Pin GPIO_PIN_5
#define Dir_rightB_GPIO_Port GPIOA
#define Dir_leftA_Pin GPIO_PIN_12
#define Dir_leftA_GPIO_Port GPIOB
#define Dir_leftB_Pin GPIO_PIN_13
#define Dir_leftB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LED_OFF       	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET)
#define LED_ON       HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET)

#define DirLA_ON			HAL_GPIO_WritePin(Dir_leftA_GPIO_Port,Dir_leftA_Pin,GPIO_PIN_SET)
#define DirLA_OFF			HAL_GPIO_WritePin(Dir_leftA_GPIO_Port,Dir_leftA_Pin,GPIO_PIN_RESET)

#define DirLB_ON			HAL_GPIO_WritePin(Dir_leftB_GPIO_Port,Dir_leftB_Pin,GPIO_PIN_SET)
#define DirLB_OFF			HAL_GPIO_WritePin(Dir_leftB_GPIO_Port,Dir_leftB_Pin,GPIO_PIN_RESET)

#define DirRA_ON   		HAL_GPIO_WritePin(Dir_rightA_GPIO_Port,Dir_rightA_Pin,GPIO_PIN_SET)
#define DirRA_OFF   	HAL_GPIO_WritePin(Dir_rightA_GPIO_Port,Dir_rightA_Pin,GPIO_PIN_RESET)

#define DirRB_ON   		HAL_GPIO_WritePin(Dir_rightB_GPIO_Port,Dir_rightB_Pin,GPIO_PIN_SET)
#define DirRB_OFF   	HAL_GPIO_WritePin(Dir_rightB_GPIO_Port,Dir_rightB_Pin,GPIO_PIN_RESET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
