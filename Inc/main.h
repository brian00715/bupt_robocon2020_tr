/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOE
#define DCT_Pin GPIO_PIN_6
#define DCT_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define QG_Pin GPIO_PIN_3
#define QG_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOA
#define GDDG_Pin GPIO_PIN_13
#define GDDG_GPIO_Port GPIOE
#define M_KEY3_Pin GPIO_PIN_8
#define M_KEY3_GPIO_Port GPIOB
#define M_KEY2_Pin GPIO_PIN_9
#define M_KEY2_GPIO_Port GPIOB
#define M_KEY1_Pin GPIO_PIN_0
#define M_KEY1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
typedef struct{
  uint8_t min;
  uint8_t sec;
  uint32_t m_sec;
}Clock;
extern Clock clock;
typedef struct{
  int main_flag;
  int chassis_control_flag;
  int chassis_handle_flag;
  int chassis_auto_flag;
  int chassis_laser_flag;
  int lcd_flag;
  int m2006_flag;
  int vesc_flag;
  int clock_1s_flag;
}Flag;
extern Flag flag;



void inc(void);

extern float test_value[10];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
