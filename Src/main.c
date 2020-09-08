/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "simplelib.h"
#include "cmd_func.h"
#include "chassis.h"
#include "laser.h"
#include "configure.h"
#include "lcd.h"
#include "sensor_gpio.h"
#include "kickball.h"
#include "touchdown.h"
#include "motor_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static int time_1ms_cnt = 0;
Clock clock = {0};
void clock_exe()
{
  clock.sec += clock.m_sec / 100;
  clock.min += clock.sec / 60;
  clock.sec %= 60;
  clock.m_sec %= 100;
}
/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// flag用来决定启用哪些模块，响应模块的执行函数会扫描flag中对应位置的值，为0则不执行
Flag flag = {
    0, //main_flag
    0, //chassis_control_flag
    0, //chassis_handle_flag
    0, //chassis_auto_flag
    0, //chassis_laser_flag
    0, //lcd_flag
    0, //m2006_flag
    0, //vesc_flag
    0  //clock_1s_flag
};
float test_value[10] = {0};
int time_5ms_cnt = 0;

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  simplelib_init(&huart1, &hcan1);
  can_id_init();
  chassis_init();
  motor_init();
  laser_init();
  lcd_init();
  flag.main_flag = 1;
  flag.chassis_auto_flag = 1;  // 配置底盘运动手动/自动模式
  flag.chassis_handle_flag = 0;

  can_msg msg1;
  msg1.in[0] = 0;  // pwm模式
  msg1.in[1] = 20; // 占空比

  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    simplelib_run();
    clock_exe();          // 时钟
    // lcd_exe();         // lcd消息
    gpio_sensor_exe();    // IO口执行函数
    // m2006_exe();       // 大疆电机
    // vsec_exe();
    // kickball_exe();    // 踢球系统
    // touchdown_exe();   // 达阵装置
    // laser_exe();       // 激光
    // chassis_exe();     // 底盘，及坐标更新

    if (time_5ms_cnt == 1) // 5ms中断
    {
      time_5ms_cnt = 0;
      can_send_msg(324, &msg1);
    }

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == RESET)
    {
      HAL_Delay(100);
      if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == RESET)
      {
        uint32_t mailbox;
        uprintf("key1 pressed!\n");
        can_send_msg(324, &msg1);
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        // CAN_FM1R_FBM0 = 0;

      }
    }

    //加入底盘，下方的测试函数弃用，故改为2,--czh
    // if(test_flag0 == 1) {
    //   test_flag0 = 0;
    //   vega_print_pos_can();
    //   //chassis_move((int)test_value[0],ANGLE2RAD(test_value[1]),test_value[2]);
    //   // chassis_canset_motorspeed(test_value[0],test_value[1],test_value[2]);
    // }

    /* USER CODE END WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void inc(void)
{

  if (flag.main_flag == 1)
  {
    time_1ms_cnt++;
    //1000ms
    if (time_1ms_cnt % 1000 == 0)
    {
      flag.clock_1s_flag = 1;
    }
    //500ms
    if (time_1ms_cnt % 500 == 0)
    {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); //led闪烁
    }
    //20ms
    if (time_1ms_cnt % 20 == 0)
    {
    }
    if (time_1ms_cnt % 10 == 0)
    {

      clock.m_sec++;
    }

    //5ms
    if (time_1ms_cnt % 5 == 0)
    {
      flag.lcd_flag = 1;

      time_5ms_cnt = 1;

      if (chassis_status.vega_is_ready == 1)
      {
        flag.chassis_control_flag = 1;
      }
      flag.vesc_flag = 1;
      flag.chassis_laser_flag = 1;
      flag.m2006_flag = 1;
    }

    //vega（全场定位）初始化时间设定，需要有15s的启动时间
    if (time_1ms_cnt % 15000 == 0 && chassis_status.vega_is_ready == 0)
    {
      chassis_status.vega_is_ready = 1;
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      uprintf("Vega init done!!!\r\n");
    }
    if (time_1ms_cnt >= 60000) // 防止int类型溢出
    {
      time_1ms_cnt = 0;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
