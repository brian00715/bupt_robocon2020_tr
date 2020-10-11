/*******************************************************************************
Copyright:      Bupt
File name:      laser.c
Description:    激光
Author:         20th
Version：       1.0
Data:           2019/12/9
*******************************************************************************/
#include "laser.h"
//TODO :区分三个激光分别对应adc的第几个通道

#define ERROR_ON_LEFT 0 //三个激光与实际的偏差
#define ERROR_ON_RIGHT 0
#define ERROR_ON_SIDE 0
#define DIS_BTW_LR 0.45

LASER laser_left;
LASER laser_side;
LASER laser_right;

LASER_ADC laser_adc[AVERAGE_AMOUNT]; // 直接从端口读入的原始数据

PID_Struct laser_ypos_pid = {200, 50, 0, 0, 0, 0, 0, 0.005};
PID_Struct laser_xpos_pid = {700, 50, 0, 0, 0, 0, 0, 0.005};

/**激光线性方程参数*/
void laser_calculate_kb(LASER *sensor)
{
  sensor->k_param = (sensor->FAR_distance - sensor->NEAR_distance) / (sensor->FAR_voltage - sensor->NEAR_voltage);
  sensor->b_param = sensor->FAR_distance - sensor->k_param * sensor->FAR_voltage;
  // uprintf("k is %f and b is %f\r\n", sensor->k_param, sensor->b_param);
}

/**激光ADC三个通道分别取数*/
void laser_adc_split(LASER *laser_l, LASER *laser_r, LASER *laser_s)
{

  for (int i = 0; i < AVERAGE_AMOUNT; i++)
  {
    laser_l->ADC_value[i] = laser_adc[i].adc_l;
    laser_r->ADC_value[i] = laser_adc[i].adc_r;
    laser_s->ADC_value[i] = laser_adc[i].adc_s;
  }
}

/**激光计算距离*/
float laser_calculate_distance(LASER *sensor, Kal_Struct *kal_laser_distance, Kal_Struct *kal_laser_adc)
{
  float sum_up = 0;
  float distance;
  for (int i = 0; i < AVERAGE_AMOUNT; i++)
  {
    sum_up += sensor->ADC_value[i];
  }
  sensor->ADC_final = (int)sum_up / AVERAGE_AMOUNT;  // 均值滤波
  sensor->ADC_final = (int)KalMan(kal_laser_adc, sensor->ADC_final);
  distance = sensor->ADC_final * sensor->k_param + sensor->b_param;
  distance = KalMan(kal_laser_distance, distance);
  return distance;
}

//激光计算x  y  angle 的方式
/**激光计算x*/
float laser_calculate_x()
{
  float laser_x = 0;
  return laser_x;
}
/**激光计算y*/
float laser_calculate_y()
{
  float laser_y = 0;
  return laser_y;
}

/**激光计算角度*/
float laser_calculate_angle()
{
  float laser_angle = 0;
  return laser_angle;
}

/**激光初始化:计算kb;打开DMA*/
void laser_init()
{

  //TODO 待校准
  laser_left.FAR_distance = 0.7;
  laser_left.FAR_voltage = 2663;
  laser_left.NEAR_distance = 0.2;
  laser_left.NEAR_voltage = 464;

  laser_right.NEAR_voltage = 457;
  laser_right.NEAR_distance = 0.20;
  laser_right.FAR_distance = 0.70;
  laser_right.FAR_voltage = 2649;

  laser_side.NEAR_voltage = 457;
  laser_side.NEAR_distance = 0.20;
  laser_side.FAR_distance = 0.70;
  laser_side.FAR_voltage = 2649;

  // uprintf("Left---");
  laser_calculate_kb(&laser_left);
  // uprintf("Right---");
  laser_calculate_kb(&laser_right);
  // uprintf("Side---");
  laser_calculate_kb(&laser_side);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)laser_adc, 3 * AVERAGE_AMOUNT) != HAL_OK)
  {
    uprintf("Laser DMA wrong!!!\r\n");
    while (1)
    {
    }
  }
}

/**激光执行函数: 获取三个激光的距离;计算x y angle*/
void laser_exe()
{
  if (flag.chassis_laser_flag != 1)
  {
    return;
  }

  laser_adc_split(&laser_left, &laser_right, &laser_side);
  laser_left.distance = laser_calculate_distance(&laser_left, &kal_distance_L, &kal_adc_L) + ERROR_ON_LEFT;
  laser_right.distance = laser_calculate_distance(&laser_right, &kal_distance_R, &kal_adc_R) + ERROR_ON_RIGHT;
  laser_side.distance = laser_calculate_distance(&laser_side, &kal_distance_S, &kal_adc_S) + ERROR_ON_SIDE;

  chassis.laser_pos_x = laser_calculate_x();
  chassis.laser_pos_y = laser_calculate_y();
  chassis.laser_angle = laser_calculate_angle();

  flag.chassis_laser_flag = 0;
}

/**激光打印三个距离*/
void laser_print_distance()
{
  uprintf("Left :%6fm\r\n", laser_left.distance);
  uprintf("Right:%6fm\r\n", laser_right.distance);
  uprintf("Side :%6fm\r\n", laser_side.distance);
}

/**激光打印pos*/
void laser_print_pos()
{
  uprintf("Laser_X:%6fm\r\n", chassis.vega_pos_x);
  uprintf("Laser_X:%6fm\r\n", chassis.vega_pos_y);
  uprintf("Laser_A:%6f\r\n", chassis.vega_angle);
}

void laser_print_raw_value()
{
  // uprintf("--laser:raw adc value\r\n");
  // uprintf("  left:");
  // for(int i=0;i<3;i++)
  // {
  //   uprintf("%.2f ",laser_adc[i].adc_l);
  // }
  // uprintf("\r\n");
  uprintf("--adc1 value: %d \r\n",HAL_ADC_GetValue(&hadc1));
  
}