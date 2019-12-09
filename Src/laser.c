/*******************************************************************************
Copyright:      Bupt
File name:      laser.c
Description:    激光
Author:         20th
Version：       1.0
Data:           2019/12/9
*******************************************************************************/
#include "laser.h"

#define DIS_BTW_LR 0.45   
#define ERROR_ON_LR 0.004 

LASER laser_left;
LASER laser_side;
LASER laser_right;

LASER_ADC laser_adc[AVERAGE_AMOUNT];

//TODO : pid文件
PID_Struct laser_ypos_pid = {200,50,0,0,0,0,0,0.005};
PID_Struct laser_xpos_pid = {700,50,0,0,0,0,0,0.005};

int laser_enable = 0;
/**激光线性方程参数*/
void laser_calculate_kb(LASER *sensor){
  sensor->k_param = (sensor->FAR_distance - sensor->NEAR_distance) / (sensor->FAR_voltage - sensor->NEAR_voltage);
  sensor->b_param = sensor->FAR_distance - sensor->k_param * sensor->FAR_voltage;
  uprintf("k is %f and b is %f\r\n", sensor->k_param, sensor->b_param);
}
/**j激光ADC三个通道分别取数*/
void laser_adc_split(LASER* laser_l,LASER* laser_r,LASER* laser_s){
  //TODO :区分三个激光分别对应adc的第几个通道 在laser_exe中调用
  for (int i = 0; i < AVERAGE_AMOUNT; i++){
    laser_l->ADC_value[i]=laser_adc[i].adc_l;
    laser_r->ADC_value[i]=laser_adc[i].adc_r;
    laser_s->ADC_value[i]=laser_adc[i].adc_s;
  }
}
/**激光计算距离*/
float laser_calculate_distance(LASER *sensor , Kal_Struct *kal_laser_distance , Kal_Struct *kal_laser_adc){
  float sum_up = 0;
  float distance;
  for (int i = 0; i < AVERAGE_AMOUNT; i++){
    sum_up += sensor->ADC_value[i];
  }
  sensor->ADC_final = (int)sum_up / AVERAGE_AMOUNT;
  //TODO :两个卡尔曼滤波是否有影响???
  sensor->ADC_final = (int)KalMan(kal_laser_adc, sensor->ADC_final);
  distance = sensor->ADC_final * sensor->k_param + sensor->b_param;
  distance = KalMan(kal_laser_distance, distance);
  return distance;
}
//TODO :激光计算x  y  angle 的方式
/**激光计算x*/
float laser_calculate_x(float target ,float laser_pos , PID_Struct *laser_pid){
  if(laser_enable != 1){
    return 0;
  }
  
  float error = 0;
  error = target - laser_pos;
  
  float pwm = laser_pid->KP * error + laser_pid->KD * (error - laser_pid->last_err);
  
  laser_pid->last_err = error;
  return pwm;
}
/**激光计算y*/
float laser_calculate_y(float target ,float laser_pos , PID_Struct *laser_pid){
  if(laser_enable != 1){
    return 0;
  }
  
  float error = 0;
  error = target - laser_pos;
  
  float pwm = laser_pid->KP * error + laser_pid->KD * (error - laser_pid->last_err);
  
  laser_pid->last_err = error;
  return pwm;
}
/**激光计算角度*/
float laser_calculate_angle(float laser_pos_L, float laser_pos_R){
  if(laser_enable != 1){
    return 0;
  }
  
  float error = laser_pos_L - laser_pos_R;
  if( fabsf(error) < 1e-3 || fabsf(error) >= 0.3 )
  {
    return 0;
  }
  
  float angle = asin(error / DIS_BTW_LR);
  return angle;
}
/**激光初始化:计算kb;打开DMA*/
void laser_init(){
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

  uprintf("Left---");
  laser_calculate_kb(&laser_left);
  uprintf("Right---");
  laser_calculate_kb(&laser_right);
  uprintf("Side---");
  laser_calculate_kb(&laser_side);
  
  if (HAL_ADC_Start_DMA(&hadc1,(uint32_t *)laser_adc, 3*AVERAGE_AMOUNT) != HAL_OK){
    while (1);
  }
}
/**激光执行函数: 获取三个激光的距离;计算x y angle*/
void laser_exe(){
  laser_adc_split(&laser_left,&laser_right,&laser_side);
  laser_left.distance = laser_calculate_distance(&laser_left , &kal_distance_L , &kal_adc_L);
  //TODO : ERROR_ON_LR有啥用????
  laser_right.distance = laser_calculate_distance(&laser_right , &kal_distance_R , &kal_adc_R) + ERROR_ON_LR;
  laser_side.distance = laser_calculate_distance(&laser_side , &kal_distance_S , &kal_adc_S);

//TODO :下面三个函数待修改
  // chassis.laser_pos_x = laser_calculate_x(laser_left.distance , laser_right.distance,);
  // chassis.laser_pos_y = laser_calculate_y(laser_left.distance , laser_right.distance);
  chassis.laser_angle = laser_calculate_angle(laser_left.distance , laser_right.distance);
}
/**激光打印三个距离*/
void laser_print_distance(){
  uprintf("Left :%6fm\r\n",laser_left.distance);
  uprintf("Right:%6fm\r\n",laser_right.distance);
  uprintf("Side :%6fm\r\n",laser_side.distance);
}
/**激光打印pos*/
void laser_print_pos(){
  uprintf("Laser_X:%6fm\r\n",chassis.vega_pos_x);
  uprintf("Laser_X:%6fm\r\n",chassis.vega_pos_y);
  uprintf("Laser_A:%6f\r\n",chassis.vega_angle);
}