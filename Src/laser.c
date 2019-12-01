#include "laser.h"

SICK_DT35 laser_L;
SICK_DT35 laser_R;

PID_Struct laser_ypos_pid = {200,50,0,0,0,0,0,0.005};
PID_Struct laser_xpos_pid = {700,50,0,0,0,0,0,0.005};

#define DIS_BTW_LR 0.45   //???????????????
#define ERROR_ON_LR 0.004 //??????????????????????


int laser_enable = 0;//???????


/*laser_init
*????
*??: ????????????????AD??????????AD?????????ADC??
*??: zh
*/
void laser_init()
{
  laser_L.FAR_distance = 0.7;
  laser_L.FAR_voltage = 2663;
  laser_L.NEAR_distance = 0.2;
  laser_L.NEAR_voltage = 464;
  
  laser_R.NEAR_voltage = 457;
  laser_R.NEAR_distance = 0.20;
  laser_R.FAR_distance = 0.70;
  laser_R.FAR_voltage = 2649;
  
  culculate_k_b_params(&laser_L);
  culculate_k_b_params(&laser_R);
  
  if (HAL_ADC_Start_DMA(&hadc1, laser_L.ADC_value, AVERAGE_AMOUNT) != HAL_OK)
        while (1)
            ;
}

/*laser_pos_correct
*??????????float target?????????PD??float kp,float kd
*??: ????????????
*??: zh
*/
float laser_xpos_correct(float target ,float laser_pos , PID_Struct *laser_pid)
{
  if(laser_enable != 1)
  {
    return 0;
  }
  
  float error = 0;
  error = target - laser_pos;
  
  float pwm = laser_pid->KP * error + laser_pid->KD * (error - laser_pid->last_err);
  
  laser_pid->last_err = error;
  return pwm;
}

/*laser_pos_correct
*??????????float target?????????PD??float kp,float kd
*??: ????????????
*??: zh
*/
float laser_ypos_correct(float target ,float laser_pos , PID_Struct *laser_pid)
{
  if(laser_enable != 1)
  {
    return 0;
  }
  
  float error = 0;
  error = target - laser_pos;
  
  float pwm = laser_pid->KP * error + laser_pid->KD * (error - laser_pid->last_err);
  
  laser_pid->last_err = error;
  return pwm;
}

/*void calculate_laser_angle(float laser_pos_L, float laser_pos_R)
*???float laser_pos_L, float laser_pos_R??????????????????????????????????
*??: ????????????
*??: zh
*/
void calculate_laser_angle(float laser_pos_L, float laser_pos_R)
{
  if(laser_enable != 1)
  {
    return;
  }
  
  float error = laser_pos_L - laser_pos_R;
  if( fabsf(error) < 1e-3 || fabsf(error) >= 0.3 )
  {
    return;
  }
  
  float angle = error / DIS_BTW_LR;
  chassis.laser_angle = asin(angle);
}

/*culculate_distance
*???SICK_DT35 *sensor????????, Kal_Struct *kal_laser_distance???????????, Kal_Struct *kal_laser_adc??ADC???????
*??: ??????????
*??: zh
*/
float culculate_distance(SICK_DT35 *sensor , Kal_Struct *kal_laser_distance , Kal_Struct *kal_laser_adc) 
{
  float sum_up = 0;
  float distance;
  for (int i = 0; i < AVERAGE_AMOUNT; i++) 
  {
    sum_up += sensor->ADC_value[i];
  }
  sensor->ADC_final = (int)sum_up / AVERAGE_AMOUNT;
  sensor->ADC_final = (int)KalMan(kal_laser_adc, sensor->ADC_final);
  distance = sensor->ADC_final * sensor->k_param + sensor->b_param;//0.005??????
  distance = KalMan(kal_laser_distance, distance);
  return distance;
}

/*void culculate_k_b_params(SICK_DT35 *sensor) 
*???SICK_DT35 *sensor????????
*??: ???AD???????????????????????????????
*??: zh
*/
void culculate_k_b_params(SICK_DT35 *sensor) 
{
  sensor->k_param = (sensor->FAR_distance - sensor->NEAR_distance) / (sensor->FAR_voltage - sensor->NEAR_voltage);
  sensor->b_param = sensor->FAR_distance - sensor->k_param * sensor->FAR_voltage;
  uprintf("k is %f and b is %f\r\n", sensor->k_param, sensor->b_param);
}

/*void laser_exe() 
*????
*??: ???????????????while?1????
*??: zh
*/
void laser_exe()
{
  laser_L.distance = culculate_distance(&laser_L , &kal_distance_L , &kal_adc_L);
  laser_R.distance = culculate_distance(&laser_R , &kal_distance_R , &kal_adc_R) + ERROR_ON_LR;
  calculate_laser_angle(laser_L.distance , laser_R.distance);
}