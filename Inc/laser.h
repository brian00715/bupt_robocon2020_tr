#ifndef __LASER_H
#define __LASER_H

#ifdef __cplusplus
extern "C" 
{
#endif
  
#include "stm32f4xx_hal.h"
#include "main.h"
#include "karman.h"
#include "adc.h"
#include "chassis.h"
#include "utils.h"
  
#define AVERAGE_AMOUNT 10
  
  /*Struct Area*/
  typedef struct
  {
    uint32_t ADC_value[AVERAGE_AMOUNT];
    int ADC_final;
    float distance;
    float FAR_distance;
    float NEAR_distance;
    float FAR_voltage;
    float NEAR_voltage;
    float k_param;
    float b_param;
  }SICK_DT35;
  
  /*Function Area*/
  void laser_init();
  float laser_xpos_correct(float target ,float laser_pos , PID_Struct *laser_pid);
  float laser_ypos_correct(float target ,float laser_pos , PID_Struct *laser_pid);
  void calculate_laser_angle(float laser_pos_L, float laser_pos_R);
  void culculate_k_b_params(SICK_DT35 *sensor);
  float culculate_distance(SICK_DT35 *sensor , Kal_Struct *kal_laser_distance , Kal_Struct *kal_laser_adc); 
  void laser_exe();
  
  /*Variable Area*/
  extern SICK_DT35 laser_L;
  extern SICK_DT35 laser_R;
  
  extern PID_Struct laser_ypos_pid;
  extern PID_Struct laser_xpos_pid;
  
  extern int laser_enable;
  
#ifdef __cplusplus
}
#endif
#endif /*__LASER_H */