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

/*Define Area*/

#define AVERAGE_AMOUNT 10

/*Struct Area*/

typedef struct{
  uint32_t adc_l;
  uint32_t adc_r;
  uint32_t adc_s;
}LASER_ADC;
typedef struct{
  uint32_t ADC_value[AVERAGE_AMOUNT];
  int ADC_final;
  float distance;
  float FAR_distance;
  float NEAR_distance;
  float FAR_voltage;
  float NEAR_voltage;
  float k_param;
  float b_param;
}LASER;

/*Variable Area*/

extern LASER laser_left;
extern LASER laser_side;
extern LASER laser_right;

extern LASER_ADC laser_adc[AVERAGE_AMOUNT];

extern PID_Struct laser_ypos_pid;
extern PID_Struct laser_xpos_pid;

extern int laser_enable;

/*Function Area*/

void laser_culculate_kb(LASER *sensor);
void laser_adc_split(LASER* laser_l,LASER* laser_r,LASER* laser_s);

float laser_culculate_distance(LASER *sensor , Kal_Struct *kal_laser_distance , Kal_Struct *kal_laser_adc);
float laser_culculate_x(float target ,float laser_pos , PID_Struct *laser_pid);
float laser_culculate_y(float target ,float laser_pos , PID_Struct *laser_pid);
float laser_calculate_angle(float laser_pos_L, float laser_pos_R);

void laser_init();
void laser_exe();
void laser_print_distance();
void laser_print_pos();


#ifdef __cplusplus
}
#endif
#endif /*__LASER_H */