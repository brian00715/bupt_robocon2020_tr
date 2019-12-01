/*******************************************************************************
Copyright:      BUPT
File name:      chassis.c
Description:    全向轮底盘控制代码
Author:         20th
Version：       1.1 更新了对于19年frame的支持
Data:           2019/10/9
History:        1.0 见老版代码
*******************************************************************************/
#ifndef __chassis_H
#define __chassis_H
#ifdef __cplusplus
extern "C" {
#endif
/*Include Area*/
#include <math.h>
#include "simplelib.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "vec.h"
#include "motordriver.h"
#include "point.h"
#include "utils.h"
#include "laser.h"
/*Define Area*/

/*Struct Area*/
typedef struct
{
  float pos_x;           //Chassis
  float pos_y;
  float angle;
  float last_pos_x;
  float last_pos_y;
  float last_angle;
  float speed_x;
  float speed_y;
  float speed_angle;
  float now_speed;

  float vega_pos_x;       //vega
  float vega_pos_y;   
  float vega_angle;	
  float vega_init_pos_x;
  float vega_init_pos_y;   
  float vega_init_angle;	

  float laser_pos_x;        //laser
  float laser_pos_y;
  float laser_angle;
  //TODO laser控制需定义一个pidstruct
  int laser_pwm;           //laser control pid
  float laser_pwm_angle;
  
  int fspeed;
  float fangle;
  float fturn;

} Chassis; 

typedef struct
{   
  int vega_is_ready;    //全场定位是否初始化成功变量，1为已经初始化，0为未初始化
  int go_to_point;                                               
    
  int is_begin;                   //用于给point_num赋值的标志位
  int count;                      //跑点的计数
  int point_num;                  //一段轨迹中的总点数，目前每段轨迹中的点相等
  int trace_count;                //轨迹计数  

    
  int run_point_test;             //跑点测试
  int run_point;
} Chassis_Status;

/*Function Area*/
void chassis_zero();
void chassis_init(void);
void chassis_update(void);
void chassis_init_pos(float x,float y);

//Until Functions:
float chassis_angle_subtract(float a, float b);
void chassis_gostraight(int speed , float angle, float turn, int is_handle);
void go_to_point_test(float point_x , float point_y);
void vector_track_ctrl(vec now, vec target, vec direct, float target_angle);
void chassis_vector_test();
int  chassis_go_track(struct point points_pos[],int point_num);
void chassis_exe();
void chassis_run_point(int trace_count);

/*Variable Area*/
extern Chassis chassis;
extern Chassis_Status chassis_status;
extern PID_Struct position_y_dir_pid;
extern float Arrive_distance;

#ifdef __cplusplus
}
#endif
#endif /*__ chassis_H */