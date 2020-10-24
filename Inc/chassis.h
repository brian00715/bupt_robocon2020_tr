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
extern "C"
{
#endif
/*Include Area*/
#include <math.h>
#include "simplelib.h"
#include "stm32f4xx_hal.h"

#include "vec.h"
#include "configure.h"
#include "utils.h"
#include "point.h"

/*Define Area*/

//角度制转化为弧度制
#define ANGLE2RAD(x) (x / 180.0f * PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x / PI * 180.0f)

  /*Struct Area*/
  typedef struct
  {
    float pos_x; //Chassis
    float pos_y;
    float angle;
    float last_pos_x;
    float last_pos_y;
    float last_angle;
    float speed_x;
    float speed_y;
    float speed_angle;
    float now_speed;

    float vega_pos_x; //vega
    float vega_pos_y;
    float vega_angle;
    float vega_init_pos_x;
    float vega_init_pos_y;
    float vega_init_angle;

    float laser_pos_x; //laser
    float laser_pos_y;
    float laser_angle;

    int fspeed;
    float fangle;
    float fturn;
  } Chassis; // 底盘、全场定位、激光的位姿数据

  typedef struct
  {
    int vega_is_ready; //全场定位是否初始化成功变量，1为已经初始化，0为未初始化
    int go_to_point;

    int is_begin; //用于给point_num赋值的标志位

    int count;          //跑点的计数
    int point_num;      //一段轨迹中的总点数，目前每段轨迹中的点相等
    int trace_count;    //轨迹计数
    int run_point_test; //跑点测试
    int run_point;
  } Chassis_Status; // 底盘的状态机

  typedef struct
  {
    float x;
    float y;
  } Point2D; // 二维空间点

  typedef struct
  {
    float x;
    float y;
    float z;
  } Point3D;

  typedef struct
  {
    float x;
    float y;
    float angle; // 弧度,相对x轴的转角
  } ChassisPosture;

  typedef enum
  {
    ABSOLUTE = 0,
    RELATIVE
  } CHASSIS_POS_MODE;

  /*Variable Area*/
  extern Chassis chassis;
  extern Chassis_Status chassis_status;
  extern PID_Struct position_y_dir_pid;
  extern PID_Struct angle_pid;
  extern float ARRIVE_CIRCLE;
  extern CHASSIS_POS_MODE Chassis_PosMode;
  extern int Chassis_ResetVegaOrigin_Flag;
  extern int Chassis_AutoArrivedAtSpecifiedPoint_Flag;
  extern ChassisPosture Chassis_OriginPosture;
  extern int Chassis_LockYaw_Flag;
  extern float Chassis_LockYaw_Value;
  extern Point2D Chassis_MovePoint;

  /*Function Area*/
  void chassis_init_status();
  void chassis_init_pos(float x, float y);
  void chassis_init(void);
  void chassis_swap_xy(Point points_pos[], int point_num);
  float chassis_calculate_traceangle(float point_x, float point_y);
  int chassis_calculate_linespeed(float point_x, float point_y, int start_speed, int final_speed, int max_speed);
  void chassis_canset_motorspeed(int s1, int s2, int s3);
  void chassis_canset_motorduty(int s1, int s2, int s3);
  void chassis_move(int speed, float direction, float target_angle);
  void chassis_move_vector(vec now_speed_vec, vec target_speed_vec, vec distance_vec, float target_angle);
  int chassis_move_trace(Point points_pos[], int point_num);
  void chassis_move_traces(int trace_rank);
  void Chassis_MoveTo5m();
  void Chassis_MoveTo6m();
  void Chassis_VegaDimTransform();
  void chassis_goto_point(float point_x, float point_y);
  void Chassis_GoToPoint_Plus(float point_x, float point_y, float target_angle);
  void chassis_goto_vector(vec target_position);
  void chassis_finish_onetrace();
  void chassis_pos_update();
  void chassis_exe();
  void chassis_getball_back();
  void chassis_touchdown_back();
  void _chassis_move_vector_2(vec now_speed_vec, vec target_speed_vec, vec distance_vec, float target_angle);
  void Chassis_ResetVegaOrigin();
  void Chassis_PrintPos();
#ifdef __cplusplus
}
#endif
#endif /*__ chassis_H */