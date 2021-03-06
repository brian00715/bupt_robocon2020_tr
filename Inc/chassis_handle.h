

#ifndef __CHASSIS_HANDLE_H
#define __CHASSIS_HANDLE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "usart.h"
#include "utils.h"
#include "chassis.h"
#include "can.h"
#include "main.h"
#include "can_utils.h"
#include "touchdown.h"
#include "kickball.h"
#include "motor_driver.h"
#include "vega.h"
#include "vesc_can.h"
#include "led.h"

/*Define Area*/
#define CHASSIS_HANDLE_MAX_SPEED 400
#define CHASSIS_HANDLE_MIN_SPEED 20 // 默认150

  /*Struct Area*/
  typedef struct
  {
    int lx; // 左侧摇杆的位置数据
    int ly;
    int rx; // 右侧摇杆的位置数据
    int ry;
    int handle_max_speed;
    int mode;
    int btstate[10];
  } Chassis_Handle; // 手柄数据结构体

  /*Variable Area*/
  extern Chassis_Handle chassis_handle;
  extern float Handle_LeftRocker_SpeedTransRatio;
  extern float Handle_RightRocker_SpeedTransRatio;
  extern int Chassis_DimReverse_Flag;
  extern int DistanceToBallSocketOK_Flag;
  extern int Handle_LeftRocker_Length;
  extern int Handle_RightRocker_Length;
  extern int correct_ry;
  extern int correct_rx;
  extern int correct_ly;
  extern int correct_lx;

  /*Function Area*/
  void handle_button(can_msg *data);
  void Handle_Rocker(can_msg *data);
  void Handle_Button_New(can_msg *data);
  void handle_exe();

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_HANDLE_H */