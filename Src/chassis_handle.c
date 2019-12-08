/*******************************************************************************
Copyright:      BUPT
File name:      chassis_handle.c
Description:    主要放手柄的控制程序方便封装
Author:         ZH
Version：       1.0
Data:           2019/10/21
*******************************************************************************/
#include "chassis_handle.h"
CHASSIS_HANDLE ChassisHandle;

void handle_button(can_msg *data)
{
  if(0 == flag.main_run_flag) return;
 
 //TODO手柄接收格式???按键功能
 uint8_t id = (uint8_t)((data->ui8[0]) * 10 + (data->ui8[1]));
 switch(id)
 {
 case 0:
   flag.chassis_handle_flag = 1;
   flag.chassis_automate_flag = 0;
   chassis_status.count = 0;
   chassis_status.run_point = 0;
   //!可设置手柄最大速度
   ChassisHandle.handle_max_speed = 600;
   uprintf("run:%d,go_to:%d,count:%d,trace:%d,handle:%d,auto:%d,is_begin:%d,handlemode:%d\r\n",
        chassis_status.run_point,chassis_status.go_to_point,chassis_status.count,chassis_status.trace_count,
        flag.chassis_handle_flag,flag.chassis_automate_flag,chassis_status.is_begin,ChassisHandle.mode);
   break;
 case 8:
   flag.chassis_handle_flag = 0;
   flag.chassis_automate_flag = 1;
   break;
 case 2:
   ChassisHandle.mode = (ChassisHandle.mode + 1) % 4;
   break;
 case 6:
   chassis_status.run_point = 1;
   break;
 }
}

void handle_rocker(can_msg *data)
{
  if(0 == flag.main_run_flag || flag.chassis_handle_flag == 0) return;
  
  //TODO :手柄摇杆零偏
  //常数修改零偏
  ChassisHandle.ry = (int)data->i16[0] - 14;
  ChassisHandle.rx = (int)data->i16[1] - 4;
  ChassisHandle.ly = (int)data->i16[2] - 2;
  ChassisHandle.lx = (int)data->i16[3] - 3;
  
  //变换坐标系
  ChassisHandle.rx *= -1;
  ChassisHandle.lx *= -1;
}

void handle_exe()
{
  if(0 == flag.main_run_flag || flag.chassis_handle_flag == 0) return;
  
  chassis.fangle = atan2(ChassisHandle.ly, ChassisHandle.lx);
  
  chassis.fspeed = (int)( sqrt(ChassisHandle.ly * ChassisHandle.ly + ChassisHandle.lx * ChassisHandle.lx) );
  if(chassis.fspeed < 60) chassis.fspeed = 0;
  if(chassis.fspeed > ChassisHandle.handle_max_speed) chassis.fspeed = ChassisHandle.handle_max_speed;
  
  chassis.fturn = (int)sqrt(ChassisHandle.ry * ChassisHandle.ry + ChassisHandle.rx * ChassisHandle.rx);  
  if(chassis.fturn > 100)
  chassis.fturn = 100 * chassis_angle_subtract(atan2(ChassisHandle.ry, ChassisHandle.rx), PI/2) * (-1);
  else 
  chassis.fturn = 0;
  
  chassis_gostraight(chassis.fspeed,chassis.fangle,chassis.fturn,1);  
}