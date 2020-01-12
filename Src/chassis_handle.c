/*******************************************************************************
Copyright:      BUPT
File name:      chassis_handle.c
Description:    主要放手柄的控制程序方便封装
Author:         ZH
Version：       1.0
Data:           2019/10/21
*******************************************************************************/
#include "chassis_handle.h"
Chassis_Handle chassis_handle;
PID_Struct handle_angle_pid = {1,0,0,0,0,5000,0,0.005};//手柄偏高角控制

void handle_button(can_msg *data)
{
  if(0 == flag.main_flag) return;
 
 //TODO手柄按键功能
 uint8_t id = (uint8_t)((data->ui8[0]) * 10 + (data->ui8[1]));
 switch(id){
case 0:
  flag.chassis_handle_flag = 1;
  flag.chassis_auto_flag = 0;
  break;
case 1:
  flag.chassis_handle_flag = 0;
  flag.chassis_auto_flag = 1;
  break;
case 2:
  break;
case 3:
  break;
case 4:
  break;
case 5:
  break;
case 6:
  break;
case 7:
  break;
case 8:
  break;
case 9:
  break;
default:break;
 }
}

void handle_rocker(can_msg *data)
{
  if(0 == flag.main_flag || flag.chassis_handle_flag == 0) return;
  
  //TODO :手柄摇杆零偏
  //常数修改零偏
  chassis_handle.ry = (int)data->i16[0] - 14;
  chassis_handle.rx = (int)data->i16[1] - 4;
  chassis_handle.ly = (int)data->i16[2] - 2;
  chassis_handle.lx = (int)data->i16[3] - 3;
  
  //变换坐标系
  chassis_handle.rx *= -1;
  chassis_handle.lx *= -1;
}
//TODO handle_button 在哪里使用参数can_msg来源 手柄发送问题
void handle_exe()
{
  if(0 == flag.main_flag || flag.chassis_handle_flag == 0) return;
  
  chassis.fangle = atan2(chassis_handle.ly, chassis_handle.lx);
  //TODO 为手柄控制加入pid
  chassis.fspeed = (int)( sqrt(chassis_handle.ly * chassis_handle.ly + chassis_handle.lx * chassis_handle.lx) );
  if(chassis.fspeed < 60) {
    chassis.fspeed = 0;
    }
  if(chassis.fspeed > chassis_handle.handle_max_speed) {
    chassis.fspeed = chassis_handle.handle_max_speed;
  }
  chassis.fturn = (int)sqrt(chassis_handle.ry * chassis_handle.ry + chassis_handle.rx * chassis_handle.rx);  
  if(chassis.fturn > 100){
    chassis.fturn = 100 * Angle_Subtract(atan2(chassis_handle.ry, chassis_handle.rx), PI/2) * (-1);
  }
  else 
    chassis.fturn = 0;
  
  chassis_move(chassis.fspeed,chassis.fangle,chassis.fturn);  
}