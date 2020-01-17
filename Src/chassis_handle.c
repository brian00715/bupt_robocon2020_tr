/*******************************************************************************
Copyright:      BUPT
File name:      chassis_handle.c
Description:    主要放手柄的控制程序方便封装
Author:         ZH
Version：       1.0
Data:           2019/10/21
*******************************************************************************/
#include "chassis_handle.h"
#include "touchdown.h"
Chassis_Handle chassis_handle;
PID_Struct handle_angle_pid = {1,0,0,0,0,5000,0,0.005};//手柄偏高角控制

void handle_button(can_msg *data)
{
  if(0 == flag.main_flag) return;
 
 //TODO手柄按键功能
 uint8_t id = (uint8_t)((data->ui8[0]) * 10 + (data->ui8[1]));
 switch(id){
 case 0:case 10:case 20:case 30:
  flag.chassis_handle_flag = 0;
  flag.chassis_auto_flag = 1;
  uprintf("Change to Auto_mode\r\n");
  break;
case 1:case 11:case 21:case 31:
  flag.chassis_handle_flag = 1;
  flag.chassis_auto_flag = 0;
  uprintf("Change to handle_mode\r\n");
  break;
case 2:
  if(data->ui8[2] == 'u'){
    touchdown_status = TOUCHDOWN_GETBALL;    
    touchdown_ready_flag = 1;
    touchdown_try_flag = 1;
    uprintf("Touchdown try\r\n");
  }
  break;
case 3:
  if(data->ui8[2] == 'u'){
  touchdown_try_finish_flag = 1;
  uprintf("Touchdown try finish\r\n");
  }
  break;
case 4:case 14:case 24:case 34:
chassis_status.trace_count = -1;
  break;
case 5:case 15:case 25:case 35:
chassis_status.trace_count = -1;
  break;
case 6://停下
  chassis_status.trace_count = 0;
  break;
case 7:
  chassis_status.trace_count = 1;
  break;
case 8:
  chassis_status.trace_count = 2;
  break;
case 9:
  chassis_status.trace_count = 3;
  break;
  
case 36:
  chassis_status.trace_count = 4;
  break;
case 37:
  chassis_status.trace_count = 5;
  break;
case 38:
  chassis_status.trace_count = 6;
  break;
case 39:
  chassis_status.trace_count = 7;
  break;
default:break;
 }
}

void handle_rocker(can_msg *data)
{
  if(0 == flag.main_flag || flag.chassis_handle_flag == 0) return;
  
  //TODO :手柄摇杆零偏
  //常数修改零偏
  chassis_handle.ry = (int)data->i16[0] - 15;
  chassis_handle.rx = (int)data->i16[1] - 7;
  chassis_handle.ly = (int)data->i16[2] - 3;
  chassis_handle.lx = (int)data->i16[3] - 2;
  
  //变换坐标系
  chassis_handle.rx *= -1;
  chassis_handle.lx *= -1;
}
//TODO handle_button 在哪里使用参数can_msg来源 手柄发送问题
void handle_exe(){
  if(0 == flag.main_flag || flag.chassis_handle_flag == 0) return;
  //速度为零时，应不读取角度，故将此处的角度先读为temp,
  float temp_fangle = atan2(chassis_handle.ly, chassis_handle.lx);
  //TODO 为手柄控制加入pid
  //加减速用线性变化，此处将速度值设为temp--czh add
  int temp_fspeed = 4*(int)( sqrt(chassis_handle.ly * chassis_handle.ly + chassis_handle.lx * chassis_handle.lx) );
 
  if(temp_fspeed < CHASSIS_HANDLE_MIN_SPEED) {
    temp_fspeed = 0;
    }
  else chassis.fangle = temp_fangle;//速度有效时才对终角赋值

  if(temp_fspeed > CHASSIS_HANDLE_MAX_SPEED) {
    temp_fspeed = CHASSIS_HANDLE_MAX_SPEED;
  }

  int fspeed_diff = temp_fspeed - chassis.fspeed;
  if(fspeed_diff > 5){
    chassis.fspeed =chassis.fspeed + 5;
  }
  else if(fspeed_diff <-10){
    chassis.fspeed =chassis.fspeed - 10;
  }
  else chassis.fspeed =temp_fspeed;

  float temp_fturn = (int)sqrt(chassis_handle.ry * chassis_handle.ry + chassis_handle.rx * chassis_handle.rx);  
  if(temp_fturn > 100){
    temp_fturn = 100 * Angle_Subtract(atan2(chassis_handle.ry, chassis_handle.rx), PI/2) * (-1);
  }
  else 
    temp_fturn = 0;
  float fturn_diff = temp_fturn - chassis.fturn;
  if(fturn_diff > 5){
      chassis.fturn += 5;
    }
    else if(fturn_diff <-5){
      chassis.fturn -= 5;
    }
    else chassis.fturn = temp_fturn;
  /*
  chassis.fspeed = (int)( sqrt(chassis_handle.ly * chassis_handle.ly + chassis_handle.lx * chassis_handle.lx) );
  if(chassis.fspeed < CHASSIS_HANDLE_MIN_SPEED) {
    chassis.fspeed = 0;
    }
  if(chassis.fspeed > CHASSIS_HANDLE_MAX_SPEED) {
    chassis.fspeed = CHASSIS_HANDLE_MAX_SPEED;
  }
  chassis.fturn = (int)sqrt(chassis_handle.ry * chassis_handle.ry + chassis_handle.rx * chassis_handle.rx);  
  if(chassis.fturn > 100){
    chassis.fturn = 75 * Angle_Subtract(atan2(chassis_handle.ry, chassis_handle.rx), PI/2) * (-1);
  }
  else 
    chassis.fturn = 0;
  */
  chassis_move(chassis.fspeed,chassis.fangle,chassis.fturn);  
}