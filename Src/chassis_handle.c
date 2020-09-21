/*******************************************************************************
* Copyright:      BUPT
* File name:      chassis_handle.c
* Description:    主要放手柄的控制程序方便封装
* Author:         ZH
* Version：       1.0
* Data:           2019/10/21
*******************************************************************************/
#include "chassis_handle.h"
//#include "touchdown.h"
#include "kickball.h"
Chassis_Handle chassis_handle;  // 手柄数据结构体，包含摇杆位置数据、模式等
PID_Struct handle_angle_pid = {1, 0, 0, 0, 0, 5000, 0, 0.005}; //手柄偏高角控制

void handle_button(can_msg *data)
{
  if (0 == flag.main_flag)
    return;
  uint8_t id = (uint8_t)((data->ui8[0]) * 10 + (data->ui8[1]));  // data的前两位储存id
  //  CAN_ID 20和21切换手动/自动模式
  //         2/3，32/33检测达阵功能
  switch (id)
  {
    case 0: //底盘设置为自动模式，电机占空比设为0
      flag.chassis_handle_flag = 0;
      flag.chassis_auto_flag = 1;
      chassis_status.trace_count = -2;
      uprintf("--Change to Auto_mode\r\n");
      break;
    case 1: //底盘设置为手动模式
      flag.chassis_handle_flag = 1;
      flag.chassis_auto_flag = 0;
      uprintf("--Change to handle_mode\r\n");
      break;
    case 2: //踢球设置为自动模式，重置踢球状态
      kickball_auto = 1;
      kickball_status = KICKBALL_NONE;
      uprintf("--Kickball_auto now\r\n");
      break;
    case 3: //踢球设置为手动模式
      kickball_auto = 0;
      uprintf("--Kickball_handle now\r\n");
      break;
    case 6: //踢球准备
      if (kickball_auto == 1 && kickball_status == KICKBALL_NONE)
      {
        kickball_prepare_flag = 1;
        uprintf("--Kickball prapare!\r\n--Manget is going up\r\n");
      }
      break;
    case 7: //拉电磁铁
      if (kickball_auto == 1 && kickball_status == KICKBALL_MAGNET_READY)
      {
        kickball_pull_magnet_flag = 1;
        uprintf("--Magnet is going down!\r\n");
      }
      break;
    case 8: //停止拉电磁铁
      if (kickball_auto == 0) break;
      if (kickball_status==KICKBALL_BOARD_TO_DOWN5 ||
      kickball_status==KICKBALL_BOARD_TO_DOWN10 ||
      kickball_status==KICKBALL_BOARD_TO_DOWN20)
      {
        kickball_stop_magnet_flag = 1;
        uprintf("--Magnet stopped!\r\nReady to kick\r\n");
      }
      break;
    case 9: //开始踢球
      if (kickball_auto == 1 && kickball_status == KICKBALL_BOARD_READY)
      {
        kickball_kick_flag = 1;
        uprintf("--Kick the ball!!!\r\n");
      }
      break;
    
    default:
      break;
  }
  /*
  switch (id)
  {
  case 0:
  case 10:
  case 20:
    flag.chassis_handle_flag = 0;
    flag.chassis_auto_flag = 1;
    uprintf("Change to Auto_mode\r\n");
    break;
  case 1:
  case 11:
  case 21:
    flag.chassis_handle_flag = 1;
    flag.chassis_auto_flag = 0;
    uprintf("Change to handle_mode\r\n");
    break;
  case 2:  // 检测达阵状态是否正常
    if (data->ui8[2] == 'u')
    {
      touchdown_status = TOUCHDOWN_GETBALL;
      touchdown_ready_flag = 1;
      touchdown_try_flag = 1;
      uprintf("Touchdown try\r\n");
    }
    break;
  case 3:
    if (data->ui8[2] == 'u')
    {
      touchdown_try_finish_flag = 1;
      uprintf("Touchdown try finish\r\n");
    }
    break;
  case 6:
    chassis_status.trace_count = 1;
    break;
  case 7:
    chassis_status.trace_count = 10;
    break;
  case 8:
    break;
  case 9:
    break;
  case 4:
  case 14:
  case 24:
  case 34:
    chassis_status.trace_count = -1;
    break;
  case 5:
  case 15:
  case 25:
  case 35:
    chassis_status.trace_count = -1;
    break;

  case 30:
    flag.chassis_auto_flag = 1;
    flag.chassis_handle_flag = 0;
    chassis_status.trace_count = 7;
    break;
  case 31:
    flag.chassis_auto_flag = 1;
    flag.chassis_handle_flag = 0;
    chassis_status.trace_count = 6;
    break;
  case 32:
    if (data->ui8[2] == 'u')
    {
      touchdown_status = TOUCHDOWN_GETBALL;
      touchdown_ready_flag = 1;
      touchdown_try_flag = 1;
      uprintf("Touchdown try\r\n");
    }
    break;
  case 33:
    if (data->ui8[2] == 'u')
    {
      touchdown_try_finish_flag = 1;
      uprintf("Touchdown try finish\r\n");
    }
    break;
  case 36:
    flag.chassis_auto_flag = 1;
    flag.chassis_handle_flag = 0;
    chassis_status.trace_count = 2;
    break;
  case 37:
    flag.chassis_auto_flag = 1;
    flag.chassis_handle_flag = 0;
    chassis_status.trace_count = 3;
    break;
  case 38:
    flag.chassis_auto_flag = 1;
    flag.chassis_handle_flag = 0;
    chassis_status.trace_count = 4;
    break;
  case 39:
    flag.chassis_auto_flag = 1;
    flag.chassis_handle_flag = 0;
    chassis_status.trace_count = 5;
    break;
  default:
    break;
  }
  */
}

/**
 * @brief 处理手柄摇杆的数据。
 *        左摇杆的深浅控制速度，方向控制偏航角。
 * @param data 手柄摇杆的数据使用int16,can可以一次发送4个
 **/
void handle_rocker(can_msg *data)
{
  if (0 == flag.main_flag || flag.chassis_handle_flag == 0)
    return;

  // 常数修改零点漂移
  chassis_handle.ry = (int)data->i16[0] - 15;
  chassis_handle.rx = (int)data->i16[1] - 7;
  chassis_handle.ly = (int)data->i16[2] - 3;
  chassis_handle.lx = (int)data->i16[3] - 2;
  // 变换坐标系
  chassis_handle.rx *= -1;
  chassis_handle.lx *= -1;
}

/**
 * @brief 手柄执行函数
 **/
void handle_exe()
{
  if (0 == flag.main_flag || flag.chassis_handle_flag == 0)
    return;
  // 速度为零时，应不读取角度，故将此处的角度先读为temp,
  float temp_fangle = atan2(chassis_handle.ly, chassis_handle.lx);
  // 加减速用线性变化，此处将速度值设为temp--czh add
  int temp_fspeed = 4 * (int)(sqrt(chassis_handle.ly * chassis_handle.ly + chassis_handle.lx * chassis_handle.lx));
  if (temp_fspeed < CHASSIS_HANDLE_MIN_SPEED)
  {
    temp_fspeed = 0;
  }
  else if (temp_fspeed > CHASSIS_HANDLE_MAX_SPEED)
  {
    temp_fspeed = CHASSIS_HANDLE_MAX_SPEED;
  }
  else
  {
    chassis.fangle = temp_fangle;
  }

  int fspeed_diff = temp_fspeed - chassis.fspeed;
  if (fspeed_diff > 5)
  {
    chassis.fspeed = chassis.fspeed + 5;
  }
  else if (fspeed_diff < -10)
  {
    chassis.fspeed = chassis.fspeed - 10;
  }
  else
    chassis.fspeed = temp_fspeed;

  float temp_fturn = (int)sqrt(chassis_handle.ry * chassis_handle.ry + chassis_handle.rx * chassis_handle.rx);
  if (temp_fturn > 100)
  {
    temp_fturn = 100 * Angle_Subtract(atan2(chassis_handle.ry, chassis_handle.rx), PI / 2) * (-1);
  }
  else
  {
    temp_fturn = 0;
  }

  float fturn_diff = temp_fturn - chassis.fturn;
  if (fturn_diff > 5)
  {
    chassis.fturn += 5;
  }
  else if (fturn_diff < -5)
  {
    chassis.fturn -= 5;
  }
  else
  {
    chassis.fturn = temp_fturn;
  }
  chassis_move(chassis.fspeed, chassis.fangle, chassis.fturn);
}