/*******************************************************************************
* Copyright:      BUPT
* File name:      chassis_handle.c
* Description:    主要放手柄的控制程序方便封装
* Author:         ZH
* Version：       1.0
* Data:           2019/10/21
*******************************************************************************/
#include "chassis_handle.h"
#include "touchdown.h"
#include "kickball.h"
#include "motor_driver.h"

Chassis_Handle chassis_handle;                                 // 手柄数据结构体，包含摇杆位置数据、模式等
PID_Struct handle_angle_pid = {1, 0, 0, 0, 0, 5000, 0, 0.005}; //手柄偏高角控制

void Handle_Button_New(can_msg *data)
{
  if (0 == flag.main_flag)
    return;
  if ((uint8_t)data->ui8[2] == 0) // 按键按下时才解析指令
    return;
  uint8_t id = (uint8_t)((data->ui8[1]) * 10 + (data->ui8[0])); // data的前两位储存id、
  switch (id)
  {
  // 十位为0系指令用于测试功能
  case 0:
    uprintf("--Handle RX OK!\r\n");
    break;
  case 1:
    uprintf("--lx: %-4d ly: %-4d rx: %-4d ry: %-4d\n",
            chassis_handle.lx, chassis_handle.ly, chassis_handle.rx, chassis_handle.ry);
    break;
  case 5: // 急停
    uprintf("\r\n##All Motor Stoped!##\r\n");
    chassis.fspeed = 0;
    vesc.mode = 1;
    vesc.current = 0;
    chassis_canset_motorspeed(0, 0, 0);
    kickball2_status = KICKBALL2_NONE;
    flag.chassis_handle_flag = 1;
    flag.chassis_auto_flag = 0;
    break;

  // 十位为1系指令用于底盘控制
  case 11:
    flag.chassis_auto_flag = 0;
    flag.chassis_handle_flag = 1;
    uprintf("--Chassis control mode change to ManualMode.\r\n");
    break;
  case 12:
    flag.chassis_handle_flag = 0;
    flag.chassis_auto_flag = 1;
    uprintf("--Chassis control mode change to AutoMode.\r\n");
    break;

  // 十位为2系指令用于踢球动作控制
  case 21: // 设置手动控制踢球
    // 状态机清零
    Kickball2_Kick_Flag = 0;
    Kickball2_Ready_Flag = 0;
    Kickball2_StopRotate_Flag = 0;
    Kickball2_ControlMode = AUTO;
    uprintf("--Kickball control mode switch to auto mode.\r\n");
    break;
  case 22: // 设置自动控制踢球
    Kickball2_ControlMode = MANUAL;
    uprintf("--Kickball control mode switch to manual mode.\r\n");
    uprintf("  ball num = %d\r\n", Kickball2_BallNum); // 踢第几颗球
    break;
  case 23: // 设置踢球状态机为READY状态
    if (Kickball2_ControlMode == MANUAL)
    {
      uprintf("##please change to auto mode!##\r\n");
      return;
    }
    Kickball2_Ready_Flag = 1;
    uprintf("--Hangle: set Kickball2_Ready_Flag to %d!\r\n", Kickball2_Ready_Flag);
    break;
  case 26: // 设置踢球状态机为KICK状态(手柄无法设置踢球电流，要通过串口助手设置)
    if (Kickball2_ControlMode == MANUAL)
    {
      uprintf("##please change to auto mode!##\r\n");
      return;
    }
    Kickball2_Kick_Flag = 1;
    uprintf("--Hangle: set Kickball2_Kick_Flag to %d!\r\n", Kickball2_Kick_Flag);
    break;

  default:
    break;
  }
}

/**
 * @brief 处理手柄摇杆的数据。
 *        左摇杆的深浅控制速度，方向控制偏航角。
 * @param data 手柄摇杆的数据使用int16,can可以一次发送4个
 **/
void Handle_Rocker(can_msg *data)
{
  if (0 == flag.main_flag || flag.chassis_handle_flag == 0)
    return;

  // 常数修改零点漂移
  chassis_handle.ry = (int)data->i16[0] - 15;
  chassis_handle.rx = (int)data->i16[1] - 5;
  chassis_handle.ly = (int)data->i16[2] - 1;
  chassis_handle.lx = (int)data->i16[3];
  // 变换坐标系
  chassis_handle.rx *= -1;
  chassis_handle.lx *= -1;
}

/**
 * @brief 手柄执行函数,左摇杆控制速度矢量，深度控制速度大小；右摇杆控制偏航角，深度控制角速度大小
 **/
void handle_exe()
{
  if (0 == flag.main_flag || flag.chassis_handle_flag == 0)
    return;
  // 速度为零时，应不读取角度，故将此处的角度先读为temp,
  float temp_fangle = atan2(chassis_handle.ly, chassis_handle.lx);
  // 加减速用线性变化，此处将速度值设为temp--czh add
  int temp_fspeed = 2 * (int)(sqrt(chassis_handle.ly * chassis_handle.ly + chassis_handle.lx * chassis_handle.lx));
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
  // 控制加速度
  int fspeed_diff = temp_fspeed - chassis.fspeed;
  if (fspeed_diff > 2)
  {
    chassis.fspeed = chassis.fspeed + 2;
  }
  else if (fspeed_diff < -20)
  {
    chassis.fspeed = chassis.fspeed - 20;
  }
  else
  {
    chassis.fspeed = temp_fspeed;
  }

  // 偏航角控制
  float temp_fturn = (int)sqrt(chassis_handle.ry * chassis_handle.ry + chassis_handle.rx * chassis_handle.rx);
  if (temp_fturn > 100)
  {
    temp_fturn = 100 * Angle_Subtract(atan2(chassis_handle.ry, chassis_handle.rx), PI / 2) * (-1);
  }
  else
  {
    temp_fturn = 0;
  }
  // 控制加速度
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