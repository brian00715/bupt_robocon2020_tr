/*******************************************************************************
* Copyright:      BUPT
* File name:      chassis_handle.c
* Description:    主要放手柄的控制程序方便封装
* Author:         ZH
* Version：       1.0
* Data:           2019/10/21
*******************************************************************************/
#include "chassis_handle.h"

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
  case 2:
    vega_print_pos();
    break;
  case 3:
    Chassis_PrintPos();
    break;
  case 4:
    Chassis_ResetVegaOrigin_Flag = 1; // 为了安全起见，开启原点重置后就不再关闭
    Chassis_ResetVegaOrigin();        // 更新原点
    uprintf("--vega origin point has been reseted.\r\n");
    break;

  case 5: // 急停
    uprintf("\r\n##All Motor Stoped!##\r\n");
    chassis.fspeed = 0;
    vesc.mode = 1;
    vesc.current = 0;
    Kickball2_Ready_Flag = 0;
    Kickball2_Kick_Flag = 0;
    Kickball2_SetState(KICKBALL2_NONE);
    comm_can_set_current(vesc.id, 0);
    chassis_canset_motorspeed(0, 0, 0);
    kickball2_status = KICKBALL2_NONE;
    flag.chassis_handle_flag = 1;
    flag.chassis_auto_flag = 0;
    led_control(5);
    break;
  case 6:
    led_control(6);
    Chassis_DimReverse_Flag = (Chassis_DimReverse_Flag + 1) % 2;
    if (Chassis_DimReverse_Flag)
    {
      uprintf("--chassis move direction has been reversed\r\n");
    }
    else
    {
      uprintf("--chassis move dierction has been reduction\r\n");
    }
    break;

  // 十位为1系指令用于底盘控制
  case 11:
    flag.chassis_auto_flag = (flag.chassis_auto_flag + 1) % 2;
    flag.chassis_handle_flag = (flag.chassis_handle_flag + 1) % 2;
    if (flag.chassis_handle_flag == 1 && flag.chassis_auto_flag == 0)
    {
      uprintf("--Chassis control mode change to ManualMode.\r\n");
      led_control(10);
    }
    else
    {
      led_control(11);
      uprintf("--Chassis control mode change to AutoMode.\r\n");
    }
    break;
  case 12:
    Chassis_PosMode = ABSOLUTE;
    uprintf("--Chassis pos mode change to absulute.\r\n");
    break;
  case 13:
    Chassis_PosMode = RELATIVE;
    uprintf("--Chassis pos mode change to relative.\r\n");
    break;
  case 14:
    led_control(14);
    SPEED_TRANSFORM_RATIO = 1;
    uprintf("--Left Rocker speed transform ratio change to 1\r\n");
    break;
  case 15:
    led_control(15);
    SPEED_TRANSFORM_RATIO = 3;
    uprintf("--Left Rocker speed transform ratio change to 3\r\n");
    break;
  case 16:
    led_control(16);
    Chassis_AutoArrivedAtSpecifiedPoint_Flag = 0;
    uprintf("--Chassis is going to run to trace:0\r\n"); // 速度为0
    chassis_status.trace_count = 0;
    break;
  case 17:
    led_control(17);
    Chassis_AutoArrivedAtSpecifiedPoint_Flag = 0;
    uprintf("--Chassis is going to run to trace:1\r\n");
    chassis_status.trace_count = 1;
    break;
  case 18:
    led_control(18);
    Chassis_AutoArrivedAtSpecifiedPoint_Flag = 0;
    uprintf("--Chassis is going to run to trace:2\r\n");
    chassis_status.trace_count = 2;
    break;
  case 19:
    led_control(19);
    Chassis_AutoArrivedAtSpecifiedPoint_Flag = 0;
    uprintf("--Chassis is going to run to trace:3\r\n");
    chassis_status.trace_count = 3;
    break;

  // 十位为2系指令用于踢球动作控制
  case 21: // 设置手动控制踢球
    // 状态机清零
    Kickball2_Kick_Flag = 0;
    Kickball2_Ready_Flag = 0;
    Kickball2_StopRotate_Flag = 0;
    Kickball2_ControlMode = AUTO;
    led_control(21);
    uprintf("--Kickball control mode switch to auto mode.\r\n");
    break;
  case 22: // 设置自动控制踢球
    led_control(22);
    Kickball2_ControlMode = MANUAL;
    uprintf("--Kickball control mode switch to manual mode.\r\n");
    uprintf("  ball num = %d\r\n", Kickball2_BallNum); // 踢第几颗球
    break;
  case 23: // 允许垂直方向位移，可以撤回到5米线
    led_control(23);
    DistanceToBallSocketOK_Flag = 0;
    uprintf("--Handle left rocker recovered\r\n");
    break;
  case 25: // 手动锁定垂直方向的移动，只能水平微调
    led_control(25);
    DistanceToBallSocketOK_Flag = 1;
    uprintf("--Handle: distance to ball socket OK! The left rocker has been locked\r\n");
    break;
  case 26: // 设置踢球状态机为READY状态
    led_control(26);
    if (Kickball2_ControlMode == MANUAL)
    {
      uprintf("##please change to auto mode!##\r\n");
      return;
    }
    Kickball2_Ready_Flag = 1;
    uprintf("--Handle: set Kickball2_Ready_Flag to %d!\r\n", Kickball2_Ready_Flag);
    break;
  case 27: // 设置踢球状态机为KICK状态(手柄无法设置踢球电流，要通过串口助手设置)
    led_control(27);
    if (Kickball2_ControlMode == MANUAL)
    {
      uprintf("##please change to auto mode!##\r\n");
      return;
    }
    Kickball2_Kick_Flag = 1;
    uprintf("--Handle: set Kickball2_Kick_Flag to %d!\r\n", Kickball2_Kick_Flag);
    break;
  case 28: // 清空状态标志位，避免电机疯转
    led_control(28);
    Kickball2_Ready_Flag = 0;
    Kickball2_Kick_Flag = 0;
    kickball2_status = KICKBALL2_NONE;
    vesc.mode = 1;
    vesc.current = 0;
    comm_can_set_current(vesc.id, 0);
    uprintf("--Handle: Kickball flags have been reseted.\r\n");
    break;
  case 29: // 按一下跑一个点
    led_control(29);
    chassis_status.trace_count = (chassis_status.trace_count + 1) % 2;
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

int Handle_LeftRockerAmplitude = 0; // 全局变量，用于告知chassis_move_traces()手柄是否进行了干预
int DistanceToBallSocketOK_Flag = 0;
int Chassis_DimReverse_Flag = 0;   // 坐标轴反转标志
float ROKER_R_ZERO_OFFSET = 0.314; // ±10°内无值，从而让手指可以一直顶着摇杆，避免误操作
int SPEED_TRANSFORM_RATIO = 3;     // 左摇杆控制速度时摇杆幅值与速度的换算比例,可通过按钮切换从而实现不同的速度调节范围
/**
 * @brief 手柄执行函数,左摇杆控制速度矢量，深度控制速度大小；右摇杆控制偏航角，与y轴的偏差角度大小控制角速度
 **/
void handle_exe()
{
  if (0 == flag.main_flag || flag.chassis_handle_flag == 0)
    return;
  // >>>>>>>>>>>>>>>>>>>>>>>>>>速度矢量控制<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 速度为零时，应不读取角度，故将此处的角度先读为temp,
  float temp_fangle = atan2(chassis_handle.ly, chassis_handle.lx);
  if (Chassis_DimReverse_Flag) // 坐标轴反转
  {
    temp_fangle += PI;
  }
  // 加减速用线性变化，此处将速度值设为temp--czh add
  int temp_fspeed = SPEED_TRANSFORM_RATIO * (int)(sqrt(chassis_handle.ly * chassis_handle.ly + chassis_handle.lx * chassis_handle.lx));
  if (temp_fspeed < CHASSIS_HANDLE_MIN_SPEED) // 解决零漂导致的静止速度不为0
  {
    temp_fspeed = 0;
  }
  else if (temp_fspeed > CHASSIS_HANDLE_MAX_SPEED)
  {
    temp_fspeed = CHASSIS_HANDLE_MAX_SPEED;
  }
  else // 线速度大小在允许的范围之内才允许有角速度
  {
    temp_fspeed -= CHASSIS_HANDLE_MIN_SPEED; // 使速度从0开始
    Handle_LeftRockerAmplitude = temp_fspeed;
    chassis.fangle = temp_fangle;
  }
  // 控制加速度
  int fspeed_diff = temp_fspeed - chassis.fspeed;
  if (fspeed_diff > 5)
  {
    chassis.fspeed = chassis.fspeed + 5;
  }
  else if (fspeed_diff < -5)
  {
    chassis.fspeed = chassis.fspeed - 5;
  }
  else
  {
    chassis.fspeed = temp_fspeed;
  }

  // >>>>>>>>>>>>>>>>>>>>>>>>偏航角控制<<<<<<<<<<<<<<<<<<<<<<<<<<
  float temp_fturn = (int)sqrt(chassis_handle.ry * chassis_handle.ry + chassis_handle.rx * chassis_handle.rx);
  if (temp_fturn > 108) // 只有摇杆顶到头才开启转动
  {
    float angle_offset = Angle_Subtract(atan2(chassis_handle.ry, chassis_handle.rx), PI / 2) * (-1);
    if (fabs(angle_offset) < ROKER_R_ZERO_OFFSET)
    {
      angle_offset = 0;
    }
    else if (angle_offset > 0)
    {
      angle_offset -= ROKER_R_ZERO_OFFSET;
    }
    else if (angle_offset < 0)
    {
      angle_offset += ROKER_R_ZERO_OFFSET;
    }

    temp_fturn = 60 * angle_offset;
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

  float delta_theta = 0.122;
  if (DistanceToBallSocketOK_Flag) // 如果与离球座的距离合适，则禁止左摇杆控制垂直方向位移
  {
    if (!(fabs(chassis.fangle) < delta_theta || fabs(chassis.fangle) > PI - delta_theta))
    {
      chassis.fangle = 0;
      chassis.fspeed = 0;
    }
  }
  chassis_move(chassis.fspeed, chassis.fangle, chassis.fturn);
}