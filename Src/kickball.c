/**
 * @brief 本程序使用循环状态机来控制踢球
 *        基本流程：
 *        1.控制电机使磁铁往上转（m2006收线，本杰明电机放线，同时旋转），
 *          磁铁到位的时候会有机构碰到微动开关，即用微动开关的状态判断磁铁是否转到位。
 *        2.磁铁转到位后开启磁力，吸着踢球板一起往下转，直至到位，使拉着踢球板的弹簧储能
 *        3.磁铁断电，将球踢出
 *        电机配置：
 *        大疆电机m2006（使用配套电调）控制达阵的门和磁铁的旋转
 *        本杰明电调控制牵引线的拉放
 * @version v1.0.0
 * @date 2020.9.7
 **/
//TODO： 状态机可用cmd打断

#include "kickball.h"
#include "sensor_gpio.h"
#include "cmd.h"
#include "motor_driver.h"

KICKBALL_STATUS kickball_status = KICKBALL_NONE; // 踢球状态机的状态表

/****************************电机驱动**************************/

/**M2006踢球电流 单位A current为负时把磁铁往上转*/
void kickball_M2006_set_current(float current)
{
  kick_current = current;
}

/**
 * @brief 设置大电机转速
 **/
void kickball_VESC_set_rpm(float rpm)
{
  vesc.mode = 2;
  vesc.rpm = rpm;
}

/**大电机放线占空比 单位A 负值放线（逆时针旋转）*/
void kickball_VESC_set_loosen_duty(float duty)
{
  if (duty > 0)
    return;
  vesc.mode = 0;
  vesc.duty = duty;
}

/**大电机拉电磁铁电流 正值拉线（顺时针旋转）*/
void kickball_VESC_set_pull_current(float current)
{
  if (current < 0)
    return;
  vesc.mode = 1;
  vesc.current = current;
}

/****************************状态**************************/
/**
 * @brief 设置电磁铁状态
 * @param status 值为1上电
 **/
void kickball_set_magnet_status(int status)
{
  magnet_state = status;
}

/**读微动开关状态 返回1则说明闭合*/
int kickball_get_microswitch_status()
{
  return microswitch_state;
}

/**
 * @brief 设置状态机的状态 
 *        并判断踢球状态机是否按照顺序执行，如果状态机执行步骤跳跃，则报错
*/
void kickball_set_status(KICKBALL_STATUS status)
{
  int state_wrong = 0;
  if (status == KICKBALL_MAGNET_TO_BOARD && kickball_status != KICKBALL_NONE)
    state_wrong = 1;
  if (status == KICKBALL_BOARD_TO_DOWN5 && kickball_status != KICKBALL_MAGNET_READY)
    state_wrong = 1;
  if (status == KICKBALL_BOARD_TO_DOWN10 && kickball_status != KICKBALL_MAGNET_READY)
    state_wrong = 1;
  if (status == KICKBALL_BOARD_TO_DOWN20 && kickball_status != KICKBALL_MAGNET_READY)
    state_wrong = 1;
  if (status == KICKBALL_KICK && kickball_status != KICKBALL_BOARD_READY)
    state_wrong = 1;
  if (state_wrong == 1)
  {
    uprintf("Kickball state switch wrong!\r\n");
    return;
  }
  kickball_status = status;
}

/****************************外部调用**************************/

float kickball_m2006_current = -5;       //-5   // 控制大疆电机将磁铁转到踢球版的电流大小
float kickball_vesc_lossen_duty = -0.35; // duty为负值时放线
float kickball_vesc_pull_current5 = 6;
float kickball_vesc_pull_current10 = 5;
float kickball_vesc_pull_current20 = 5;
// 外部调用flag
int kickball_prepare_flag = 0; //cmd控制，置1时开始动作：电磁铁转到踢球版上，到位后上电
int kickball_ready_flag = 0;   //内部置1 反馈给主控
int kickball_kick_flag = 0;    //cmd控制，置1执行踢球动作：电磁铁断电
// 内部调试flag
int kickball_pull_magnet_flag = 0; //cmd控制，置1开始拉电磁铁
int kickball_stop_magnet_flag = 0; //cmd控制，置1停止拉电磁铁
// 五球分数
int kickball_five_score[5] = {20, 20, 20, 20, 20};
// 当前第几颗球
int kickball_num = 0;

/**
 * @brief 踢球状态机，使用自动模式时在main中循环执行
 **/
void kickball_state_machine()
{
  switch (kickball_status)
  {
  case KICKBALL_NONE:
  {
    if (kickball_prepare_flag == 1)
    {                                                // 外部启动 ready_flag
      kickball_num++;                                // 执行下一个状态
      kickball_num %= 5;                             // 控制状态循环
      kickball_set_status(KICKBALL_MAGNET_TO_BOARD); // 执行下一个状态：把磁铁转上去
      kickball_prepare_flag = 0;
    }
    break;
  }

  case KICKBALL_MAGNET_TO_BOARD:
  {
    kickball_M2006_set_current(kickball_m2006_current);       // 磁铁往上转
    kickball_VESC_set_loosen_duty(kickball_vesc_lossen_duty); // 本杰明电调放线（占空比为负值，逆时针旋转）
    if (kickball_get_microswitch_status() == 1)               //闭环检测微动开关flag
    {
      kickball_set_magnet_status(1);              // 磁铁上电
      kickball_VESC_set_loosen_duty(0);           // 本杰明电机断电
      kickball_M2006_set_current(0);              // m2006断电
      kickball_set_status(KICKBALL_MAGNET_READY); // 执行下一个状态：准备踢球
    }
    break;
  }

  case KICKBALL_MAGNET_READY:
  {
    // 由于磁铁吸附没有反馈，因此需要人眼观察是否吸上，
    // cmd发送cmd_kickball_pull_magnet启动拉电磁铁
    if (kickball_pull_magnet_flag == 1)
    {
      if (kickball_five_score[kickball_num] == 5)
        kickball_set_status(KICKBALL_BOARD_TO_DOWN5);
      if (kickball_five_score[kickball_num] == 10)
        kickball_set_status(KICKBALL_BOARD_TO_DOWN10);
      if (kickball_five_score[kickball_num] == 20)
        kickball_set_status(KICKBALL_BOARD_TO_DOWN20);
      kickball_pull_magnet_flag = 0;
    }
    break;
  }

  case KICKBALL_BOARD_TO_DOWN5: // 5分球
  {
    kickball_VESC_set_pull_current(kickball_vesc_pull_current5);
    //TODO： 人眼闭环 需要改为编码器闭环，或者电流环
    if (kickball_stop_magnet_flag == 1)
    {
      kickball_set_status(KICKBALL_BOARD_READY);
      kickball_stop_magnet_flag = 0;
    }
    break;
  }

  case KICKBALL_BOARD_TO_DOWN10: // 10分球
  {
    //TODO： 人眼闭环 需要改为编码器闭环，或者电流环
    kickball_VESC_set_pull_current(kickball_vesc_pull_current10);
    if (kickball_stop_magnet_flag == 1)
    {
      kickball_set_status(KICKBALL_BOARD_READY);
      kickball_stop_magnet_flag = 0;
    }
    break;
  }

  case KICKBALL_BOARD_TO_DOWN20: // 20分球
  {
    //TODO： 人眼闭环 需要改为编码器闭环，或者电流环
    kickball_VESC_set_pull_current(kickball_vesc_pull_current20);
    if (kickball_stop_magnet_flag == 1)
    {
      // cmd决定何时停止
      kickball_set_status(KICKBALL_BOARD_READY);
      kickball_stop_magnet_flag = 0;
    }
    break;
  }

  case KICKBALL_BOARD_READY: // 踢球板就位
  {
    kickball_VESC_set_rpm(0);
    kickball_ready_flag = 1;
    if (kickball_kick_flag == 1)
    {
      // cmd决定何时踢球
      kickball_set_status(KICKBALL_KICK);
      kickball_kick_flag = 0;
      kickball_ready_flag = 0;
    }
    break;
  }

  case KICKBALL_KICK:
  {
    kickball_set_magnet_status(0);      // 磁铁断电，踢球版在弹簧的作用下被弹出
    kickball_VESC_set_pull_current(0);  // 本杰明电机断电
    kickball_set_status(KICKBALL_NONE); // 控制状态机循环
    break;
  }
  }
}

int kickball_auto = 1; // 踢球控制模式 0 手动 1 自动
void kickball_exe()
{
  if (kickball_auto == 1)
    kickball_state_machine(); // 自动控制则启动状态机
  else
  {
    if (kickball_get_microswitch_status() == 1) // 闭环检测微动开关flag，微动开关被碰到返回1
    {
      kickball_set_magnet_status(1);    // 电磁铁上电
      kickball_VESC_set_loosen_duty(0); // 电机停转
      kickball_M2006_set_current(0);    // 电机停转
    }
  }
}
