/**
 * @brief 本程序使用循环状态机来控制踢球
 *        >>第一代踢球装置基本流程<<
 *        1.控制电机使磁铁往上转（m2006收线，本杰明电机放线，同时旋转），
 *          磁铁到位的时候会有机构碰到微动开关，即用微动开关的状态判断磁铁是否转到位。
 *        2.磁铁转到位后开启磁力，吸着踢球板一起往下转，直至到位，使拉着踢球板的弹簧储能
 *        3.磁铁断电，将球踢出
 *        >>第二代踢球装置基本流程<<
 *        1.控制本杰明电机旋转踢球柱至合适的位置，让弹簧蓄力。
 *        2.超过该位置后弹簧
 *        >>电机配置<<
 *        大疆电机m2006（使用配套电调）控制达阵的门和磁铁的旋转
 *        本杰明电调控制牵引线的拉放
 * @version v2.0.0
 * @date 2020.10.3
 **/

#include "kickball.h"
#include "led.h"

//===============================第一代踢球===============================
/*M2006踢球电流 单位A current为负时把磁铁往上转*/
static inline void kickball_M2006_set_current(float current)
{
  MoterDriver_M2006_Current = current;
}

/*设置大电机转速*/
static inline void kickball_VESC_set_rpm(float rpm)
{
  vesc.mode = 2;
  vesc.rpm = rpm;
}

/**大电机放线占空比 单位A 正值放线*/
static inline void kickball_VESC_set_loosen_duty(float duty)
{
  if (duty < 0)
    return;
  vesc.mode = 0;
  vesc.duty = duty;
}

/**大电机拉电磁铁电流 负值收线*/
static inline void kickball_VESC_set_pull_current(float current)
{
  if (current > 0)
    return;
  vesc.mode = 1;
  vesc.current = current;
}

/****************************IO端口状态**************************/
/**
 * @brief 设置电磁铁状态
 * @param status 值为1上电
 **/
void kickball_set_magnet_status(int status)
{
  magnet_state = status;
}

/*读微动开关状态 返回1则说明闭合*/
int kickball_get_microswitch_status()
{
  return microswitch_state;
}

/*设置状态机的状态,并判断踢球状态机是否按照顺序执行，
如果状态机执行步骤跳跃，则报错*/
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
//vesc正值放线，负值收线;2006正值把踢球版往上转，负值往下转
float kickball_m2006_current = 5.3;      // 控制大疆电机将磁铁转到踢球版的电流大小
float kickball_vesc_lossen_duty = 1;     // duty为正时放线
float kickball_vesc_pull_current5 = -5;  // 5分球电流
float kickball_vesc_pull_current10 = -5; // 10分球电流
float kickball_vesc_pull_current20 = -5; // 20分球电流
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
KICKBALL_STATUS kickball_status = KICKBALL_NONE;
CONTROL_MODE Kickball_ControlMode = AUTO; // 踢球控制模式 0 手动 1 自动
/*踢球状态机，使用自动模式时在main中循环执行*/
void kickball_state_machine()
{
  switch (kickball_status)
  {
  case KICKBALL_NONE:
  {
    if (kickball_prepare_flag == 1)
    {                                                // cmd启动 ready_flag
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
    if (kickball_get_microswitch_status() == 1)               // 闭环检测微动开关flag
    {
      kickball_set_magnet_status(1);              // 磁铁上电
      kickball_VESC_set_loosen_duty(0);           // 本杰明电机断电
      kickball_M2006_set_current(0);              // m2006断电
      kickball_set_status(KICKBALL_MAGNET_READY); // 执行下一个状态：下拉电磁铁
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
    kickball_VESC_set_pull_current(kickball_vesc_pull_current5); // 开始拉电磁铁
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

  case KICKBALL_BOARD_READY: // 停止收线，踢球板就位
  {
    comm_can_set_rpm(vesc.id, 0); // 立即执行
    kickball_VESC_set_rpm(0);     // 设置参数为0
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

void kickball_exe()
{
  if (Kickball_ControlMode == AUTO)
    kickball_state_machine(); // 自动控制则启动状态机
  else
  {
    if (kickball_get_microswitch_status() == 1) // 闭环检测微动开关flag，微动开关被碰到返回1
    {
      kickball_set_magnet_status(1);      // 电磁铁上电
      comm_can_set_rpm(vesc.id, 0);       // 立即执行
      robomaster_set_current(0, 0, 0, 0); // 立即执行
      kickball_VESC_set_loosen_duty(0);   // 电机停转
      kickball_M2006_set_current(0);      // 电机停转
    }
  }
}
//===============================第一代踢球END===============================

//===============================第二代踢球==================================
#if KICKBALL_GEN == 2
KICKBALL2_STATUS kickball2_status = KICKBALL2_NONE;
CONTROL_MODE Kickball2_ControlMode = AUTO;
int16_t Kickball2_StopAngle = 45;               // 需要让电机停电时的角度
int16_t Kickball2_SpringRawAngle = 255;         // 弹簧原长对应的角度
int16_t Kickball2_SpringAutoRecoverAngle = 236; // 弹簧能自动拉回的角度
float Kickball2_KickCurrent = -6.5;             // CMD设置或使用默认值-5
int Kickball2_Ready_Flag = 0;                   // 由全场定位置1，或使用CMD
int Kickball2_Kick_Flag = 0;                    // 由CMD置1
int Kickball2_StopRotate_Flag = 0;              // 根据编码器角度值来确定,或使用CMD
int Kickball2_BallNum = -1;
void Kickball2_StateMachine()
{
  switch (kickball2_status)
  {
  case KICKBALL2_NONE:
    // led_control(30);
    if (Kickball2_Ready_Flag)
    {
      Kickball2_Ready_Flag = 0;
      uprintf("--StateMachine: chassis ready.\r\n");
      Kickball2_SetState(KICKBALL2_READY);
    }
    break;

  case KICKBALL2_READY: // 底盘位置就位
    // led_control(31);
    if (Kickball2_Kick_Flag)
    {
      Kickball2_Kick_Flag = 0;
      uprintf("--StateMachine: start kicking the ball!\r\n");
      VESC_SwitchStopByAngle_Flag = 1;
      VESC_TargetAngle = Kickball2_StopAngle;
      Kickball2_SetState(KICKBALL2_KICK);
    }
    break;

  case KICKBALL2_KICK: // 开始踢球
    // led_control(32);
    vesc.mode = 1;
    vesc.current = Kickball2_KickCurrent;
    comm_can_set_current(vesc.id, Kickball2_KickCurrent);
    chassis_canset_motorduty(0, 0, 0); // 底盘锁死

    if (VESC_SwitchStopByAngle_Flag == 0) // 由StopByAngle（）函数置0
    {
      vesc.mode = 1;
      vesc.current = 0; // 电流置0即可，弹簧会把踢球柱拉回去
      comm_can_set_current(vesc.id, 0);
      // uprintf("--StateMachine: kick ball finished.\r\n");
      // Kickball2_SetState(KICKBALL2_NONE);

      if (VESC_CurrentAngle - Kickball2_SpringAutoRecoverAngle > -5)
      {
        uprintf("--StateMachine: kick ball finished.\r\n");
        uprintf("  StateMachine: start setting spring to raw length.\r\n");
        VESC_SwitchStopByAngle_Flag = 1;
        VESC_TargetAngle = Kickball2_SpringRawAngle;
        Kickball2_SetState(KICKBALL2_SET_SPRING_RAW);
      }
    }
    break;

  case KICKBALL2_SET_SPRING_RAW: // 弹簧返回原长
    vesc.mode = 0;
    vesc.duty = 0.4;

    if (VESC_SwitchStopByAngle_Flag == 0) // 由StopByAngle()置0
    {
      vesc.mode = 1;
      vesc.current = 0; // 为了使弹簧缩回原长，需要立即停止
      uprintf("--StateMachine: spring has set to raw length.\r\n");
      Kickball2_SetState(KICKBALL2_NONE);
    }
    break;

  default:
    uprintf("##StateMachineError!##\r\n");
    break;
  }
}

void Kickball2_SetState(KICKBALL2_STATUS status)
{
  int state_wrong = 0;
  if (status == KICKBALL2_READY && kickball2_status != KICKBALL2_NONE)
    state_wrong = 1;
  if (status == KICKBALL2_KICK && kickball2_status != KICKBALL2_READY)
    state_wrong = 1;
  if (status == KICKBALL2_SET_SPRING_RAW && kickball2_status != KICKBALL2_KICK)
    state_wrong = 1;
  if (status == KICKBALL2_NONE && kickball2_status != KICKBALL2_SET_SPRING_RAW)
    state_wrong = 1;
  // if (status == KICKBALL2_NONE && kickball2_status != KICKBALL2_KICK)
  //   state_wrong = 1;

  if (state_wrong == 1)
  {
    uprintf("##Kickball state switch wrong!##\r\n");
    return;
  }
  kickball2_status = status;
}

void Kickball2_EXE()
{
  if (Kickball2_ControlMode == AUTO)
    Kickball2_StateMachine(); // 自动控制则启动状态机
  else
  {
    // 手动模式下直接给本杰明电调发CAN消息实现控制
  }
}
#endif
