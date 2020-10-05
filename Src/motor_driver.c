#include "motor_driver.h"

/**
 * @brief 设定本杰明电调的canid为88
 **/
void motor_init()
{
  vesc.id = 88;
}

//vesc
VESC_STATE vesc = {0}; // 本杰明电调的状态机状态表

/**
 * @brief 本杰明电调的执行函数，从cmd获取数据
 *        根据vesc结构体的mode执行相应功能
 *        mode0 占空比
 *        mode1 电流环
 *        mode2 速度环
 *        mode3 位置环（通常不用）
 **/
void vesc_exe()
{
  if (flag.vesc_flag == 0)
  {
    return;
  }
  VESC_StopByAngle(); // 旋转到指定角度后停止
  if (vesc.mode == 0)
  {
    comm_can_set_duty(vesc.id, vesc.duty);
  }
  if (vesc.mode == 1)
  {
    comm_can_set_current(vesc.id, vesc.current);
  }
  if (vesc.mode == 2)
  {
    comm_can_set_rpm(vesc.id, vesc.rpm);
  }
  if (vesc.mode == 3)
    comm_can_set_pos(vesc.id, vesc.position); //位置环
  flag.vesc_flag = 0;
}

// float touchdown_current = 0; // 达阵电流,单位毫安
float MoterDriver_M2006_Current = 0; // 踢球电流
/**
 * @brief m2006大疆电机的执行函数,从cmd获取目标电流
 *        已弃用达阵，转踢球板的为1号电调
 **/
void m2006_exe()
{
  if (flag.m2006_flag == 0) // 控制5ms发一次
    return;
  int16_t kick_I = (int16_t)(MoterDriver_M2006_Current * 1000);
  robomaster_set_current(kick_I, 0, 0, 0);
  // Robomaster_StopByAngle(0);
  flag.m2006_flag = 0;
}

int VESC_StatusBag_Flag = 0;
int16_t VESC_CurrentAngle = 0; // 编码器反馈的角度值
int32_t VESC_CurrentRPM = 0;
int VESC_SwitchPrintInfo_Flag = 0;
/**
 * @brief 本杰明电调反馈状态包解析函数
 **/
void VESC_RX_Handle(can_msg *data)
{
  if (0 == flag.main_flag)
    return;
  int32_t index = 0;
  VESC_CurrentAngle = buffer_get_int16(data->ui8, &index) / 50;
  VESC_CurrentRPM = buffer_get_int32(data->ui8,&index);
}

int16_t VESC_TargetAngle = 300;
int VESC_SwitchStopByAngle_Flag = 0;
float pre_current = 0;
/**
 * @brief 旋转到指定角度后停止
 **/
void VESC_StopByAngle()
{
  if (VESC_SwitchStopByAngle_Flag == 0 )
  {
    return;
  }
  pre_current = vesc.current;
  if (abs(VESC_CurrentAngle - VESC_TargetAngle) < 10)
  {
    vesc.mode = 1;
    vesc.current = 0;
    VESC_SwitchStopByAngle_Flag = 0;
  }
  // else
  // {
  //   vesc.mode = 1;
  //   vesc.current = pre_current;
  // }
}

void VESC_PrintInfo()
{
  if (VESC_SwitchPrintInfo_Flag)
  {
    uprintf("--vesc status>>");
    uprintf(" current_angle: %d target_angle:%d mode:%d ", VESC_CurrentAngle, VESC_TargetAngle,vesc.mode);
    switch (vesc.mode)
    {
    case 0:
      uprintf("duty:%.2f\r\n", vesc.duty);
      break;
    case 1:
      uprintf("current:%.2f\r\n", vesc.current);
      break;
    case 2:
      uprintf("rpm:%.2f\r\n", vesc.rpm);
      break;
    case 3:
      uprintf("pos:%.2f\r\n", vesc.position);
      break;
    default:
      uprintf("##Vesc Mode Error!##\r\n");
      break;
    }
  }
}
