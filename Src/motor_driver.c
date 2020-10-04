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
  // flag.vesc_flag = 0;
  if (flag.vesc_flag == 0)
  {
    return;
  }
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

int16_t VESC_Angle = 0;  // 编码器反馈的角度值
/**
 * @brief 本杰明电调反馈状态包解析函数
 **/
void VESC_RX_Handle(can_msg *data)
{
  if (0 == flag.main_flag)
    return; 
  int32_t index = 0;
  VESC_Angle = buffer_get_int16(data->ui8,&index)/50;
  uprintf("--vesc angle: %d\r\n",VESC_Angle);

}
