#include "main.h"
#include "robomaster.h"
#include "vesc_can.h"
#include "kickball.h"
#include "motor_driver.h"

void motor_init()
{
  vesc.id = 88;
  // vesc.id=88;
}

//vesc
VESC_STATE vesc = {0}; // 本杰明电调的状态机状态表

/**
 * @brief 本杰明电调的执行函数
 *        根据vesc结构体的mode执行相应功能
 *        mode0 占空比
 *        mode1 电流环
 *        mode2 速度环
 *        mode3 位置环（通常不用）
 **/
void vsec_exe()
{
  flag.vesc_flag = 0;
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
    ; //位置环
}

float touchdown_current = 0; // 达阵电流
float kick_current = 0;      // 踢球电流
/**
 * @brief m2006电机的执行函数
 **/
void m2006_exe()
{
  if (flag.m2006_flag == 0)
    return;
  int16_t door_I = (int16_t)(touchdown_current * 1000);
  int16_t kick_I = (int16_t)(kick_current * 1000);
  robomaster_set_current(door_I, kick_I, 0, 0);
  flag.m2006_flag = 0;
}
