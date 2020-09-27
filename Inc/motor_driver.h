#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H
#include "can_utils.h"

void motor_init();
//vesc
typedef struct
{
    uint8_t id;     // 本杰明电调的can消息id
    int mode;       // 模式（速度环、电流环等）
    float duty;     // 占空比
    float current;  // 电流
    float rpm;      // 转速
    float position; // 位置
} VESC_STATE;

extern VESC_STATE vesc;
void vesc_exe();
//m2006
extern float touchdown_current;
extern float MoterDriver_M2006_Current;
void m2006_exe();
void motor_init();

#endif //_MOTOR_DRIVER_H