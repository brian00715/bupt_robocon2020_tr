#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H
#include "can_utils.h"  

void motor_init();
//vesc
typedef struct{
    uint8_t id;
    int mode;    
    float duty;
    float current;
    float rpm;
    float position;
} VESC_STATE;
extern VESC_STATE vesc;
void vsec_exe();
//m2006
extern volatile float touchdown_current;
extern float kick_current;
void m2006_exe();
void motor_init();


#endif //_MOTOR_DRIVER_H