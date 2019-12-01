#ifndef __maxon_H
#define __maxon_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx.h"
#include "main.h"
#include "usart.h"
#include "can.h"
#include "simplelib.h"

#define MAX_MOTOR_SPEED 630

void motor_canset3speed(int s1,int s2,int s0);

#ifdef __cplusplus
}
#endif
#endif /*__ maxon_H */