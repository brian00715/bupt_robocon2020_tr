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
#include "configure.h"
#include "utils.h"

void motor_chassis_speed(int s1,int s2,int s0);

#ifdef __cplusplus
}
#endif
#endif /*__ maxon_H */