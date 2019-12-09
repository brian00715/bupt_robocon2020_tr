

#ifndef __CHASSIS_HANDLE_H
#define __CHASSIS_HANDLE_H

#ifdef __cplusplus
extern "C" {
#endif
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "usart.h"
#include "utils.h"
#include "chassis.h"
#include "can.h"
#include "main.h"
#include "math.h"
#include "can_utils.h"
/*Define Area*/
/*Struct Area*/
typedef struct{
  int lx;
  int ly;
  int rx;
  int ry;
  int handle_max_speed;
  int mode;
  int btstate[10];
}Chassis_Handle;
/*Variable Area*/
extern Chassis_Handle chassis_handle;

/*Function Area*/
void handle_button(can_msg *data);
void handle_rocker( can_msg *data);
void handle_exe();



#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_HANDLE_H */