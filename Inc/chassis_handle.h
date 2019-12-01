/*******************************************************************************
Copyright:      BUPT
File name:      chassis_handle.c
Description:    主要放手柄的控制程序
方便封装
Author:         ZH
Version：       1.0
Data:           2019/10/21
History:        none
*******************************************************************************/

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
}CHASSIS_HANDLE;
/*Variable Area*/
extern CHASSIS_HANDLE ChassisHandle;

/*Function Area*/
void chassis_handle_button(can_msg *data);
void chassis_rocker( can_msg *data);
void chassis_handle_control();



#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_HANDLE_H */