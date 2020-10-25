/*******************************************************************************
Copyright:      BUPT
File name:      configure.c
Description:    所有的配置，例如整合chassis等需要用到的结构体等，目的是简化后续代码和
方便封装
Author:         ZX 
Version：       1.0
Data:           2019/10/9
History:        none
*******************************************************************************/
#ifndef __CONFIGURE_H
#define __CONFIGURE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdlib.h>
#include <math.h>

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "utils.h"
#include "chassis.h"

    /*Struct Area*/
    typedef struct
    {
        uint32_t motor0_id; //电机id
        uint32_t motor1_id;
        uint32_t motor2_id;
        uint32_t lcd_id;
        uint32_t led_id;
    } Can_id_send;

    typedef struct can_id_recive
    {
        uint32_t handle_button_id; //手柄按键
        uint32_t handle_rocker_id; //手柄摇杆
        uint32_t pos_id;
        uint32_t lcd_id;
    } Can_id_recive;

    /*Function Area*/
    void can_id_init();
    /*Variable Area*/
    extern Can_id_send send_id;
    extern Can_id_recive recive_id;

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLELIB_H */