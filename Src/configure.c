/*******************************************************************************
Copyright:      BUPT
File name:      configure.c
Description:    所有的配置外部配置
Author:         ZX 
Version：       1.0
Data:           2019/10/9
*******************************************************************************/
#include "configure.h"

Can_id_send send_id;
Can_id_recive recive_id;

void can_id_init()
{
    send_id.motor0_id = 100;
    send_id.motor1_id = 101;
    send_id.motor2_id = 102;  
    send_id.lcd_id = 1;
    
    recive_id.handle_button_id = 325;
    recive_id.handle_rocker_id = 324;
    recive_id.lcd_id = 2;
    recive_id.pos_id = 88;    //自己的全场定位
}


