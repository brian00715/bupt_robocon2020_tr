/*******************************************************************************
Copyright:      Bupt
File name:      motordriver.c
Description:    电机控制，设置速度
Author:         19th & ZX & Leo
Version：       1.0
Data:           2019/12/01
*******************************************************************************/
#include "motordriver.h"
#include "configure.h"
#include "utils.h"

can_msg can_TX_data;

void motor_canset3speed(int s1,int s2,int s3)
{
    Limit(s1,MAX_MOTOR_SPEED);
    Limit(s2,MAX_MOTOR_SPEED);
    Limit(s3,MAX_MOTOR_SPEED);
    //TODO why
    
    if(s1 < 0) s1 = MAX_MOTOR_SPEED - s1;
    if(s2 < 0) s2 = MAX_MOTOR_SPEED - s2;
    if(s3 < 0) s3 = MAX_MOTOR_SPEED - s3;
    
    can_TX_data.ui16[0] = (uint16_t)s1;
    can_TX_data.ui16[1] = (uint16_t)s2;
    can_TX_data.ui16[2] = (uint16_t)s3;
    
    can_send_msg(send_id.motor0_id, &can_TX_data);
}
