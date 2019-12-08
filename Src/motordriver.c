/*******************************************************************************
Copyright:      Bupt
File name:      motordriver.c
Description:    电机控制，设置速度
Author:         19th & ZX & Leo
Version：       1.0
Data:           2019/12/01
*******************************************************************************/
#include "motordriver.h"

#define MAX_CHASSIS_MOTOR_SPEED 630
 
void motor_chassis_speed(int s1,int s2,int s3){
    can_msg can_TX_data[3];
    Limit(s1,MAX_CHASSIS_MOTOR_SPEED);
    Limit(s2,MAX_CHASSIS_MOTOR_SPEED);
    Limit(s3,MAX_CHASSIS_MOTOR_SPEED);
    
    //TODO: 可更改选项，三个电机速度用同一个canid发送

    can_TX_data[0].in[0] = 1;
    can_TX_data[1].in[0] = 1;
    can_TX_data[2].in[0] = 1;
    can_TX_data[0].in[0] = s1;
    can_TX_data[1].in[0] = s2;
    can_TX_data[2].in[0] = s3;
    
    can_send_msg(send_id.motor0_id, &can_TX_data[0]);
    can_send_msg(send_id.motor1_id, &can_TX_data[1]);
    can_send_msg(send_id.motor2_id, &can_TX_data[2]);
}
