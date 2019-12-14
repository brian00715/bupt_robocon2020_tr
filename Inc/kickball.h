#ifndef _KICKBALL_H
#define _KICKBALL_H

#include<vesc_can.h>
#include<robomaster.h>

#define KICK_MOTOR_CAN_ID 88

typedef enum{
    KICKBALL_NONE=0,
    KICKBALL_MAGNET_TO_BOARD,
    KICKBALL_MAGNET_READY,
    KICKBALL_BOARD_TO_DOWN,
    KICKBALL_BOARD_READY,
    KICKBALL_KICK
}KICKBALL_STATUS;


extern float kickball_m2006_current;
extern float kickball_vesc_pull_current;
extern float kickball_vesc_lossen_current;


void kickball_M2006_set_current(float current);
void kickball_VESC_set_loosen_current(float current);
void kickball_VESC_set_loosen_duty(float duty);
void kickball_VESC_set_pull_current(float current);

void kickball_set_magnet_status(int status);
int kickball_get_microswitch_status();
void kickball_set_status(KICKBALL_STATUS status);

void kickball_exe();


#endif //_KICKBALL_H