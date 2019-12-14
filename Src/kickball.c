#include "kickball.h"
#include "sensor_gpio.h"

static KICKBALL_STATUS kickball_status = 0;
float kickball_m2006_current=-4;
float kickball_vesc_pull_current=0;
float kickball_vesc_lossen_current=0;

/****************************电机驱动**************************/

/**M2006踢球电流 单位A 负时拉电磁铁*/
void kickball_M2006_set_current(float current){
  kick_current = current;
}
/**VESC放线电流 单位A 负时放线*/
void kickball_VESC_set_loosen_current(float current){
  if (current > 0)     return; //松线是电机反转，其值应为负
  comm_can_set_current(KICK_MOTOR_CAN_ID, current); //松线用速度环可能会稳定一些
}
/**VESC放线占空比*/
void kickball_VESC_set_loosen_duty(float duty){
  if (duty > 0)     return;
  comm_can_set_duty(KICK_MOTOR_CAN_ID, duty);//vesc
}
/**VESC拉电磁铁电流 正时拉线*/
void kickball_VESC_set_pull_current(float current){
  if (current < 0)    return;
  comm_can_set_current(KICK_MOTOR_CAN_ID, current); //拉线用电流环可能会稳定一些
}
/****************************状态**************************/

/**设置电磁铁状态 1 上电*/
void kickball_set_magnet_status(int status){
  magnet_state=status;
}
/**读微动开关状态 1 闭合*/
int kickball_get_microswitch_status(){
  return microswitch_state;
}
/**设置踢球状态*/
void kickball_set_status(KICKBALL_STATUS status){
  if (status == KICKBALL_MAGNET_TO_BOARD && kickball_status!=KICKBALL_NONE)
    return;
  if (status == KICKBALL_BOARD_TO_DOWN && kickball_status!=KICKBALL_MAGNET_READY)
    return;
  if(status == KICKBALL_KICK &&kickball_status!=KICKBALL_BOARD_READY)
    return;
  kickball_status = status; 
}

/****************************外部调用**************************/
void kickball_exe(){
 switch (kickball_status){
  case KICKBALL_NONE:  {
    //do nothing
    break;}

  case KICKBALL_MAGNET_TO_BOARD:{
    kickball_set_magnet_status(1);
    kickball_M2006_set_current(kickball_m2006_current);
    kickball_VESC_set_loosen_current(kickball_vesc_lossen_current);
    if( kickball_get_microswitch_status() == 1){
      // kickball_set_magnet_status(1);
      kickball_VESC_set_loosen_current(0);
      kickball_M2006_set_current(0);
      kickball_status = KICKBALL_MAGNET_READY;      
    }
    break;}

  case KICKBALL_MAGNET_READY:{
    //do nothing
    break;}

  case KICKBALL_BOARD_TO_DOWN:{
    kickball_VESC_set_pull_current(kickball_vesc_pull_current);
    //if()  //得加一个到达预定位置的反馈，来进行状态转换
    //  kickball_status = KICKBALL_BOARD_READY;
    break;}

  case KICKBALL_BOARD_READY:{
    comm_can_set_rpm(KICK_MOTOR_CAN_ID,0);
    break;}
    
  case KICKBALL_KICK:{
    kickball_set_magnet_status(0);
    kickball_VESC_set_pull_current(0);
    kickball_status = KICKBALL_NONE;
    break;}
  }
}
