#include "kickball.h"
#include "sensor_gpio.h"
#include "cmd.h"
#include "motor_driver.h"

KICKBALL_STATUS kickball_status = KICKBALL_NONE;

/****************************电机驱动**************************/
/**M2006踢球电流 单位A 负时拉电磁铁*/
void kickball_M2006_set_current(float current){
  kick_current = current;
}

/**VESC rpm*/
void kickball_VESC_set_rpm(float rpm){
  vesc.mode = 2;
  vesc.rpm = rpm;
}
/**VESC放线占空比 单位A 负时放线*/
void kickball_VESC_set_loosen_duty(float duty){
  if (duty > 0)     return;
  vesc.mode = 0;
  vesc.duty = duty;
}
/**VESC拉电磁铁电流 正时拉线*/
void kickball_VESC_set_pull_current(float current){
  if (current < 0)    return;
  vesc.mode = 1;
  vesc.current = current; 
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
  int state_wrong = 0;
  if (status == KICKBALL_MAGNET_TO_BOARD && kickball_status!=KICKBALL_NONE) state_wrong = 1;
  if (status == KICKBALL_BOARD_TO_DOWN5 && kickball_status!=KICKBALL_MAGNET_READY) state_wrong = 1;
  if (status == KICKBALL_BOARD_TO_DOWN10 && kickball_status!=KICKBALL_MAGNET_READY) state_wrong = 1;
  if (status == KICKBALL_BOARD_TO_DOWN20 && kickball_status!=KICKBALL_MAGNET_READY) state_wrong = 1;
  if(status == KICKBALL_KICK &&kickball_status!=KICKBALL_BOARD_READY) state_wrong = 1;
  if(state_wrong == 1){
    uprintf("Kickball state switch wrong!\r\n");
    return;}
  kickball_status = status;   
}
/****************************外部调用**************************/


float kickball_m2006_current =-5;
float kickball_vesc_lossen_duty =-0.15; 
float kickball_vesc_pull_current5 = 1.2;
float kickball_vesc_pull_current10 = 1.2;
float kickball_vesc_pull_current20 = 1.2;
//外部调用flag
int kickball_prepare_flag= 0;      //外部置1 踢球准备
int kickball_ready_flag= 0;        //内部置1 反馈给主控
int kickball_kick_flag= 0;         //外部置1 执行踢球
//内部调试flag
int kickball_pull_magnet_flag=0;   //cmd指令置1 开始拉电磁铁
int kickball_stop_magnet_flag=0;   //cmd指令置1 停止拉电磁铁            
//五球分数
int kickball_five_score[5]={20,20,20,20,20};
//当前第几颗球
int kickball_num=0;


void kickball_state_machine(){
  switch (kickball_status){
  case KICKBALL_NONE:  {
    if(kickball_prepare_flag == 1){                           //外部启动 ready_flag
      kickball_num ++ ;
      kickball_num %= 5;
      kickball_set_status(KICKBALL_MAGNET_TO_BOARD);
      kickball_prepare_flag = 0;
    }
    break;}

  case KICKBALL_MAGNET_TO_BOARD:{
    kickball_M2006_set_current(kickball_m2006_current);
    kickball_VESC_set_loosen_duty(kickball_vesc_lossen_duty);
    if( kickball_get_microswitch_status() == 1){               //闭环检测微动开关flag
      kickball_set_magnet_status(1);
      kickball_VESC_set_loosen_duty(0);
      kickball_M2006_set_current(0);
      kickball_set_status(KICKBALL_MAGNET_READY);      
    }
    break;}

  case KICKBALL_MAGNET_READY:{
    if(kickball_pull_magnet_flag == 1){                                   //人眼观察是否吸上，cmd启动拉电磁铁
    if(kickball_five_score[kickball_num]==5) kickball_set_status(KICKBALL_BOARD_TO_DOWN5);
    if(kickball_five_score[kickball_num]==10) kickball_set_status(KICKBALL_BOARD_TO_DOWN10);
    if(kickball_five_score[kickball_num]==20) kickball_set_status(KICKBALL_BOARD_TO_DOWN20);
    kickball_pull_magnet_flag = 0;
    }
    break;}

  case KICKBALL_BOARD_TO_DOWN5:{
    kickball_VESC_set_pull_current(kickball_vesc_pull_current5);
    //TODO： 人眼闭环 需要改为编码器闭环，或者电流环
    if(kickball_stop_magnet_flag == 1){  
    kickball_set_status(KICKBALL_BOARD_READY);
    kickball_stop_magnet_flag = 0;
    }
    break;}

  case KICKBALL_BOARD_TO_DOWN10:{
    //TODO： 人眼闭环 需要改为编码器闭环，或者电流环
    kickball_VESC_set_pull_current(kickball_vesc_pull_current10);
    if(kickball_stop_magnet_flag == 1){  
    kickball_set_status(KICKBALL_BOARD_READY);
    kickball_stop_magnet_flag = 0;
    }
    break;}

  case KICKBALL_BOARD_TO_DOWN20:{
    //TODO： 人眼闭环 需要改为编码器闭环，或者电流环
    kickball_VESC_set_pull_current(kickball_vesc_pull_current20);
    if(kickball_stop_magnet_flag == 1){                                   //cmd决定何时停止
    kickball_set_status(KICKBALL_BOARD_READY);
    kickball_stop_magnet_flag = 0;
    }
    break;}

  case KICKBALL_BOARD_READY:{
    kickball_VESC_set_rpm(0);
    kickball_ready_flag = 1;
    if(kickball_kick_flag == 1){                                         //cmd决定何时踢球
      kickball_set_status(KICKBALL_KICK);
      kickball_kick_flag = 0;
      kickball_ready_flag = 0;
    }
    break;}
    
  case KICKBALL_KICK:{
    kickball_set_magnet_status(0);
    kickball_VESC_set_pull_current(0);
    kickball_set_status(KICKBALL_NONE);
    break;}
  }
}


int kickball_auto = 1; 


void kickball_exe(){
 if(kickball_auto == 1) kickball_state_machine();
 else{
   if( kickball_get_microswitch_status() == 1){               //闭环检测微动开关flag
      kickball_set_magnet_status(1);
      kickball_VESC_set_loosen_duty(0);
      kickball_M2006_set_current(0);
   }
 }
}
 