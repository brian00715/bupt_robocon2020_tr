/*******************************************************************************
Copyright:      Bupt
File name:      vega.c
Description:    action全场定位，使用前初始化十秒
Author:         Leo
Version：       1.0
Data:           2019/11/24
*******************************************************************************/
#include "vega.h"
#include "cmd.h"
#include "chassis.h"
#include "configure.h"
#include "utils.h"
//TODO: vega的赋值校准
/**vega校准*/
void vega_calibration(){
  for(int i = 0;i<=20;i++){
    uprintf_to(&VEGA_USART,"ACTR");
  }
  uprintf("send vega calibrate message\r\n");
}
/**vega复位*/
void vega_reset(){
  for(int i = 0;i<=20;i++){
    uprintf_to(&VEGA_USART,"ACT0");
  }
  uprintf("send vega reset message\r\n");
}

//TODO: 全场定位解算到底盘中心
//TODO: 需要修改vega(vega's data),vega_position(vega 位置)
float vega_position[3]={-1,1,PI/4};
/**vega相对坐标原点位置设置*/
void vega_set_position(float Dx ,float Dy ,float Dangle ){
  vega_position[0]=Dx;
  vega_position[1]=Dy;
  vega_position[2]=Dangle;
  uprintf("Vega_position:\r\nx:%5f y:%5f angle:%5f\r\n",vega_position[0],vega_position[1],vega_position[2]);
}
/**vega坐标系变换*/
void vega_coordinate(float pos[3]){
  float vega[3];
  vega[0]=1;
  vega[1]=1;
  vega[2]=PI;
  Coordinate_System_Transform(vega,vega_position,pos);
}
/**vage坐标打印*/
void vega_print_pos(){
  if(chassis_status.vega_is_ready ==1){
      uprintf("Vega:\r\nx:%5f y:%5f angle:%5f\r\n",chassis.vega_pos_x,chassis.vega_pos_y,chassis.vega_angle);}
  else{
      uprintf("vega has not inited\r\n");} 
}