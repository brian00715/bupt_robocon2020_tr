/*******************************************************************************
Copyright:      Bupt
File name:      vega.c
Description:    action全场定位，使用前初始化十秒
Author:         Leo
Version：       1.0
Data:           2019/11/24
*******************************************************************************/
#include "string.h"
#include "vega.h"
#include "cmd.h"
#include "chassis.h"
#include "configure.h"
#include "utils.h"

//!:全场定位的坐标系变换---数据：上电时刻东大全场定位到坐标原点pos;东大全场定位坐标系定位点 pos
float vega_in_coordinate[3]={-1,1,PI/4};
float center_in_vega[3]={1,1,1};

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
/**vega相对坐标原点位置设置*/
void vega_set_position(float Dx ,float Dy ,float Dangle ){
  vega_in_coordinate[0]=Dx;
  vega_in_coordinate[1]=Dy;
  vega_in_coordinate[2]=Dangle;
  uprintf("Vega_position:\r\nx:%5f y:%5f angle:%5f\r\n",vega_in_coordinate[0],vega_in_coordinate[1],vega_in_coordinate[2]);
}
/**vega坐标系变换*/
void vega_coordinate(float pos[3]){

  center_in_vega[0]=1;
  center_in_vega[1]=1;
  center_in_vega[2]=PI;
  Coordinate_System_Transform(center_in_vega,vega_in_coordinate,pos);
}
/**vega赋值修正*/
void vega_correct_pos(char *pos,float correctvalue){
  Vega_Correct veag_correct;
  veag_correct.ch[0]='A';
  veag_correct.ch[1]='C';
  veag_correct.ch[2]='T';

  if(strcmp(pos,"x")==0||strcmp(pos,"X")==0){ 
    veag_correct.ch[3]='X';}
  if(strcmp(pos,"y")==0||strcmp(pos,"Y")==0){ 
    veag_correct.ch[3]='Y';}
  if(strcmp(pos,"angle")==0||strcmp(pos,"Angle")==0||strcmp(pos,"ANGLE")==0){ 
    veag_correct.ch[3]='J';}
  
  veag_correct.fl[1] = correctvalue;

  for(int i = 0;i<=20;i++){
    uprintf_to(&VEGA_USART,"%s",veag_correct.ch);}
}
/**vage坐标打印*/
void vega_print_pos(){
  if(chassis_status.vega_is_ready ==1){
      uprintf("Vega:\r\nx:%5f y:%5f angle:%5f\r\n",chassis.vega_pos_x,chassis.vega_pos_y,chassis.vega_angle);}
  else{
      uprintf("vega has not inited\r\n");} 
}