/*******************************************************************************
Copyright:      Bupt
File name:      vega_action.c
Description:    action全场定位，使用前初始化十秒
Author:         Leo
Version：       1.0
Data:           2019/11/24
*******************************************************************************/
#include "vega_action.h"
#include "cmd.h"
#include "utils.h"

void vega_action_reset(){
  for(int i = 0;i<=20;i++){
    //uprintf_to(&VEGA_USART,"ACT0");
  }
}

float Angle_Limit_180(float angle){
 while(angle>180)
 {
     angle -= 360;
 }
 while(angle<=-180)
 {
     angle += 360;
 }
 return angle;
}
float Angle_Limit_PI(float angle){
 while(angle > PI)
 {
     angle -= 2*PI;
 }
 while(angle<=-PI)
 {
     angle += 2*PI;
 }
 return angle;
}
/**坐标系变换
 * now 当前坐标系
 * now_in_target 当前坐标系在目标坐标系中的位置，角度逆时针为正
 * target 目标坐标系
*/
void Coordinate_System_Transform(float now[3],float now_in_target[3] , float target[3])
{
  float c = cos(-now_in_target[2]);
  float s = sin(-now_in_target[2]);
  target[0] = now[0] * c + now[1]* s - now_in_target[0];
  target[1] = now[0] *(-s) + now[1]* c - now_in_target[1];
  target[2] = Angle_Limit_PI( now[2]-now_in_target[2]);
}
float vega_position[3]={1,1,PI};
void vega_coordinate(float pos[3]){
  float vega[3];
  vega[0]=1;
  vega[1]=1;
  vega[2]=PI;
  Coordinate_System_Transform(vega,vega_position,pos);
}