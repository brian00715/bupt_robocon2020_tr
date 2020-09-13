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
float vega_in_coordinate[3] = {0.439f, 0.320f, 0};
float center_in_vega[3] = {1, 1, 1};

/**vega坐标系变换*/
void vega_coordinate(float pos[3])
{
  center_in_vega[0] = chassis.vega_pos_x;
  center_in_vega[1] = chassis.vega_pos_y;
  center_in_vega[2] = chassis.vega_angle;
  Coordinate_System_Transform(center_in_vega, vega_in_coordinate, pos);
}

/**vage坐标打印*/
void vega_print_pos()
{
  if (chassis_status.vega_is_ready == 1)
  {
    uprintf("Vega:\r\nx:%5f y:%5f angle:%5f\r\n", chassis.vega_pos_x, chassis.vega_pos_y, chassis.vega_angle);
  }
  else
  {
    uprintf("vega has not inited\r\n");
  }
}

/**
 * @brief 自研全场定位坐标发送，CAN总线方式，发送两个数据包
 *        分别为角度，x坐标和y坐标
 *        ID分别为11和22
 **/
void vega_print_pos_can()
{
  can_msg angle_msg;
  angle_msg.fl[0] = chassis.vega_angle;
  can_send_msg(11, &angle_msg);
  can_msg xy_msg;
  xy_msg.fl[0] = chassis.vega_pos_x;
  xy_msg.fl[1] = chassis.vega_pos_y;
  can_send_msg(22, &xy_msg);  
}

/**vega校准*/
// ！！！！禁用！！！！
// void vega_calibration()
// {
//   for (int i = 0; i <= 20; i++)
//   {
//     uprintf_to(&VEGA_USART, "ACTR");
//   }
//   uprintf("send vega calibrate message\r\n");
// }

// /**vega复位*/
// // ！！！！禁用！！！！
// void vega_reset()
// {
//   for (int i = 0; i <= 20; i++)
//   {
//     uprintf_to(&VEGA_USART, "ACT0");
//   }
//   uprintf("send vega reset message\r\n");
// }

// /**vega相对坐标原点位置设置*/
// // ！！！！禁用！！！！
// void vega_set_position(float Dx, float Dy, float Dangle)
// { 
//   vega_in_coordinate[0] = Dx;
//   vega_in_coordinate[1] = Dy;
//   vega_in_coordinate[2] = Dangle;
//   uprintf("Vega_position:\r\nx:%5f y:%5f angle:%5f\r\n", vega_in_coordinate[0], vega_in_coordinate[1], vega_in_coordinate[2]);
// }


/**vega赋值修正*/
// void vega_correct_pos(float x, float y, float angle)
// {
//   Vega_Correct veag_correct;
//   veag_correct.ch[0] = 'A';
//   veag_correct.ch[1] = 'C';
//   veag_correct.ch[2] = 'T';
//   veag_correct.ch[3] = 'A';

//   veag_correct.fl[1] = angle;
//   veag_correct.fl[2] = x;
//   veag_correct.fl[3] = y;

//   for (int i = 0; i <= 20; i++)
//   {
//     for (int j = 0; j < 15; j++)
//       uprintf_to(&VEGA_USART, "%c", veag_correct.ch[j]);
//   }
// }

// void vega_correct_pos(char *pos,float correctvalue){
//   Vega_Correct veag_correct;
//   veag_correct.ch[0]='A';
//   veag_correct.ch[1]='C';
//   veag_correct.ch[2]='T';

//   if(strcmp(pos,"x")==0||strcmp(pos,"X")==0){
//     veag_correct.ch[3]='X';}
//   if(strcmp(pos,"y")==0||strcmp(pos,"Y")==0){
//     veag_correct.ch[3]='Y';}
//   if(strcmp(pos,"angle")==0||strcmp(pos,"Angle")==0||strcmp(pos,"ANGLE")==0){
//     veag_correct.ch[3]='J';}

//   veag_correct.fl[1] = correctvalue;

//   // for(int i = 0;i<=20;i++){
//   //   uprintf_to(&VEGA_USART,"%s",veag_correct.ch);}

//   for(int i = 0;i<=20;i++){
//     for(int j=0;j<8;j++)
//     uprintf_to(&VEGA_USART,"%c",veag_correct.ch[j]);}
// }

