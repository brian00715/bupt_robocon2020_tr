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

void vega_action_reset(){
  for(int i = 0;i<=20;i++){
    uprintf_to(&VEGA_USART,"ACT0");
  }
}

//TODO vega解算到中心
//TODO vega校准


