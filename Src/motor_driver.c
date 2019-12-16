#include "main.h"
#include "robomaster.h"
#include "vesc_can.h"
#include "kickball.h"
#include "motor_driver.h"
void motor_init()
{
    vesc.id=88;
}
//vesc
VESC_STATE vesc = {0};

void vsec_exe(){
    if (flag.vesc_flag==0) return;
    if(vesc.mode == 0) comm_can_set_duty(vesc.id, vesc.duty); 
    if(vesc.mode == 1) comm_can_set_current(vesc.id, vesc.current);
    if(vesc.mode == 2) comm_can_set_rpm(vesc.id,vesc.rpm);
    if(vesc.mode == 3) //位置环
    flag.vesc_flag = 0;    
}

//m2006
float touchdown_current=0;
float kick_current=0;
void m2006_exe(){
  if(flag.m2006_flag == 0) return;
  int16_t door_I = (int16_t)(touchdown_current*1000) ;
  int16_t kick_I = (int16_t)(kick_current*1000) ;
  robomaster_set_current(door_I,kick_I,0,0);
  flag.m2006_flag = 0;
}
