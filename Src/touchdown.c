#include"touchdown.h"
#include "sensor_gpio.h"
#include "cmd.h"
#include "motor_driver.h"
static TOUCHDOWN_STATUS touchdown_status=TOUCHDOWN_NONE;

/****************************电机气缸**************************/
/**气缸推球*/
void touchdown_cylinder_push(){
    cylinder_state = 1; 
}
/**气缸收回*/
void touchdown_cylinder_pull(){
    cylinder_state = 0; 
}
/**电机开门*/
void touchdown_m2006_open(float current){    
    if(current > 0) return ;
    touchdown_current = current;
}
/**电机关门*/
void touchdown_m2006_close(float current){ 
    if(current < 0) return ;
    touchdown_current = current;    
}
/****************************外部调用**************************/
int touchdown_ready_flag =0;            //接到球置1 推出球置0 外部读取有无球
int touchdown_try_flag =0;              //外部置1 推球
float touchdown_open_current = -1.5;
float touchdown_close_current = 1.1;
int touchdown_auto_flag = 1;

void touchdown_state_machine(){
    static Clock time ={0};
    static int waitingtime=0;
    switch(touchdown_status){
    case TOUCHDOWN_NONE:{
        touchdown_m2006_close(touchdown_close_current);
        if(infrared_state == 0){ //红外被挡住为 0
            touchdown_status = TOUCHDOWN_GETBALL;    
            touchdown_ready_flag = 1;
        }
    break;}

    case TOUCHDOWN_GETBALL:{
        touchdown_m2006_close(touchdown_close_current);
        if(touchdown_try_flag == 1){
            touchdown_status = TOUCHDOWN_TRY; 
            touchdown_try_flag = 0;
            touchdown_status = TOUCHDOWN_TRY;

            
            time.min=clock.min;
            time.sec=clock.sec;
        }

    break;}

    case TOUCHDOWN_TRY:{
        touchdown_m2006_open(touchdown_open_current);
        touchdown_cylinder_push();
        waitingtime = (int) (60*(clock.min-time.min) + (clock.sec-time.sec) +(clock.m_sec-time.m_sec)/100);
        if(waitingtime == 4){
            touchdown_cylinder_pull();
            touchdown_m2006_close(touchdown_close_current);
            touchdown_status = TOUCHDOWN_NONE;
        }        
        touchdown_ready_flag = 0;
    break;}
 }
}


void touchdown_exe(){
 if (touchdown_auto_flag == 1){ touchdown_state_machine();}
 
}