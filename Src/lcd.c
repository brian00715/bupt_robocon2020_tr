#include "can_utils.h"
#include "string.h"
#include "lcd.h"
#include "main.h"
#include "configure.h"
#include "chassis.h"
#include "sensor_gpio.h"

#define MAX_LCD_MSG_NUM 50
Lcd_Msg lcd_msg[MAX_LCD_MSG_NUM]={0};
static int lcd_msg_length=0;

int lcd_add(char tag[],void* value_pointer){
    
    if(lcd_msg_length >= MAX_LCD_MSG_NUM){
        uprintf("Lcd_Msg Box is full!!!\r\n");
        return MAX_LCD_MSG_NUM;
    }
    

    strcpy(lcd_msg[lcd_msg_length].tag,tag);
    lcd_msg[lcd_msg_length].value = value_pointer;
    lcd_msg_length++;

    return lcd_msg_length;
} 


void lcd_init(){
    lcd_add("IMA",&magnet_state);
    lcd_add("IMI",&microswitch_state);
    lcd_add("IIN",&infrared_state);
    lcd_add("ICY",&cylinder_state);

    // lcd_add("Fvx",&chassis.vega_pos_x);
    // lcd_add("Fvy",&chassis.vega_pos_x);
    // lcd_add("Fva",&chassis.vega_angle);
    // lcd_add("Flx",&chassis.laser_pos_x);
    // lcd_add("Fly",&chassis.laser_pos_y);
    // lcd_add("Fla",&chassis.laser_angle);
}




int lcd_exe(){
    if(flag.lcd_flag==0) return 0;
    static int now_num=0;
    can_msg lcd_can_msg;

    switch (lcd_msg[now_num].tag[0]){
        case 'i':
        case 'I':{
            strcpy(lcd_can_msg.ch,lcd_msg[now_num].tag);
            lcd_can_msg.in[1] = *(int*)lcd_msg[now_num].value;
            break;}
        case 'f':
        case 'F':{
            strcpy(lcd_can_msg.ch,lcd_msg[now_num].tag);
            lcd_can_msg.fl[1] = *(float*)lcd_msg[now_num].value;
            break;}
    }
    
    if(lcd_msg[now_num].tag[0]=='i'||lcd_msg[now_num].tag[0]=='I'){
        
    }
    if(lcd_msg[now_num].tag[0]=='f'||lcd_msg[now_num].tag[0]=='F'){
        
    }

    can_send_msg(send_id.lcd_id,&lcd_can_msg);
    flag.lcd_flag = 0;
    now_num ++;
    now_num %= lcd_msg_length;
    return (now_num+1);  
}