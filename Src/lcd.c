#include "can_utils.h"
#include "string.h"
#include "lcd.h"
#include "main.h"
#include "configure.h"
#include "chassis.h"
#include "sensor_gpio.h"
#define LCD_VAR_ID 121
#define LCD_FLAG_ID 122
#define LCD_POS_ID 123

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
    lcd_add("FX",&chassis.vega_pos_x);
    lcd_add("FY",&chassis.vega_pos_y);
    lcd_add("FA",&chassis.vega_angle);
    // lcd_add("ICY",&cylinder_state);
    lcd_add("ICN",&chassis_status.count);
    // lcd_add("Fvx",&chassis.vega_pos_x);
    // lcd_add("Fvy",&chassis.vega_pos_x);
    // lcd_add("Fva",&chassis.vega_angle);
    // lcd_add("Flx",&chassis.laser_pos_x);
    // lcd_add("Fly",&chassis.laser_pos_y);
    // lcd_add("ITIM",&clock.m_sec);
    // lcd_add("Fla",&chassis.laser_angle);
}

void lcd_exe(){
    if(flag.lcd_flag==0) return;
    lcd_send_var();
    lcd_send_flag();
}

void lcd_send_var(){
    can_msg lcd_var_msg={0};
    static int now_num=0;
    switch (lcd_msg[now_num].tag[0]){
        case 'i':
        case 'I':{
            strcpy(lcd_var_msg.ch,lcd_msg[now_num].tag);
            lcd_var_msg.in[1] = *(int*)lcd_msg[now_num].value;
            break;}
        case 'f':
        case 'F':{
            strcpy(lcd_var_msg.ch,lcd_msg[now_num].tag);
            lcd_var_msg.fl[1] = *(float*)lcd_msg[now_num].value;
            break;}
    }
    
    if(lcd_msg[now_num].tag[0]=='i'||lcd_msg[now_num].tag[0]=='I'){
        
    }
    if(lcd_msg[now_num].tag[0]=='f'||lcd_msg[now_num].tag[0]=='F'){
        
    }

    can_send_msg(LCD_VAR_ID,&lcd_var_msg);
    flag.lcd_flag = 0;
    now_num ++;
    now_num %= lcd_msg_length;
}
uint8_t lcd_bite(int s[8]){
    uint8_t pack = 0;
    for(int i=0;i<8;i++){
        if(s[i]) 
        pack |= (1<<i);
    }
    return pack;
}
void lcd_send_flag(){
    can_msg lcd_flag_msg={0};
    int tr_flag[8]={0};
    
    
    // tr_flag[0] = 1;
    // tr_flag[1] = 2;
    // tr_flag[2] = 3;
    // tr_flag[3] = 4;
    // tr_flag[4] = 5;
    // tr_flag[5] = 6;
    // tr_flag[6] = 7;
    // tr_flag[7] = 8;
    
    
    
    tr_flag[0] = microswitch_state;
    tr_flag[1] = infrared_state;
    tr_flag[2] = magnet_state;
    tr_flag[3] = cylinder_state;
    tr_flag[4] = flag.chassis_handle_flag;
    tr_flag[5] = chassis_status.vega_is_ready;
    tr_flag[6] = 0;
    tr_flag[7] = 0;
    
    lcd_flag_msg.ui8[0] = lcd_bite(tr_flag);
    lcd_flag_msg.ui8[1] = clock.sec;
    lcd_flag_msg.ui8[2] = clock.min;
    lcd_flag_msg.ui8[3] = 0;

    // lcd_flag_msg.in[1] = 0;
    // lcd_flag_msg.fl[1] = 0;

    can_send_msg(LCD_FLAG_ID,&lcd_flag_msg);

}
void lcd_send_pos(){
    can_msg lcd_pos_msg={0};

}





