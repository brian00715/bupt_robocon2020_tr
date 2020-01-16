#include "configure.h"

Can_id_send send_id;
Can_id_recive recive_id;

void can_id_init()
{
    send_id.motor0_id = 101;
    send_id.motor1_id = 102;
    send_id.motor2_id = 103;  
    send_id.lcd_id = 121;
    
    recive_id.handle_button_id = 98;
    recive_id.handle_rocker_id = 99;
    recive_id.lcd_id = 200;
    recive_id.pos_id = 1;    //自己的全场定位
}

