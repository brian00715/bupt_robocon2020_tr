#include"touchdown.h"
#include "sensor_gpio.h"

static TOUCHDOWN_STATUS touchdown_status=0;

/****************************电机气缸**************************/

/**气缸推球*/
void touchdown_cylinder_push(){
    cylinder_state = 1; //!
}
/**气缸收回*/
void touchdown_cylinder_pull(){
    cylinder_state = 0; //!
}
void touchdown_m2006_open(){

}
void touchdown_m2006_close(){
    
}

void touchdown_exe(){
 switch(touchdown_status){
    case TOUCHDOWN_NONE:{
        if(infrared_state == 1){ //!
            touchdown_status = TOUCHDOWN_GETBALL;
        }
    break;}

    case TOUCHDOWN_GETBALL:{
        touchdown_m2006_close();
    break;}

    case TOUCHDOWN_READY:{
        touchdown_m2006_open();
        //Hal_Delay(5);
        touchdown_cylinder_push();
        //Hal_Delay(5);
        touchdown_cylinder_pull();
    break;}
 }
}