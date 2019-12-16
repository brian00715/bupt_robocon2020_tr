#include <stdlib.h>
#include "cmd_func.h"
#include "can_utils.h"
#include "can_func.h"
#include "flags.h"
#include "chassis.h"
#include "vega.h"
#include "laser.h"
#include "point_parser.h"
#include "sensor_gpio.h"
#include "robomaster.h"
#include "kickball.h"
#include "touchdown.h"
#include "motor_driver.h"

void cmd_hello(int argc, char *argv[]) {    
    uprintf("Hello!\r\n");
}
//vega
void cmd_vega_print_pos(int argc, char *argv[]){
    vega_print_pos();   
}
void cmd_vega_set_position(int argc, char *argv[]){
    vega_set_position(atof(argv[1]),atof(argv[2]),atof(argv[3]));
}
void cmd_vega_reset(int argc, char *argv[]){
    vega_reset();     
}
void cmd_vega_correct_pos(int argc, char *argv[]){
    vega_correct_pos(atof(argv[1]),atof(argv[2]),atof(argv[3]));
    uprintf("Vega correct to:\r\n");
    uprintf("X:%5f Y:%5f Angle:%5f\r\n",atof(argv[1]),atof(argv[2]),atof(argv[3]));
}
void cmd_vega_calibrate(int argc, char *argv[]){
    vega_calibration();    
}
//laser
void cmd_laser_print_diatance(int argc, char *argv[]){
    laser_print_distance();
}
void cmd_laser_print_pos(int argc, char *argv[]){
    laser_print_pos();
}
//point
void cmd_point_print_path(int argc, char *argv[]){
    point_print_path();
}
//gpio
void cmd_gpio_magnet(int argc, char *argv[]){
    magnet_state=atoi(argv[1]);
    uprintf("Magnet state is %d\r\n",atoi(argv[1]));
}
void cmd_gpio_cylinder(int argc, char *argv[]){
    cylinder_state=atoi(argv[1]);
    uprintf("Cylinder state is %d\r\n",atoi(argv[1]));
}
void cmd_gpio_microswitch(int argc, char *argv[]){
    uprintf("Key state is %d\r\n",microswitch_state);
}
void cmd_gpio_infrared(int argc, char *argv[]){
    uprintf("Infrared state is %d\r\n",infrared_state);
}
void cmd_gpio_all(int argc, char *argv[]){
    uprintf("\r\nMicroswitch state is %d\r\n",microswitch_state);
    uprintf("Infrared    state is %d\r\n",infrared_state);
    uprintf("Magnet      state is %d\r\n",magnet_state);
    uprintf("Cylinder    state is %d\r\n",cylinder_state);
}
//kickball
void cmd_kickball_auto(int argc, char *argv[]){
    if(atof(argv[1]) == 1){
        kickball_ready_flag = 0;
        kickball_prepare_flag = 0;
        kickball_pull_magnet_flag = 0;
        kickball_stop_magnet_flag = 0;
        kickball_kick_flag = 0;
        kickball_auto = 1;
        uprintf("Kickball switch to auto mode\r\n");
    }
    else {
        kickball_auto = 0;        
        uprintf("Kickball switch to handle mode\r\n");
        uprintf("kicknum = %d\r\n",kickball_num+1);
        
    }
}

//kickball handle
void cmd_kickball_m2006(int argc, char *argv[]){
    kickball_M2006_set_current(atof(argv[1]));
    uprintf("M2006 current set to %f A\r\n",atof(argv[1]));
}
void cmd_kickball_vesc(int argc, char *argv[]){
    if(atof(argv[1])>=0){
        kickball_VESC_set_pull_current(atof(argv[1]));
        uprintf("VESC pull current is %f \r\n",atof(argv[1]));
    }
    else{
        kickball_VESC_set_loosen_duty(atof(argv[1]));
        uprintf("VESC lossen duty is %f \r\n",atof(argv[1]));
    }
    
}

//kickball auto
void cmd_kickball_prepare(int argc, char *argv[]){
    kickball_prepare_flag = 1;
    uprintf("Kickball prapare!\r\nManget is going up\r\n");
}
void cmd_kickball_pull_magnet(int argc, char *argv[]){
    kickball_pull_magnet_flag = 1;
    uprintf("Magnet is going down!\r\n");
}
void cmd_kickball_stop_magnet(int argc, char *argv[]){       
    kickball_stop_magnet_flag = 1;
    uprintf("Magnet stopped!\r\nReady to kick\r\n");
}
void cmd_kickball_kick(int argc, char *argv[]){
    kickball_kick_flag = 1;
    uprintf("Kick the ball!!!\r\n");
}
//touchdown
void cmd_touchdown_try(int argc, char *argv[]){
    touchdown_try_flag = 1;
    uprintf("Try touchdown!!!\r\n");
}




void cmd_func_init(void) {
    cmd_add("hello", " ",cmd_hello);
    //vega
    cmd_add("vega_print_pos", "",cmd_vega_print_pos);
    cmd_add("vega_correct_pos", "",cmd_vega_correct_pos);
    cmd_add("vega_set_position", "",cmd_vega_set_position);
    cmd_add("vega_reset", "",cmd_vega_reset);
    cmd_add("vega_calibrate", "",cmd_vega_calibrate);
    //laser
    cmd_add("laser_print_diatance", "",cmd_laser_print_diatance);
    cmd_add("laser_print_pos", "",cmd_laser_print_pos);
    //point
    cmd_add("point_print_path", "",cmd_point_print_path);\
    //gpio
    cmd_add("gpio_magnet", "",cmd_gpio_magnet);
    cmd_add("gpio_cylinder", "",cmd_gpio_cylinder);
    cmd_add("gpio_microswitch", "",cmd_gpio_microswitch);
    cmd_add("gpio_infrared", "",cmd_gpio_infrared);
    cmd_add("gpio_all", "",cmd_gpio_all);    
    //kickball
    cmd_add("kickball_auto", "",cmd_kickball_auto);

    cmd_add("kickball_m2006", "",cmd_kickball_m2006);
    cmd_add("kickball_vesc", "",cmd_kickball_vesc);

    cmd_add("kickball_prepare", "",cmd_kickball_prepare);
    cmd_add("kickball_pull_magnet", "",cmd_kickball_pull_magnet);
    cmd_add("kickball_stop_magnet", "",cmd_kickball_stop_magnet);
    cmd_add("kickball_kick", "",cmd_kickball_kick);
    //touchdown
    cmd_add("touchdown_try", "",cmd_touchdown_try);
}