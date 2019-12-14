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

void cmd_hello(int argc, char *argv[]) {    
    uprintf("Hello!\r\n");
}
//vega cmd
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
//laser cmd
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
//kickball
void cmd_kickball_m2006(int argc, char *argv[]){
    kickball_m2006_current=atof(argv[1]);
    uprintf("M2006 work current set to %f A\r\n",kickball_m2006_current);
}
void cmd_kickball_vesc_pull(int argc, char *argv[]){
    if(atof(argv[1])<0){
        uprintf("VESC pull current should > 0\r\n");
        return;
    }
    kickball_vesc_pull_current=atof(argv[1]);
    uprintf("VESC pull current set to %f A\r\n",kickball_vesc_pull_current);
}
void cmd_kickball_vesc_lossen(int argc, char *argv[]){
    if(atof(argv[1])>0){
        uprintf("VESC lossen current should < 0\r\n");
        return;
    }
    kickball_vesc_lossen_current=atof(argv[1]);
    uprintf("VESC lossen current set to %f A\r\n",kickball_vesc_lossen_current);
}

void cmd_kickball_start(int argc, char *argv[]){
    kickball_set_status(KICKBALL_MAGNET_TO_BOARD);
    uprintf("Kickball start!\r\nManget is going up\r\n");
}
void cmd_kickball_ready(int argc, char *argv[]){
    kickball_set_status(KICKBALL_BOARD_TO_DOWN);
    uprintf("Magnet is down!\r\nReady to kick\r\n");
}
void cmd_kickball_kick(int argc, char *argv[]){
    kickball_set_status(KICKBALL_KICK);
    uprintf("Magnet is down!\r\nReady to kick\r\n");
}




void cmd_func_init(void) {
    cmd_add("hello", " ",cmd_hello);
    //vega
    cmd_add("vega_print_pos", "print vega x y angle",cmd_vega_print_pos);
    cmd_add("vega_correct_pos", "correct vega x y angle",cmd_vega_correct_pos);
    cmd_add("vega_set_position", "set vega position compared to coordinate systemx y angle",cmd_vega_set_position);
    cmd_add("vega_reset", "reset vega",cmd_vega_reset);
    cmd_add("vega_calibrate", "calibrate vega",cmd_vega_calibrate);
    //laser
    cmd_add("laser_print_diatance", "print laser 3 diatance",cmd_laser_print_diatance);
    cmd_add("laser_print_pos", "print laser x y angle",cmd_laser_print_pos);
    //point
    cmd_add("point_print_path", "print laser x y angle",cmd_point_print_path);\
    //gpio
    cmd_add("gpio_magnet", "",cmd_gpio_magnet);
    cmd_add("gpio_cylinder", "",cmd_gpio_cylinder);
    cmd_add("gpio_microswitch", "",cmd_gpio_microswitch);
    cmd_add("gpio_infrared", "",cmd_gpio_infrared);
    //kickball
    cmd_add("kickball_m2006", "",cmd_kickball_m2006);
    cmd_add("kickball_vesc_pull", "",cmd_kickball_vesc_pull);
    cmd_add("kickball_vesc_lossen", "",cmd_kickball_vesc_lossen);
    cmd_add("kickball_start", "",cmd_kickball_start);
    cmd_add("kickball_ready", "",cmd_kickball_ready);
    cmd_add("kickball_kick", "",cmd_kickball_kick);
}