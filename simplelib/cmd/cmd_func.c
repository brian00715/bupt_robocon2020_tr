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
    gpio_magnet(atoi(argv[1]));
}
void cmd_gpio_cylinder(int argc, char *argv[]){
    gpio_cylinder(atoi(argv[1]));
    uprintf("Cylinder state is %d\r\n",atoi(argv[1]));
}
void cmd_gpio_key(int argc, char *argv[]){
    int state=gpio_micro_key();
    uprintf("Key state is %d\r\n",state);
}
void cmd_gpio_infrared(int argc, char *argv[]){
    int state=gpio_infrared();
    uprintf("Infrared state is %d\r\n",state);
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
    cmd_add("gpio_magnet", "correct vega x y angle",cmd_gpio_magnet);
    cmd_add("gpio_cylinder", "set vega position compared to coordinate systemx y angle",cmd_gpio_cylinder);
    cmd_add("gpio_key", "reset vega",cmd_gpio_key);
    cmd_add("gpio_infrared", "calibrate vega",cmd_gpio_infrared);
}