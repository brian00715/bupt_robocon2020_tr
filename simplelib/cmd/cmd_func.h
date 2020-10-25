#ifndef __CMD_FUNC_H
#define __CMD_FUNC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "simplelib_cfg.h"
#include "cmd.h"
#include "can_utils.h"
#include "can_func.h"
#include "flags.h"
#include "chassis.h"
#include "vega.h"
#include "laser.h"
#include "main.h"
#include "point_parser.h"
#include "sensor_gpio.h"
#include "robomaster.h"
#include "kickball.h"
#include "touchdown.h"
#include "motor_driver.h"
#include "vesc_can.h"
#include <stdlib.h>

    void cmd_func_init(void);
    void cmd_hello_func(int argc, char *argv[]);
    void cmd_can_test(int argc, char *argv[]);
    void cmd_stop_rocker(int argc, char *argv[]);
    void cmd_show_rocker(int argc, char *argv[]);
    void cmd_wave_test(int argc, char *argv[]);


#ifdef __cplusplus
}
#endif

#endif /* __CMD_FUNC_H */