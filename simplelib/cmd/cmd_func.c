#include <stdlib.h>
#include "cmd_func.h"
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

void CMD_Hello(int argc, char *argv[])
{
    uprintf("Hello!\r\n");
}

//vega
void CMD_Vega_PrintPos(int argc, char *argv[])
{
    vega_print_pos();
}

// void cmd_vega_set_position(int argc, char *argv[])
// {
//     vega_set_position(atof(argv[1]), atof(argv[2]), atof(argv[3]));
// }
// void cmd_vega_reset(int argc, char *argv[])
// {
//     vega_reset();
// }
// //东大全场定位赋值修正
// void cmd_vega_correct_pos(int argc, char *argv[])
// {
//     vega_correct_pos(atof(argv[1]), atof(argv[2]), atof(argv[3]));
//     uprintf("Vega correct to:\r\n");
//     uprintf("X:%5f Y:%5f Angle:%5f\r\n", atof(argv[1]), atof(argv[2]), atof(argv[3]));
// }
// void cmd_vega_calibrate(int argc, char *argv[])
// {
//     vega_calibration();
// }

// //laser
// void cmd_laser_print_diatance(int argc, char *argv[])
// {
//     laser_print_distance();
// }
// void cmd_laser_print_pos(int argc, char *argv[])
// {
//     laser_print_pos();
// }

//point
void CMD_Point_PrintPath(int argc, char *argv[])
{
    point_print_path();
}

/*设置m2006的参数，用以手动控制*/
void CMD_M2006_SetCurrent(int argc, char *argv[])
{
    if (Kickball_ControlMode == AUTO)
    {
        uprintf("##Please change to manual mode!##\r\n");
        return;
    }
    //touchdown_current = atof(argv[1]); // 达阵电机电流
    //uprintf("M2006 touchdown current is %f.\r\n", atof(argv[1]));
    MoterDriver_M2006_Current = atof(argv[1]); // 转磁铁电机电流
    uprintf("M2006 current is %f.\r\n", atof(argv[1]));
}

/*设置本杰明电调的参数,用以手动控制*/
void CMD_VESC_SetParam(int argc, char *argv[])
{
    if (Kickball_ControlMode == AUTO)
    {
        uprintf("##Please change to manual mode!##\r\n");
        return;
    }

    vesc.mode = atoi(argv[1]);
    switch (vesc.mode)
    {
    case 0:
    {
        // 正值顺时针转动（收线），负值逆时针转动（放线）
        vesc.duty = atof(argv[2]);
        uprintf("--Vesc work in Duty at %f\r\n", atof(argv[2]));
        break;
    }
    case 1:
    {
        vesc.current = atof(argv[2]);
        uprintf("--Vesc work in Current at %f\r\n", atof(argv[2]));
        break;
    }
    case 2:
    {
        vesc.rpm = atof(argv[2]);
        uprintf("--Vesc work in RPM at %f\r\n", atof(argv[2]));
        break;
    }
    case 3:
    {
        vesc.position = atof(argv[2]);
        uprintf("--Vesc work in Position at %f\r\n", atof(argv[2]));
        break;
    }
    default:
    {
        uprintf("--parameter explain:\r\n ");
        uprintf("  0:duty\r\n1:current\r\n2:speed\r\n3:position\r\n");
        break;
    }
    }
}

//gpio
void CMD_GPIO_Magnet_SetStatus(int argc, char *argv[])
{
    magnet_state = atoi(argv[1]);
    uprintf("Magnet state set to %d\r\n", atoi(argv[1]));
}

// void cmd_gpio_cylinder(int argc, char *argv[])
// {
//     cylinder_state = atoi(argv[1]);
//     uprintf("Cylinder state is %d\r\n", atoi(argv[1]));
// }

void CMD_GPIO_Microswitch_GetStatus(int argc, char *argv[])
{
    uprintf("Key state is %d\r\n", microswitch_state);
}

// void cmd_gpio_infrared(int argc, char *argv[])
// {
//     uprintf("Infrared state is %d\r\n", infrared_state);
// }

void CMD_GPIO_GetAllStatus(int argc, char *argv[])
{
    uprintf("\r\n--Microswitch state is %d\r\n", microswitch_state);
    uprintf("  Infrared    state is %d\r\n", infrared_state);
    uprintf("  Magnet      state is %d\r\n", magnet_state);
    uprintf("  Cylinder    state is %d\r\n", cylinder_state);
}

//kickball
void CMD_Kickball_SetControlMode(int argc, char *argv[])
{
    if (atof(argv[1]) == 1) // 输入1启动自动控制模式
    {
        // 状态机清零
        kickball_ready_flag = 0;
        kickball_prepare_flag = 0;
        kickball_pull_magnet_flag = 0;
        kickball_stop_magnet_flag = 0;
        kickball_kick_flag = 0;
        Kickball_ControlMode = AUTO;
        uprintf("--Kickball control mode switch to auto mode.\r\n");
    }
    else // 输入0启动手动控制模式
    {
        Kickball_ControlMode = MANUAL;
        uprintf("--Kickball control mode switch to manual mode.\r\n");
        uprintf("  kicknum = %d\r\n", kickball_num + 1); // 踢第几颗球
    }
}

/**
 * @brief cmd设定转磁铁的m2006的电流值，用以调试转速.
 *        需要在自动控制模式下使用
 **/
void CMD_Kickball_M2006Current(int argc, char *argv[])
{
    if (Kickball_ControlMode == MANUAL)
    {
        uprintf("##please change to auto mode!##\r\n");
        return;
    }
    kickball_m2006_current = (atof(argv[1]));
    uprintf("--kickball M2006 current set to %.2f A.\r\n", atof(argv[1]));
}

/**
 * @brief cmd设定收放线电机的电流值，用以调试转速
 **/
void CMD_Kickball_VESC(int argc, char *argv[])
{
    if (Kickball_ControlMode == MANUAL)
    {
        uprintf("##please change to auto mode!##\r\n");
        return;
    }

    if (atof(argv[1]) < 0) // 参数小于0则是收线电流
    {
        kickball_vesc_pull_current5 = (atof(argv[1]));
        uprintf("--kickball VESC pull current set to %.2f. \r\n", atof(argv[1]));
    }
    else // 参数大于0则是放线占空比
    {
        kickball_vesc_lossen_duty = (atof(argv[1]));
        uprintf("--kickball VESC lossen duty set to %.2f. \r\n", atof(argv[1]));
    }
}

/**
 * @brief 只有自动控制模式下才能使用此函数，执行将磁铁往上转的过程
 **/
void CMD_Kickball_Prepare(int argc, char *argv[])
{
    if (Kickball_ControlMode == MANUAL)
    {
        uprintf("##please change to auto mode!##\r\n");
        return;
    }
    kickball_prepare_flag = 1;
    uprintf("--Kickball prapare!\r\n  Manget is going up.\r\n");
}

/**
 * @brief 磁铁吸到踢球版上后开始往下拉
 **/
void CMD_Kickball_PullMagnet(int argc, char *argv[])
{
    if (Kickball_ControlMode == MANUAL)
    {
        uprintf("##please change to auto mode!##\r\n");
        return;
    }
    kickball_pull_magnet_flag = 1;
    uprintf("--Magnet is going down!\r\n");
}

/**
 * @brief 人眼判断弹簧拉紧后cmd调用此函数，电机停转，准备踢出
 **/
void CMD_Kickball_StopPullMagnet(int argc, char *argv[])
{
    if (Kickball_ControlMode == MANUAL)
    {
        uprintf("##please change to auto mode!##\r\n");
        return;
    }
    kickball_stop_magnet_flag = 1;
    uprintf("--Magnet stopped!\r\n  Ready to kick.\r\n");
}

void CMD_Kickball_Kick(int argc, char *argv[])
{
    if (Kickball_ControlMode == MANUAL)
    {
        uprintf("##please change to auto mode!##\r\n");
        return;
    }
    kickball_kick_flag = 1;
    uprintf("--Kick the ball!!--\r\n");
}

int kickball_stoped_flag = 0; // 局部变量,表征调用了CMD_Kickball_StopAll函数
/*停止动作，状态机复位*/
void CMD_Kickball_StopAll(int argc, char *argv[])
{
    vesc.duty = 0;
    vesc.mode = 0;                       // 占空比模式
    comm_can_set_rpm(vesc.id, vesc.rpm); // 立即执行
    MoterDriver_M2006_Current = 0;
    robomaster_set_current(0, 0, 0, 0); // 立即执行
    uprintf("\r\n##All Moter Stoped!##\r\n");

    // 状态机复位
    kickball_ready_flag = 0;
    kickball_prepare_flag = 0;
    kickball_pull_magnet_flag = 0;
    kickball_stop_magnet_flag = 0;
    kickball_kick_flag = 0;
    Kickball_ControlMode = MANUAL;
    kickball_status = KICKBALL_NONE;
    magnet_state = 0;
    uprintf("==Status Machine Reseted! Now is Maunal Mode==\r\n");
    uprintf("--You can enter <kickball_reset_megnet> to continue.\r\n\r\n");
    kickball_stoped_flag = 1;
}

/*将钢丝收回，重新准备下一次踢球动作*/
/*可以手动设置收线电流，或使用默认值*/
void CMD_Kickball_ResetMegnet(int argc, char *argv[])
{
    if (kickball_stoped_flag == 0)
    {
        uprintf("##Please Call <CMD_Kickball_StopAll> at first!##\r\n");
        return;
    }
    kickball_stoped_flag = 0; // 标志复位
    uprintf("--megnet start reset.\r\n");
    uprintf("  please enter <vesc 0 0> at proper time.\r\n");
    if (argc < 2)
    {
        kickball_VESC_set_pull_current(-1); // 收线
    }
    else
    {
        kickball_VESC_set_pull_current(atof(argv[1]));
    }
}

void CMD_Chassis_Move(int argc, char *argv[])
{
    // test_value[0] = atof(argv[1]);
    // test_value[1] = atof(argv[2]);
    // test_value[2] = atof(argv[3]);
    Chassis_MoterDuty[0] = atoi(argv[1]);
    Chassis_MoterDuty[1] = atoi(argv[2]);
    Chassis_MoterDuty[2] = atoi(argv[3]);
    uprintf("move in duty of %d  %d %d\r\n", Chassis_MoterDuty[0], 
    Chassis_MoterDuty[1], Chassis_MoterDuty[2]);
}

//打印实际的底盘坐标
void CMD_Chassis_PrintPos(int argc, char *argv[])
{
    uprintf("Chassis:\r\nx:%5f y:%5f angle:%5f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
}

void cmd_func_init(void)
{
    cmd_add("hello", "", CMD_Hello);
    //point
    cmd_add("point_print_path", "", CMD_Point_PrintPath);

    //vega
    cmd_add("vega_print_pos", "", CMD_Vega_PrintPos);
    // cmd_add("vega_correct_pos", "", cmd_vega_correct_pos);
    // cmd_add("vega_set_position", "", cmd_vega_set_position);
    // cmd_add("vega_reset", "reset the vega(BAN!)", cmd_vega_reset);
    // cmd_add("vega_calibrate", "BAN!", cmd_vega_calibrate);

    //gpio
    cmd_add("gpio_set_magnet_status", "<1 to on;0 to off>", CMD_GPIO_Magnet_SetStatus);
    // cmd_add("gpio_cylinder", "set cylinder state", cmd_gpio_cylinder);
    cmd_add("gpio_microswitch", "get microswitch state", CMD_GPIO_Microswitch_GetStatus);
    // cmd_add("gpio_infrared", "get infrared state", cmd_gpio_infrared);
    cmd_add("gpio_all", "get all gpio state", CMD_GPIO_GetAllStatus);

    //motor
    cmd_add("vesc", "<mode(0,1,2)> <value>", CMD_VESC_SetParam);
    cmd_add("m2006", "<current>", CMD_M2006_SetCurrent);

    //kickball
    cmd_add("kickball_set_control_mode", "0 to handle; 1 to auto", CMD_Kickball_SetControlMode);
    cmd_add("kickball_m2006", "<current value>", CMD_Kickball_M2006Current);
    cmd_add("kickball_vesc", "<current value>(<0pull;>0loose)", CMD_Kickball_VESC);
    cmd_add("kickball_prepare", "turn magnet to board", CMD_Kickball_Prepare);
    cmd_add("kickball_pull_magnet", "", CMD_Kickball_PullMagnet);
    cmd_add("kickball_stop_pull_magnet", "", CMD_Kickball_StopPullMagnet);
    cmd_add("kickball_kick", "", CMD_Kickball_Kick);
    cmd_add("kickball_stop_all", "stop moter & reset status", CMD_Kickball_StopAll);
    cmd_add("kickball_reset_megnet", "pull megnet after stop motor", CMD_Kickball_ResetMegnet);

    //chassis
    cmd_add("chassis_move", "<speed of 3 motors>", CMD_Chassis_Move);
    cmd_add("chassis_print_pos", "", CMD_Chassis_PrintPos);

    //touchdown
    // cmd_add("touchdown_auto", "", cmd_touchdown_auto);
    // cmd_add("touchdown_open", "<current value>", cmd_touchdown_open);
    // cmd_add("touchdown_close", "<current value>", cmd_touchdown_close);
    // cmd_add("touchdown_try", "", cmd_touchdown_try);
    // cmd_add("touchdown_try_finish", "", cmd_touchdown_try_finish);
    //laser
    // cmd_add("laser_print_diatance", "", cmd_laser_print_diatance);
    // cmd_add("laser_print_pos", "", cmd_laser_print_pos);
}

//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓【达阵CMD函数】↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
// /**
//  * @brief 达阵控制模式 1 自动控制 0 手动控制
//  **/
// void cmd_touchdown_auto(int argc, char *argv[])
// {
//     if (atof(argv[1]) == 1)
//     {
//         touchdown_ready_flag = 0;
//         touchdown_try_flag = 0;
//         touchdown_auto_flag = 1;
//         uprintf("--Touchdwon switch to auto mode--\r\n");
//     }
//     else
//     {
//         touchdown_auto_flag = 0;
//         uprintf("--Touchdwon switch to handle mode--\r\n");
//     }
// }
// void cmd_touchdown_open(int argc, char *argv[])
// {
//     if (atof(argv[1]) > 0)
//     {
//         uprintf("Touchdwon open current should < 0.\r\n");
//     }
//     else
//     {
//         touchdown_m2006_open(atof(argv[1]));
//         uprintf("Touchdwon open current is %f.\r\n", atof(argv[1]));
//     }
// }
// void cmd_touchdown_close(int argc, char *argv[])
// {
//     if (atof(argv[1]) < 0)
//     {
//         uprintf("Touchdwon open current should > 0.\r\n");
//     }
//     else
//     {
//         touchdown_m2006_close(atof(argv[1]));
//         uprintf("Touchdwon close current is %f\r\n", atof(argv[1]));
//     }
// }
// void cmd_touchdown_try(int argc, char *argv[])
// {
//     if (touchdown_ready_flag == 1)
//     {
//         touchdown_try_flag = 1;
//         uprintf("Try touchdown!!!\r\n");
//     }
//     else
//     {
//         uprintf("No ball in the basket.\r\n");
//     }
// }
// void cmd_touchdown_try_finish(int argc, char *argv[])
// {
//     if (touchdown_try_flag == 1)
//     {
//         touchdown_try_finish_flag = 1;
//         uprintf("Try touchdown finish!!!\r\n");
//     }
//     else
//     {
//         uprintf("Haven't try\r\n");
//     }
// }