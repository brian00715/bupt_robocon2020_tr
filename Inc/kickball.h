#ifndef _KICKBALL_H
#define _KICKBALL_H

typedef enum
{
    KICKBALL_NONE = 0,
    KICKBALL_MAGNET_TO_BOARD,
    KICKBALL_MAGNET_READY,
    KICKBALL_BOARD_TO_DOWN5,
    KICKBALL_BOARD_TO_DOWN10,
    KICKBALL_BOARD_TO_DOWN20,
    KICKBALL_BOARD_READY,
    KICKBALL_KICK
} KICKBALL_STATUS;

typedef enum
{
    MANUAL = 0,
    AUTO = 1
} CONTROL_MODE;

extern KICKBALL_STATUS kickball_status;
extern CONTROL_MODE Kickball_ControlMode;

extern float kickball_m2006_current;
extern float kickball_vesc_lossen_duty;
extern float kickball_vesc_pull_current5;
extern float kickball_vesc_pull_current10;
extern float kickball_vesc_pull_current20;

extern int kickball_prepare_flag; //外部置1 踢球准备
extern int kickball_ready_flag;   //内部置1 反馈给主控
extern int kickball_kick_flag;    //外部置1 执行踢球
//内部调试flag
extern int kickball_pull_magnet_flag; //cmd指令置1 开始拉电磁铁
extern int kickball_stop_magnet_flag; //cmd指令置1 停止拉电磁铁
//五球分数
extern int kickball_five_score[5];
//当前第几颗球
extern int kickball_num;
extern CONTROL_MODE Kickball_ControlMode;

void kickball_M2006_set_current(float current);
void kickball_VESC_set_loosen_duty(float duty);
void kickball_VESC_set_pull_current(float current);
void kickball_VESC_set_rpm(float rpm);

void kickball_set_magnet_status(int status);
int kickball_get_microswitch_status();
void kickball_set_status(KICKBALL_STATUS status);

void kickball_exe();

#endif //_KICKBALL_H