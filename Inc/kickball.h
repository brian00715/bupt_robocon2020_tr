#ifndef _KICKBALL_H
#define _KICKBALL_H

#define KICKBALL_GEN 2 // 选择踢球装置的代数,置1选择第一代，置2选择第二代

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
    KICKBALL2_NONE = 0,
    KICKBALL2_READY, // 运动到位后状态机置为READY（全场定位）
    KICKBALL2_KICK,
    KICKBALL2_STOP_ROTATE // 根据编码器的角度值确定什么时候停止
} KICKBALL2_STATUS;

typedef enum
{
    MANUAL = 0,
    AUTO = 1
} CONTROL_MODE;

//======================第一代踢球======================
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

void kickball_M2006_set_current(float current);
void kickball_VESC_set_loosen_duty(float duty);
void kickball_VESC_set_pull_current(float current);
void kickball_VESC_set_rpm(float rpm);
void kickball_set_magnet_status(int status);
int kickball_get_microswitch_status();
void kickball_set_status(KICKBALL_STATUS status);
void kickball_exe();

//======================第二代踢球======================
extern KICKBALL2_STATUS kickball2_status;
extern CONTROL_MODE Kickball2_ControlMode;
extern float Kickball2_KickCurrent; // CMD设置或使用默认值-5
extern int Kickball2_Ready_Flag;    // 由全场定位置1
extern int Kickball2_Kick_Flag;     // 由CMD置1
extern int Kickball2_StopRotate_Flag;     // 根据旋转角度值来确定,或使用CMD
extern int Kickball2_BallNum;
void Kickball2_StateMachine();
void Kickball2_SetState(KICKBALL2_STATUS status);
void Kickball2_EXE();

#endif //_KICKBALL_H