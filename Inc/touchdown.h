#ifndef _TOUCHDOWN_H
#define _TOUCHDOWN_H

typedef enum{
    TOUCHDOWN_NONE=0,
    TOUCHDOWN_GETBALL,
    TOUCHDOWN_TRY,
    TOUCHDOWN_FINISH
}TOUCHDOWN_STATUS;

void touchdown_cylinder_push();
void touchdown_cylinder_pull();
void touchdown_m2006_open(float current);
void touchdown_m2006_close(float current);

extern int touchdown_ready_flag;            //接到球置1 推出球置0 外部读取有无球
extern int touchdown_try_flag;              //外部置1 推球

extern int touchdown_try_finish_flag ;     

extern TOUCHDOWN_STATUS touchdown_status;

extern float touchdown_open_current;
extern float touchdown_close_current;
extern int touchdown_auto_flag;

void touchdown_exe();



#endif //_TOUCHDOWN_H