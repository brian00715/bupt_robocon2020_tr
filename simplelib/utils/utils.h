#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "simplelib_cfg.h"
#ifdef SL_UTILS
#include "math.h"

/**
 * @brief	宏数据处理函数
 * @note	切勿在传入带有副作用的参数.e.g. *p++
 */
#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max
#define Min(A, B)            ((A) <= (B) ? (A):(B))
#define Max(A, B)			 ((A) >= (B) ? (A):(B))
#define PI					3.1415926535898
#define SL_OK				 0
#define SL_ERROR			 1

typedef struct{
	float KP;
	float KD;
	float KI;
	float i;
	float last_err;
	float i_max;
	float last_d;
    float I_TIME;    //2018年7月9日 修改，增加积分时间，
                     //以前是作为宏定义，但不同PID的积分应该是不一样的                 
} PID_Struct;

float PID_Release(PID_Struct *PID,float target,float now);
void reset_PID(PID_Struct * s);
void PID_init();

float Angle_Limit_180(float angle);
float Angle_Limit_PI(float angle);
void Coordinate_System_Transform(float now[3],float now_in_target[3] , float target[3]);

#endif // SL_UTILS

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H */