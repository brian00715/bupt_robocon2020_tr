#ifndef __robomaster_H
#define __robomaster_H
#ifdef __cplusplus
extern "C" {
#endif
  
#include "utils.h"
#include "can_utils.h"  
  


/*Struct Area*/
typedef enum{
  _M2006,
  _M3508
}ROBOMASTER_TYPE;

typedef struct{
  int16_t	 	speed_rpm;
  float  	  real_current;
  int16_t  	given_current;
  uint8_t  	hall;
  uint16_t 	angle;				//abs angle range:[0,8191]
  uint16_t 	last_angle;	  //abs angle range:[0,8191]
  uint16_t	offset_angle;
  int32_t		round_cnt;
  int32_t		total_angle;
  int target_position;
  int target_speed;
  ROBOMASTER_TYPE type;
}RoboMaster;

/*Variable Area*/
extern int ver_slide_error;
extern int M2006_speed[4];


extern RoboMaster robomaster[4];
extern PID_Struct Robomaster_Speed_PID[4];
extern PID_Struct Robomaster_Position_PID[4];
extern int robomaster_flag;
/*Function Area*/
void can_robomaster_rcv_1( can_msg *pRxMsg);
void can_robomaster_rcv_2( can_msg *pRxMsg);
void RoboconMaster_Control();
float robomaster_position_pid_control(int id);
float robomaster_speed_pid_control(int id);
void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void M3508_init(int id);
void M2006_init(int id);


void m2006_exe();
extern float door_current;
extern float tick_current;


#ifdef __cplusplus
}
#endif
#endif /*__ robomaster_H */