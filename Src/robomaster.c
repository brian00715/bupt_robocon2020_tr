/***********************************************************
* @brief 大疆电机的驱动文件
***********************************************************/

#include "robomaster.h"
#include "can_utils.h"
#include "main.h"

RoboMaster robomaster[4];
int M2006_speed[4] = {0};
PID_Struct Robomaster_Speed_PID[4];    // = {1.4,0.9,0.6,0,0,5000,0,0.005};
PID_Struct Robomaster_Position_PID[4]; // = {0.13,0,0.082,0,0,5000,0,0.005};
int robomaster_flag = 0;
int ver_slide_error = 0;

void can_robomaster_rcv_1(can_msg *pRxMsg)
{
  //pRxMsg->StdId
  //这里id还没改
  static int first_flag = 1;
  if (first_flag == 1)
  {
    robomaster[0].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
    robomaster[0].offset_angle = robomaster[0].angle;
    first_flag = 0;
    robomaster[0].round_cnt = 0;
    return;
  }
  robomaster[0].last_angle = robomaster[0].angle;
  robomaster[0].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
  robomaster[0].speed_rpm = (int16_t)(pRxMsg->ui8[2] << 8 | pRxMsg->ui8[3]);
  robomaster[0].real_current = (pRxMsg->ui8[4] << 8 | pRxMsg->ui8[5]) * 5.f / 16384.f;

  if (robomaster[0].angle - robomaster[0].last_angle > 4096)
    robomaster[0].round_cnt--;
  else if (robomaster[0].angle - robomaster[0].last_angle < -4096)
    robomaster[0].round_cnt++;
  robomaster[0].total_angle = robomaster[0].round_cnt * 8192 + robomaster[0].angle - robomaster[0].offset_angle;
}

void can_robomaster_rcv_2(can_msg *pRxMsg)
{
  //pRxMsg->StdId
  //这里id还没改
  static int first_flag = 1;
  if (first_flag == 1)
  {
    robomaster[1].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
    robomaster[1].offset_angle = robomaster[1].angle;
    first_flag = 0;
    robomaster[1].round_cnt = 0;
    return;
  }
  robomaster[1].last_angle = robomaster[1].angle;
  robomaster[1].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
  robomaster[1].speed_rpm = (int16_t)(pRxMsg->ui8[2] << 8 | pRxMsg->ui8[3]);
  robomaster[1].real_current = (pRxMsg->ui8[4] << 8 | pRxMsg->ui8[5]) * 5.f / 16384.f;

  if (robomaster[1].angle - robomaster[1].last_angle > 4096)
    robomaster[1].round_cnt--;
  else if (robomaster[1].angle - robomaster[1].last_angle < -4096)
    robomaster[1].round_cnt++;
  robomaster[1].total_angle = robomaster[1].round_cnt * 8192 + robomaster[1].angle - robomaster[1].offset_angle;
}


/*电机驱动*/
void RoboconMaster_Control()
{
  float speed_out[4];
  for (int i = 0; i < 4; i++)
  {
    speed_out[i] = robomaster_speed_pid_control(i);
  }
  robomaster_set_current((int16_t)speed_out[0], (int16_t)speed_out[1], (int16_t)speed_out[2], (int16_t)speed_out[3]);
}

/*仅速度环*/
float robomaster_speed_pid_control(int id)
{
  float speed_out = 0;
  speed_out = PID_Release(&Robomaster_Speed_PID[id], M2006_speed[id], (float)robomaster[id].speed_rpm);
  //speed_out=PID_Release(&Robomaster_Speed_PID[id],(float)robomaster[id].target_speed,(float)robomaster[id].speed_rpm);
  return speed_out;
}

/*位置环*/
float robomaster_position_pid_control(int id)
{
  float speed_out = 0;
  float position_out = 0;
  position_out = PID_Release(&Robomaster_Position_PID[id],
                             (float)robomaster[id].target_position + ver_slide_error, (float)robomaster[id].total_angle);
  speed_out = PID_Release(&Robomaster_Speed_PID[id], position_out, (float)robomaster[id].speed_rpm);
  //speed_out=PID_Release(&Robomaster_Speed_PID[id],(float)robomaster[id].target_speed,(float)robomaster[id].speed_rpm);
  return speed_out;
}


/*电流环*/
void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  uint8_t Data[8];
  Data[0] = (iq1 >> 8);
  Data[1] = iq1;
  Data[2] = (iq2 >> 8);
  Data[3] = iq2;
  Data[4] = iq3 >> 8;
  Data[5] = iq3;
  Data[6] = iq4 >> 8;
  Data[7] = iq4;
  can_msg dji_data;
  for (int i = 0; i <= 7; i++)
  {
    dji_data.ui8[i] = Data[i];
  }
  can_send_msg(0x200, &dji_data); // can总线控制大疆电机
}

void M2006_init(int id)
{
  reset_PID(&Robomaster_Speed_PID[id]);
  reset_PID(&Robomaster_Position_PID[id]);
  Robomaster_Speed_PID[id].KP = 1.4;
  Robomaster_Speed_PID[id].KI = 0.9;
  Robomaster_Speed_PID[id].KD = 0.6;
  Robomaster_Speed_PID[id].i_max = 5000;
  Robomaster_Speed_PID[id].I_TIME = 0.005;

  Robomaster_Position_PID[id].KP = 0.13;
  Robomaster_Position_PID[id].KI = 0;
  Robomaster_Position_PID[id].KD = 0.082;
  Robomaster_Position_PID[id].i_max = 5000;
  Robomaster_Position_PID[id].I_TIME = 0.005;
}

void M3508_init(int id)
{
  reset_PID(&Robomaster_Speed_PID[id]);
  reset_PID(&Robomaster_Position_PID[id]);
  Robomaster_Speed_PID[id].KP = 1.6;
  Robomaster_Speed_PID[id].KI = 0.52;
  Robomaster_Speed_PID[id].KD = 0.6;
  Robomaster_Speed_PID[id].i_max = 5000;
  Robomaster_Speed_PID[id].I_TIME = 0.005;

  Robomaster_Position_PID[id].KP = 0.1;
  Robomaster_Position_PID[id].KI = 0;
  Robomaster_Position_PID[id].KD = 0.8;
  Robomaster_Position_PID[id].i_max = 5000;
  Robomaster_Position_PID[id].I_TIME = 0.005;
}
