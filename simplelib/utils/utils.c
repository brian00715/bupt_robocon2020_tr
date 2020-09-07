#include "utils.h"

void PID_init()
{
}

float PID_Release(PID_Struct *PID, float target, float now)
{
  float err;
  float err_dt;
  float result;

  err = target - now;
  err_dt = err - PID->last_err; 

  err_dt *= 0.384f;
  err_dt += PID->last_d * 0.615f;  //低通滤波

  PID->last_err = err;

  PID->i += err * PID->I_TIME;  // 积分量

  Limit(PID->i, PID->i_max);  // 限制积分量大小
  PID->last_d = err_dt;

  result = err * PID->KP + err_dt * PID->KD + PID->i * PID->KI;
  return result;
}

void reset_PID(PID_Struct *s)
{
  s->i = 0;
  s->last_err = 0;
  s->last_d = 0;
}

float Angle_Limit_180(float angle)
{
  while (angle > 180)
  {
    angle -= 360;
  }
  while (angle <= -180)
  {
    angle += 360;
  }
  return angle;
}
float Angle_Limit_PI(float angle)
{
  while (angle > PI)
  {
    angle -= 2 * PI;
  }
  while (angle <= -PI)
  {
    angle += 2 * PI;
  }
  return angle;
}

/**??????*/
float Angle_Subtract(float a, float b)
{
  float out = a - b;
  return Angle_Limit_PI(out);
}

float Angle_Between_Points(float start_x, float start_y, float end_x, float end_y)
{
  float angle = 0;
  if (start_x == end_x)
  {
    if (start_y < end_y)
      angle = PI / 2;
    if (start_y = end_y)
      angle = 0;
    if (start_y > end_y)
      angle = -PI / 2;
  }
  else
  {
    angle = atan2f(end_y - start_y, end_x - start_x);
  }
  return angle;
}

/**坐标系矩阵变换，
 * now 全场定位返回坐标
 * now_in_target 全场定位坐标相对于世界坐标的位置，角度为X轴偏角，逆时针为正
 * target 最终坐标
*/
void Coordinate_System_Transform(float now[3], float now_in_target[3], float target[3])
{
  float c = cos(-now_in_target[2]);
  float s = sin(-now_in_target[2]);
  target[0] = now[0] * c + now[1] * s - now_in_target[0];
  target[1] = now[0] * (-s) + now[1] * c - now_in_target[1];
  target[2] = Angle_Limit_PI(now[2] - now_in_target[2]);
}
