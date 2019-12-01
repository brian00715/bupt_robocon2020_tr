/*******************************************************************************
Copyright:      BUPT
File name:      chassis.c
Description:    全向轮底盘控制代码
Author:         ZX & ZH & LEO
Version：       1.0
Data:           2019/12/01
*******************************************************************************/
#include "chassis.h"
#include "chassis_handle.h"
#include "laser.h"

Chassis chassis;
Chassis_Status chassis_status;
PID_Struct position_y_dir_pid = {3000,175000,0,0,0,5000,0,0.005};//速度方向控制
float Arrive_distance = 0.005;
//FIXME  角度
float ERR_angle_m2 = -PI/3 , ERR_angle_m1 = -PI/3 + 1*2*PI/3 , ERR_angle_m0 = -PI/3 + 2*2*PI/3; //三轮与全场定位模块安装偏角

/**Chassis初始化*/
void chassis_zero()
{
  chassis.fspeed = 0;    
  chassis.fangle = 0;    
  chassis.fturn = 0;
  chassis_status.go_to_point = 0;           //go_to_point_for_test函数控制变量，1为开启，0为关闭
  chassis_status.count = 0;                //跑点计数，初试为0
  chassis_status.trace_count = 0;
  chassis_status.run_point = 0;
  chassis_status.run_point_test = 0;
  chassis_status.vega_is_ready = 0;       //初始化前不ready
  chassis_status.is_begin = 1;
  
  //TODO chassis初始化的controlflag
  flag.chassis_control_flag = 0;//五毫秒跑一次
}

/**底盘起点坐标设置*/
void chassis_init_pos(float x,float y)
{
  chassis.vega_init_pos_x=0;
  chassis.vega_init_pos_y=0;
  chassis.vega_init_angle=0;
  chassis.vega_init_pos_x += x - chassis.pos_x;
  chassis.vega_init_pos_y += y - chassis.pos_y;
}

/**底盘初始化*/
void chassis_init(void)
{
  chassis_init_pos(points_pos0[0].x , points_pos0[0].y);
  chassis_zero();
}

/**底盘更新坐标*/
void chassis_update(void)
{ 
  chassis.pos_x = chassis.vega_pos_x/1000 + chassis.vega_init_pos_x;    //m
  chassis.pos_y = chassis.vega_pos_y/1000 + chassis.vega_init_pos_y;    //m
  chassis.angle = (chassis.vega_angle / 180.f) * PI + chassis.vega_init_angle;    //弧度

  chassis.speed_x = (chassis.pos_x - chassis.last_pos_x) / 0.005;    //m/s
  chassis.speed_y = (chassis.pos_y - chassis.last_pos_y) / 0.005;    //m/s
  chassis.speed_angle = (chassis.angle - chassis.last_angle) / 0.005;    //弧度/s
  chassis.now_speed = vec_model(vec_create(chassis.speed_x,chassis.speed_y));

  chassis.last_pos_x = chassis.pos_x;
  chassis.last_pos_y = chassis.pos_y;
  chassis.last_angle = chassis.angle;  
}

/**角度减法函数*/
float chassis_angle_subtract(float a, float b)
{
  float out = a - b;
  while(out > PI)
  {
    out -= 2 * PI;
  }
  while(out < - PI)
  {
    out += 2 * PI;
  }
  return out;
}

/**角度pid控制*/
float chassis_PID_Angle_Control(float target_angle)
{
  float chassis_turn_angle_KP = -1000;
  float chassis_turn_angle_KD = 0;
  float angle_err = chassis_angle_subtract(target_angle, chassis.angle); 
  static float angle_last_err = 0;
  float P_out = angle_err * chassis_turn_angle_KP;
  float D_out = (angle_last_err - angle_err) * chassis_turn_angle_KD;
  angle_last_err = angle_err;
  return P_out + D_out;
}

/**底盘驱动
*参数：angle 方向角; speed 速度; turn 自转方位角; ishandle 手柄;
*/
void chassis_gostraight(int speed , float angle, float turn, int is_handle)
{
  //FIXME  角度
  float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
  float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
  float Chassis_motor2 = -(speed*cos((ERR_angle_m2 + chassis.angle) - angle));
  
  float turn_output = 0;

  if(is_handle)
  {
    //TODO 为手柄的旋转加入pid
    turn_output = turn;//全场定位方向环  
  }
  else
  {
    turn_output = chassis_PID_Angle_Control(turn);
  }
  if(turn_output >350)//陀螺仪角度PID
  {
    turn_output = 350;
  }
  if(turn_output < -350)
  {
    turn_output = -350;
  }
  //TODO LASER 
  float laser_motor0= 0;
  float laser_motor1 = 0;
  float laser_motor2 = 0;
  if(laser_enable)
  {
    //FIXME  角度
    laser_motor0 = -(chassis.laser_pwm * cos((ERR_angle_m0 + chassis.angle) - chassis.laser_pwm_angle ));
    laser_motor1 = -(chassis.laser_pwm * cos((ERR_angle_m2 + chassis.angle) - chassis.laser_pwm_angle ));
    laser_motor2 = -(chassis.laser_pwm * cos((ERR_angle_m1 + chassis.angle) - chassis.laser_pwm_angle ));
  }
  else
  {
    laser_motor0 = 0;
    laser_motor1 = 0;
    laser_motor2 = 0;
  }
  //FIXME  角度
  motor_canset3speed((int)(Chassis_motor0 + turn_output + laser_motor0),
                     (int)(Chassis_motor2 + turn_output + laser_motor2),
                     (int)(Chassis_motor1 + turn_output + laser_motor1));
}

/**根据目标点更新新的速度方向
*参数：point_y 	 point_x  地面坐标系下目标点的位置
*返回值： 底盘的速度方向角
*/
float point_tracer_angle_return( float point_x , float point_y )
{
  float chassis_target_x = point_x - chassis.pos_x;  
  float chassis_target_y = point_y - chassis.pos_y;   //坐标系变换，在保证角度环（小车坐标系不变的情况下），将目标点由地面坐标系变换到小车坐标系中
  float mid = atan2f(chassis_target_y , chassis_target_x);
  float angle = mid;
  return angle;
}

float point_tracer_angle_return_static(float start_x, float start_y , float end_x , float end_y)
{
  float angle = atan2f( end_y-start_y , end_x-start_x );
  return angle;
}

/*速度曲线生成 直线，由于跑点比较密集，直接用直线生成速度就比较平滑了
*参数：float point_y 	 point_x  地面坐标系下目标点的位置 start_speed 开始时速度 final_speed结束时速度 max_speed最大速度
*返回值： 电机当前的速度
*说明: by zh
*/
int chassis_calculate_speed(float point_x , float point_y , int start_speed , int final_speed , int max_speed)
{
  float distance_to_target = sqrtf( (point_x - chassis.pos_x)*(point_x - chassis.pos_x) + (point_y - chassis.pos_y)*(point_y - chassis.pos_y) );
  int int_speed = (int)((final_speed - start_speed)*distance_to_target + start_speed);
  if(int_speed > max_speed) //速度限制
    int_speed = max_speed;
  else if(int_speed <= 0) 
    int_speed = 0;
  return int_speed;
}

/**到达点后，将各种flag置位(重置至跑点前的状态)
*参数：void
*返回值： void
*说明: 为point_tracer内部函数，配合point_tracer使用
*作者: zx
*/
void point_arrive()
{
  chassis_status.run_point = 0;
  chassis_status.count = 0;
  chassis.fspeed = 0;
  chassis_status.is_begin = 1;//开始下一段时使用
  laser_enable = 0;
  //chassis_status.trace_count = chassis_status.trace_count + 1;//开始下一段
}

/**go_to_point_for_test go_to函数，用于测试中的回到初始位置或者一些特殊位置
*参数：float point_X , float point_y 最终目标点的位置
*说明: 应该只会用于调试中，速度矢量方向改变的算法，其余逻辑与point_tracer完全一致。
*作者: zx
*/
void go_to_point_test(float point_x , float point_y)
{
  float distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  if( distance >= Arrive_distance ) //可以开始跑点，并且未到达目标点
  { 
    chassis.fangle = point_tracer_angle_return(point_x , point_y);
    int speed = (int)distance * 1000;
    if(speed > 450)
    {
      speed = 450;
    }
    //chassis.fspeed = chassis_calculate_speed(point_x ,point_y,150,0,450);1
    chassis.fspeed = speed;
    chassis_gostraight( chassis.fspeed , chassis.fangle , 0 , 0);
  }  
  else //到达目标点
  {
    chassis_status.go_to_point = 0;
    uprintf("arrive:%f,%f,%f,%f\r\n",chassis.pos_x, chassis.pos_y, chassis.angle, chassis.laser_angle); 
    chassis_gostraight( 0 , 0 , 0 , 0);
    //laser_enable = 0;
  }    
  return;
}

/*vector_track_ctrl 用于使速度平滑
*参数：now,target为速度向量，direct为位移向量
*说明: 
*作者: zh
*/
void vector_track_ctrl(vec now, vec target, vec direct, float target_angle)//速度向量now,target     direct位移向量
{
  float chassis_vector_d = 0;
  if(vec_is_zero(target))return;
  chassis_vector_d = vec_mul(direct,vec_normal(target));//速度投影
  float vx_output = vec_model(target);
  float vy_output = -PID_Release(&position_y_dir_pid,0,chassis_vector_d);
  vec output = vec_add(vec_mul_i(vec_unit(target),vx_output), vec_mul_i(vec_normal(target),vy_output));
  
  chassis.fspeed = (int)vec_model(output);
  if(chassis.fspeed > 2100) chassis.fspeed = 2100;
  if(chassis.fspeed < -2100) chassis.fspeed = -2100;
  chassis.fangle = atan2(output.y,output.x);
  chassis_gostraight(chassis.fspeed ,chassis.fangle, target_angle, 0);
}

/*chassis_vector_test 用于测试
*参数：void
*说明: 
*作者: zh
*/
void chassis_vector_test()
{
  chassis.fturn = 0;  
  vec direct = vec_create(0 - chassis.pos_x,0 - chassis.pos_y);
  vec target = vec_create(500,0);
  vec now = vec_create(chassis.speed_x,chassis.speed_y);
  vector_track_ctrl(now, target, direct, 0);
}


/*chassis_go_track(float points_pos_x[],float points_pos_y[],int point_num) 用于跑点
*参数：points_pos_x[],points_pos_y[],要跑的轨迹点数组
*       int point_num，要跑的轨迹点个数
*说明: 该方法关键在于随时纠正小车的航向角，让跑点更平滑，避免了速度方向的突变
*作者: zh
*/
int chassis_go_track(Point points_pos[],int point_num)
{
  float scope_path = 0.03;
  double dis_to_next = 0;
  
  /*只在每一段路径第一次调用这个函数的时候，赋予chassis_status.point_num初值*/
  if(chassis_status.is_begin == 1)
  {
    chassis_status.point_num = point_num;
    chassis_status.is_begin = 0;
  }
  
  static double deltaX = 0, deltaY = 0;
  deltaX = chassis.pos_x - points_pos[chassis_status.count].x;
  deltaY = chassis.pos_y - points_pos[chassis_status.count].y; 
  
  dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
  
  if( point_num - chassis_status.count <= 20 && chassis_status.trace_count == 0)
  {
    //TODO 激光laser为偏角
    laser_enable = 1;
    calculate_laser_angle(laser_L.distance , laser_R.distance);
    chassis.vega_init_angle = chassis.laser_angle + points_pos[point_num].target_angle - chassis.angle;//计算偏移角
    float laser_ypos = (laser_L.distance + laser_R.distance) / 2;
    chassis.laser_pwm = (int)laser_ypos_correct(0.33, laser_ypos , &laser_ypos_pid);
    chassis.laser_pwm_angle = PI;//PI仅测试用
  }
  
  if(chassis.fspeed >= 700)///动态scope
  {
    scope_path = 0.16;
  }
  else
  {
    scope_path = 0.15;
  }
  
  int arriveJudge = 0;
  arriveJudge = (dis_to_next <= 0.005 && ( chassis_status.count >= chassis_status.point_num - 2)) 
    || (dis_to_next <= scope_path && chassis_status.count >= 0 && chassis_status.count < chassis_status.point_num - 2);
  if(arriveJudge)//判断经过此点 
  {
    uprintf("到%d点,(%f,%f,%f,%f)\r\n",chassis_status.count,chassis.pos_x,chassis.pos_y,chassis.angle,chassis.laser_angle);
    chassis_status.count += 1;
    
    if(chassis_status.count >= chassis_status.point_num - 1)//到达目的地
    {
      point_arrive();
      chassis_status.trace_count = (chassis_status.trace_count + 1) % 2;
      uprintf("到trace%d终点(%f,%f)\r\n",chassis_status.trace_count,chassis.pos_x,chassis.pos_y);
      chassis_gostraight(0,0,points_pos[point_num - 1].target_angle,0); 
      return 1;
    }
  }
  
  vec target, direct;
  target = vec_create((float)points_pos[chassis_status.count].speed * cos(points_pos[chassis_status.count].direct),  (float)points_pos[chassis_status.count].speed * sin(points_pos[chassis_status.count].direct));
  direct = vec_create(points_pos[chassis_status.count].x - chassis.pos_x, points_pos[chassis_status.count].y - chassis.pos_y);
  
  if(chassis_status.count < chassis_status.point_num - 3)
  {
    vec now = vec_create(chassis.speed_x,chassis.speed_y);
    vector_track_ctrl(now, target, direct, points_pos[chassis_status.count].target_angle);
  }
  /*由于速度慢，最后三个点采用其它跑点方法*/
  else if(chassis_status.count >= chassis_status.point_num - 3 && chassis_status.count <= chassis_status.point_num - 2)
  {
    chassis.fangle = point_tracer_angle_return(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_speed(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y,60,50,60);
    chassis_gostraight(chassis.fspeed,chassis.fangle,points_pos[chassis_status.count].target_angle,0);
  }
  else if(chassis_status.count == chassis_status.point_num - 1)
  {
    chassis.fangle = point_tracer_angle_return(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_speed(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y,30,0,30);
    chassis_gostraight(chassis.fspeed,chassis.fangle,points_pos[chassis_status.count].target_angle,0);
  }
  
  return 0;
}

/*chassis_exe
*参数：无
*说明: 把代码放在一堆，为了简洁
*作者: zh
*/
void chassis_exe()
{
  chassis_update();

  if(flag.chassis_automate_flag == 1)
  {
    chassis_run_point(chassis_status.trace_count);
  }
  if(flag.chassis_handle_flag == 1)
  {
    chassis_handle_control();
  }
}

/*chassis_run_point
*参数：要跑的轨迹
*说明: 跑完轨迹后再执行其它动作
*作者: zh
*/
void chassis_run_point(int trace_count)
{
  if(chassis_status.run_point == 0) return;
  
  switch(trace_count)
  {
  case 0:
    if( chassis_go_track(points_pos0,123) == 1)
    {
    }
    break;
  case 1:
    if( chassis_go_track(points_pos1,123) == 1)
    {
    }
    break;
  case 2:
    if( chassis_go_track(points_pos2,123) == 1)
    {
    }
    break;
  case 3:
    if( chassis_go_track(points_pos3,123) == 1)
    {
    }
    break;
  case 4:
    if( chassis_go_track(points_pos4,123) == 1)
    {
    }
    break;
  case 5:
    if( chassis_go_track(points_pos5,123) == 1)
    {
    }
    break;
  case 6:
    if( chassis_go_track(points_pos6,123) == 1)
    {
    }
    break;
  case 7:
    if( chassis_go_track(points_pos7,123) == 1)
    {
    }
    break;
  case 8:
    if( chassis_go_track(points_pos8,123) == 1)
    {
    }
    break;
  case 9:
    if( chassis_go_track(points_pos9,123) == 1)
    {
    }
    break;
  }
}