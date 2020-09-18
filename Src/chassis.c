/*******************************************************************************
Copyright:      BUPT
File name:      chassis.c
Description:    全向轮底盘控制代码
Author:         ZX & ZH & LEO
Version：       1.0
Data:           2019/12/09
*******************************************************************************/
#include "chassis.h"
#include "chassis_handle.h"
#include "laser.h"
#include "point_parser.h"

#define MAX_CHASSIS_MOVE_SPEED 500  //! chassis_move中对移动速度做限幅
#define MAX_CHASSIS_ANGLE_SPEED 350 //! chassis_move中对自转速度做限幅

Chassis chassis;
Chassis_Status chassis_status;
// 500 0 走直线
PID_Struct y_pid = {3000, 125000, 0, 0, 0, 5000, 0, 0.005}; //速度方向控制
PID_Struct angle_pid = {1000, 0, 0, 0, 0, 5000, 0, 0.005};  //偏高角控制

float Arrive_distance = 0.005;
/*****************************初始化*************************/

/**底盘状态机初始化*/
void chassis_init_status()
{
  chassis_status.go_to_point = -1; // go_to_point_for_test函数控制变量，1为开启，0为关闭
  chassis_status.count = 0;        // 跑点计数，初试为0
  chassis_status.trace_count = 0;
  chassis_status.run_point = 0;
  chassis_status.run_point_test = 0;
  chassis_status.vega_is_ready = 0; // 初始化前不ready
  chassis_status.is_begin = 1;
  flag.chassis_control_flag = 0; // 控制周期5ms
}

/**底盘位置初始化*/
void chassis_init_pos(float x, float y)
{
  chassis.fspeed = 0;
  chassis.fangle = 0;
  chassis.fturn = 0;
  chassis.vega_init_pos_x = 0;
  chassis.vega_init_pos_y = 0;
  chassis.vega_init_angle = 0;
  chassis.vega_init_pos_x += x - chassis.pos_x;
  chassis.vega_init_pos_y += y - chassis.pos_y;
}

/**底盘总初始化*/
void chassis_init(void)
{
  // chassis_init_pos(points_pos0[1].x , points_pos0[1].y);
  chassis_init_pos(0, 0);
  chassis_init_status();
}

/****************************计算**************************/

/**
 * @brief 交换x、y坐标轴的函数，同时修正0度方向，由于轨迹规划上位机坐标轴与实际坐标轴不符合而使用
 * @param points_pos 待交换轨迹集
 * @param point_num point_num
 * @author zohycao
 **/
void chassis_swap_xy(Point points_pos[], int point_num)
{
  int i;
  float temp_pos;
  for (i = 0; i < point_num; i++)
  {
    temp_pos = points_pos[i].x;
    points_pos[i].x = points_pos[i].y;
    points_pos[i].y = temp_pos;
    points_pos[i].direct = -points_pos[i].direct;
    points_pos[i].direct += PI / 2;
  }
  return;
}

/**计算到目标点的角度*/
float chassis_calculate_traceangle(float point_x, float point_y)
{
  return Angle_Between_Points(chassis.pos_x, chassis.pos_y, point_x, point_y);
}

/**计算直线跑点速度*/
int chassis_calculate_linespeed(float point_x, float point_y, int start_speed, int final_speed, int max_speed)
{
  float distance_to_target = 1000 * sqrtf((point_x - chassis.pos_x) * (point_x - chassis.pos_x) + 
                                          (point_y - chassis.pos_y) * (point_y - chassis.pos_y));

  //int int_speed = (int)((start_speed - final_speed)*distance_to_target + final_speed);

  int int_speed = (int)distance_to_target;  // 直接用

  // if(int_speed < 0) int_speed = -int_speed;
  Limit_From_To(int_speed, 0, max_speed);
  return int_speed;
}




/***********************************【驱动】***********************************
* 控制架构为：
* 顶层驱动（跑所有点）→上层驱动（当前点跑到下一个目标点）→中层驱动（跑向量，如何跑到下一个点）→
  底层驱动（跑速度，实时速度与方向调整）
*/


/**
 * @brief 底盘电机驱动（设置占空比）
 * @param s1 s2 s3 三个电机的占空比
 **/
void chassis_canset_motorduty(int s1, int s2, int s3)
{
  can_msg can_TX_data[3];
  can_TX_data[0].in[0] = 0;
  can_TX_data[1].in[0] = 0;
  can_TX_data[2].in[0] = 0;
  can_TX_data[0].in[1] = s1;
  can_TX_data[1].in[1] = s2;
  can_TX_data[2].in[1] = s3;

  can_send_msg(send_id.motor0_id, &can_TX_data[0]);
  can_send_msg(send_id.motor1_id, &can_TX_data[1]);
  can_send_msg(send_id.motor2_id, &can_TX_data[2]);
}

/**
 * @brief 底盘电机驱动（设置速度）
 * @param s1 s2 s3 三个电机的速度
 **/
void chassis_canset_motorspeed(int s1, int s2, int s3)
{
  can_msg can_TX_data[3];

  can_TX_data[0].in[0] = 1;
  can_TX_data[1].in[0] = 1;
  can_TX_data[2].in[0] = 1;
  can_TX_data[0].in[1] = s1;
  can_TX_data[1].in[1] = s2;
  can_TX_data[2].in[1] = s3;

  can_send_msg(send_id.motor0_id, &can_TX_data[0]);
  can_send_msg(send_id.motor1_id, &can_TX_data[1]);
  can_send_msg(send_id.motor2_id, &can_TX_data[2]);
}

/**
 * @brief  底盘底层驱动(跑速度)
 * @param speed 速度
 * @param direction 速度方向
 * @param target_angle 偏航角
 **/
void chassis_move(int speed, float direction, float target_angle)
{
  float ERR_angle_m2 = PI / 3, ERR_angle_m1 = -PI / 3, ERR_angle_m0 = PI; //三轮与全场定位模块安装偏角

  Limit(speed, MAX_CHASSIS_MOVE_SPEED);

  float speed_out_0 = -(speed * cos((ERR_angle_m0 + chassis.angle) - direction));
  float speed_out_1 = -(speed * cos((ERR_angle_m1 + chassis.angle) - direction));
  float speed_out_2 = -(speed * cos((ERR_angle_m2 + chassis.angle) - direction));
  float angle_output = 0;
  // 此处将angle_output改为了两种，即自动时为偏航角，手动时为自旋角速度，后续需改成根据MODE再判断，
  if (flag.chassis_auto_flag == 1 && flag.chassis_handle_flag == 0)
  {
    angle_output = -PID_Release(&angle_pid, target_angle, chassis.angle);
    Limit(angle_output, MAX_CHASSIS_ANGLE_SPEED);
  }
  else if (flag.chassis_auto_flag == 0 && flag.chassis_handle_flag == 1)
  {
    angle_output = target_angle;
    Limit(angle_output, MAX_CHASSIS_ANGLE_SPEED);
  }

  float motor0 = speed_out_0 + angle_output;
  float motor1 = speed_out_1 + angle_output;
  float motor2 = speed_out_2 + angle_output;
  chassis_canset_motorspeed((int)motor0, (int)motor1, (int)motor2);
}


vec output;
vec nor;
float vx_output;
float vy_output;
float vector_d = 0;
/**
 * @brief 底盘中层驱动(跑向量):now_speed_vec 当前速度;
 * @param target_speed_vec 目标速度;
 * @param distance_vec 位移向量; 
 * @param target_angle 偏航角
 **/
void chassis_move_vector(vec now_speed_vec, vec target_speed_vec, vec distance_vec, float target_angle)
{
  if (vec_is_zero(target_speed_vec))
  {
    //TODO 目标点速度为0无法运动 需修改
    chassis_move(0, 0, chassis.angle);
  }
  else
  {

    vector_d = vec_mul(distance_vec, vec_normal(target_speed_vec)); //速度投影
                                                                    //:pid控制量改为distance
                                                                    //vector_d = vec_model(distance_vec);
    vx_output = (float)vec_model(target_speed_vec);
    vy_output = (float)-PID_Release(&y_pid, 0, vector_d);
    nor = vec_mul_i(vec_normal(target_speed_vec), vy_output);
    output = vec_add(target_speed_vec, nor); //vec_mul_i(vec_normal(target_speed_vec),vy_output));

    chassis.fspeed = (int)vec_model(output);
    chassis.fangle = atan2(output.y, output.x);
    chassis_move(chassis.fspeed, chassis.fangle, target_angle);
  }
}

/**
 * @brief  底盘上层驱动(跑轨迹)
 * @param points_pos 轨迹点集
 * @param point_num 轨迹点数
 **/
int chassis_move_trace(Point points_pos[], int point_num)
{
  /*只在每一段路径第一次调用这个函数的时候，赋予chassis_status.point_num初值*/
  if (chassis_status.is_begin == 1)
  {
    chassis_status.point_num = point_num;
    chassis_status.is_begin = 0;
  }

  static vec now_speed_vec, target_speed_vec, distance_vec;
  now_speed_vec = vec_create(chassis.speed_x, chassis.speed_y);
  target_speed_vec = vec_create((float)points_pos[chassis_status.count].speed * cos(points_pos[chassis_status.count].direct),
                                (float)points_pos[chassis_status.count].speed * sin(points_pos[chassis_status.count].direct));
  distance_vec = vec_create(points_pos[chassis_status.count].x - chassis.pos_x,
                            points_pos[chassis_status.count].y - chassis.pos_y);

  double distance_to_next = vec_model(distance_vec);

  float dynamic_arrive_distance = 6 * Arrive_distance; //可修改
  int arriveJudge = (distance_to_next <= Arrive_distance || distance_to_next <= dynamic_arrive_distance);

  if (arriveJudge && (chassis_status.count <= (chassis_status.point_num - 2))) //判断经过此点
  {
    uprintf("%d---(%f,%f,%f)\r\n", chassis_status.count, chassis.pos_x, chassis.pos_y, chassis.angle);
    chassis_status.count += 1;

    // if(chassis_status.count >= chassis_status.point_num - 3){ //到达目的地
    //   chassis_finish_onetrace();
    //   uprintf("到trace%d终点(%f,%f)\r\n",chassis_status.trace_count,chassis.pos_x,chassis.pos_y);
    //   chassis_move(0,0,points_pos[point_num - 1].target_angle);
    //   return 1;
    // }
  }

  if (chassis_status.count < chassis_status.point_num - 3)
  {
    //  _chassis_move_vector_2(now_speed_vec, target_speed_vec, distance_vec, points_pos[chassis_status.count].target_angle);
    chassis_move_vector(now_speed_vec, target_speed_vec, distance_vec, points_pos[chassis_status.count].target_angle);
  }

  /*由于速度慢，*最后三个点*采用其它跑点方法*/
  else if (chassis_status.count == chassis_status.point_num - 3)
  {
    // chassis_status.trace_count = -1;
    chassis.fangle = chassis_calculate_traceangle(points_pos[chassis_status.count].x,
                                                  points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_linespeed(points_pos[chassis_status.count].x,
                                                 points_pos[chassis_status.count].y, 60, 40, 60);
    chassis_move(chassis.fspeed, chassis.fangle, points_pos[chassis_status.count].target_angle);
  }

  else if (chassis_status.count == chassis_status.point_num - 2)
  {
    //  chassis_status.trace_count = -1;
    chassis.fangle = chassis_calculate_traceangle(points_pos[chassis_status.count].x,
                                                  points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_linespeed(points_pos[chassis_status.count].x,
                                                 points_pos[chassis_status.count].y, 40, 20, 40);
    chassis_move(chassis.fspeed, chassis.fangle, points_pos[chassis_status.count].target_angle);
    chassis_goto_point(points_pos[chassis_status.count].x, points_pos[chassis_status.count].y);
  }
  else if (chassis_status.count == chassis_status.point_num - 1)
  {
    chassis.fangle = chassis_calculate_traceangle(points_pos[chassis_status.count].x,
                                                  points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_linespeed(points_pos[chassis_status.count].x,
                                                 points_pos[chassis_status.count].y, 20, 0, 20);
    chassis_move(chassis.fspeed, chassis.fangle, points_pos[chassis_status.count].target_angle);
    chassis_finish_onetrace();
    return 1;
  }
  return 0;
}

/**
 * @brief  底盘顶层驱动(跑全场轨迹)
 * @param trace_num 
 **/
void chassis_move_traces(int trace_num)
{
  // vec test0 = {0};
  // vec test;
  switch (trace_num)
  {
    case -2: //设置底盘电机占空比为0
      chassis_canset_motorduty(0, 0, 0);
      break;
    case -1: //设置底盘电机速度为0
      chassis_canset_motorspeed(0, 0, 0);
      break;
    case 0: 
      break;
    case 1: //小车移动到初始点
      chassis_GotoPoint(0, 0, 0);
      break;
    case 2: //跑测试点
      chassis_GotoPoint(0.3, 0, 0);
      break;
    case 3:
      chassis_GotoPoint(0, 0.3, 0);
      break;
    case 4: 
      chassis_GotoPoint(0, 0, 0.5);
      break;
    case 5:
      chassis_goto_point(0.5, 0);
      break;
    case 6:
      chassis_GotoPoint(0.5, 0.5, 0);
      break;
    case 7:
      chassis_GotoPoint(0.2, 0, 0.05);
      break;
    case 8:
      chassis_GotoPoint(0.6, 0.3, 0.2);
      break;
    case 9:
      chassis_GotoPoint(0, 0.6, 0.1);
      break;
    default:
      break;
  }
  /*旧车代码（已不用）
  switch (trace_num)
  {
  case -2:
    chassis_move(0, 0, 0);
    break;
  case -1:
    chassis_canset_motorspeed(0, 0, 0);
    break;
  case 0:
    break;
  case 1: //起点-接球
    if (chassis_status.count == chassis_status.point_num - 2)
    {
      chassis_status.trace_count = -1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
    }
    chassis_move_trace(points_pos0, 81);

    break;
  case 2: //接球-达阵1  mode4 6
    if (chassis_status.count == chassis_status.point_num - 1)
    {
      chassis_status.trace_count = -1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
    }
    chassis_move_trace(points_pos1, 123); //点数带更改

    break;
  case 3: // 接球-达阵2 mode4 7
    if (chassis_status.count == chassis_status.point_num - 1)
    {
      chassis_status.trace_count = -1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
    }
    chassis_move_trace(points_pos2, 123); //点数带更改

    break;
  case 4: // 接球-达阵3 mode4 8
    if (chassis_status.count == chassis_status.point_num - 1)
    {
      chassis_status.trace_count = -1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
    }
    chassis_move_trace(points_pos3, 123); //点数带更改

    break;
  case 5: // 接球-达阵4 mode4 9
    if (chassis_status.count == chassis_status.point_num - 1)
    {
      chassis_status.trace_count = -1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
    }
    chassis_move_trace(points_pos4, 123); //点数带更改

    break;
  case 6: // 达阵到接球 mode4 1

    chassis_touchdown_back();
    break;
  case 7: // 遥控后到接球 mode4 0

    chassis_getball_back();
    break;

  case 10:
    chassis_goto_point(0, 0);
  default:
    break;
  }
  */
}





/****************************测试**************************/

/**测试用：随距离减速到某一目标点*/
void chassis_goto_point(float point_x, float point_y)
{
  float distance = sqrtf((chassis.pos_x - point_x) * (chassis.pos_x - point_x) + (chassis.pos_y - point_y) * (chassis.pos_y - point_y));
  if (distance >= Arrive_distance)
  {
    chassis_status.go_to_point = 1;
    chassis.fangle = chassis_calculate_traceangle(point_x, point_y);
    chassis.fspeed = chassis_calculate_linespeed(point_x, point_y, 150, 0, 300);
    chassis_move(chassis.fspeed, chassis.fangle, 0);
  }
  else
  {
    chassis_status.go_to_point = 0;
    uprintf("arrive:%f,%f,%f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
    chassis_move(0, 0, chassis.angle);
  }
  return;
}

/**
  * @brief 底盘直线跑点
  * @param <point_x> 目标点横坐标
  * @param <point_y> 目标点纵坐标
  * @param <target_angle> 底盘目标位姿
  **/
 void chassis_GotoPoint(float point_x, float point_y, float target_angle)
{
  float distance = sqrtf((chassis.pos_x - point_x) * (chassis.pos_x - point_x) + (chassis.pos_y - point_y) * (chassis.pos_y - point_y));
  if (distance >= Arrive_distance)
  {
    chassis_status.go_to_point = 1;
    chassis.fangle = chassis_calculate_traceangle(point_x, point_y);
    chassis.fspeed = chassis_calculate_linespeed(point_x, point_y, 150, 0, 300);
    //chassis.fturn = target_angle;
    chassis_move(chassis.fspeed, chassis.fangle, target_angle);
  }
  else
  {
    chassis_status.go_to_point = 0;
    // uprintf("arrive:%f,%f,%f\r\n", chassis.pos_x, chassis.pos_y, target_angle);
    chassis_move(0, 0, target_angle);
  }
  return;
}

/**测试用：按向量跑*/
void chassis_goto_vector(vec target_position)
{
  chassis.fturn = 0;
  vec distance_vec = vec_create(target_position.x - chassis.pos_x, target_position.y - chassis.pos_y);
  vec target_speed_vec = vec_create(0, 0);
  vec now_speed_vec = vec_create(chassis.speed_x, chassis.speed_y);
  chassis_move_vector(now_speed_vec, target_speed_vec, distance_vec, chassis.angle);
}




/****************************状态&执行**************************/

/**跑完每一段路径后,标志位的改变*/
void chassis_finish_onetrace()
{
  //TODO: 内容待更改
  chassis_status.run_point = 0;
  chassis_status.count = 0;
  chassis.fspeed = 0;
  chassis_status.is_begin = 1; //开始下一段时使用
  flag.chassis_laser_flag = 0;
  // chassis_status.trace_count += 1;
  //测试用
  chassis_status.trace_count = -1;
}

/**更新底盘位姿*/
void chassis_pos_update()
{
  // 使用全场定位装置更新底盘坐标
  chassis.pos_x = chassis.vega_pos_x + chassis.vega_init_pos_x;                //m
  chassis.pos_y = chassis.vega_pos_y + chassis.vega_init_pos_y;                //m
  chassis.angle = (chassis.vega_angle / 180.f) * PI + chassis.vega_init_angle; //弧度

  // 全场定位can的发送时间间隔为5ms，因此用坐标差除以0.005就是瞬时速度
  chassis.speed_x = (chassis.pos_x - chassis.last_pos_x) / 0.005;     // m/s
  chassis.speed_y = (chassis.pos_y - chassis.last_pos_y) / 0.005;     // m/s
  chassis.speed_angle = (chassis.angle - chassis.last_angle) / 0.005; // 弧度/s
  chassis.now_speed = vec_model(vec_create(chassis.speed_x, chassis.speed_y)); // 合成速度

  chassis.last_pos_x = chassis.pos_x;
  chassis.last_pos_y = chassis.pos_y;
  chassis.last_angle = chassis.angle;
}

/**底盘执行函数*/
void chassis_exe()
{
  chassis_pos_update();  // 更新底盘位姿
  if (flag.chassis_auto_flag == 1 && flag.chassis_handle_flag == 0)  // 使用自动控制
  {
    chassis_move_traces(chassis_status.trace_count);
  }
  if (flag.chassis_handle_flag == 1 && flag.chassis_auto_flag == 0)  // 使用手动控制
  {
    handle_exe();
  }
}

/********************************************跑轨迹 改*******************************
PID_Struct tang_pid = {3000,175000,0,0,0,5000,0,0.005}; 
PID_Struct norm_pid = {3000,175000,0,0,0,5000,0,0.005}; 
void _chassis_move_vector(vec now_speed_vec, vec target_speed_vec, vec distance_vec, float target_angle){
  // int is_now_speed_zero = vec_is_zero(now_speed_vec);
  // int is_target_speed_zero = vec_is_zero(target_speed_vec);
  // int is_distance_zero = vec_is_zero(distance_vec);
  // if(is_distance_zero) return;
  
  vec tang_vec = vec_unit(target_speed_vec);
  vec norm_vec = vec_normal(tang_vec);

  float target_on_tang = vec_mul(target_speed_vec,tang_vec);
  float target_on_norm = vec_mul(target_speed_vec,norm_vec);  
  float speed_on_tang = vec_mul(now_speed_vec,tang_vec);
  float speed_on_norm = vec_mul(now_speed_vec,norm_vec);
  float distance_on_tang = vec_mul(distance_vec,norm_vec);
  float distance_on_norm = vec_mul(distance_vec,norm_vec);

  int speedup_on_tang = -1;
  int speedup_on_norm = -1;
  if(target_on_tang>speed_on_tang) {speedup_on_tang=1;}
  if(target_on_norm>speed_on_norm) {speedup_on_norm=1;}

  float output_on_tang = target_on_tang - speedup_on_tang * PID_Release(&tang_pid,0,distance_on_tang);
  float output_on_norm = target_on_norm - speedup_on_norm * PID_Release(&norm_pid,0,distance_on_norm);

  vec output = vec_add(vec_mul_i(norm_vec,output_on_norm),vec_mul_i(tang_vec,output_on_tang));
  
  chassis.fspeed = (int)vec_model(output);
  chassis.fangle = atan2(output.y,output.x);
  chassis_move(chassis.fspeed ,chassis.fangle, target_angle);
}
void _chassis_move_vector_2(vec now_speed_vec, vec target_speed_vec, vec distance_vec, float target_angle){

  vec target_tang_vec = vec_unit(target_speed_vec);
  vec target_norm_vec = vec_normal(target_tang_vec);
  vec distance_tang_vec = vec_unit(distance_vec);

  float angle_dis_tar = vec_angle_sub(target_tang_vec,distance_tang_vec);
  float output_on_tang = vec_model(target_speed_vec);
  float output_on_norm = -tan(angle_dis_tar) * output_on_tang;

  vec output = vec_add(vec_mul_i(target_norm_vec,output_on_norm),vec_mul_i(target_tang_vec,output_on_tang));
  
  chassis.fspeed = (int)vec_model(output);
  chassis.fangle = atan2(output.y,output.x);
  chassis_move(chassis.fspeed ,chassis.fangle, target_angle);


}

********************************************跑轨迹 改*******************************/

void chassis_touchdown_back()
{
  static int arrive_point = 0;

  if (arrive_point == 0)
  {
    chassis_goto_point(4.059, 3.802); //!
  }

  if (chassis_status.go_to_point == 0)
  {
    arrive_point = 1;
    chassis_status.go_to_point = -1;
  }

  if (arrive_point == 1)
  {
    chassis_move(0, 0, 0);
    if (chassis.angle < 0.087 && chassis.angle > -0.087)
    {
      arrive_point = 2;
    }
  }

  if (arrive_point == 2)
  {
    chassis_move_trace(points_pos4, 83);
    if (chassis_status.count == chassis_status.point_num - 1)
    {
      chassis_status.trace_count = -1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
      arrive_point = 0;
    }
  }
}

void chassis_getball_back()
{
  static int arrive_point = 0;

  if (arrive_point == 0)
  {
    chassis_goto_point(4.976, 0.350); //!
  }

  if (chassis_status.go_to_point == 0)
  {
    arrive_point = 1;
    chassis_status.go_to_point = -1;
  }

  if (arrive_point == 1)
  {
    chassis_move(0, 0, 0);
    if (chassis.angle < 0.087 && chassis.angle > -0.087)
    {
      chassis_status.trace_count = -1;
      arrive_point = 0;
    }
  }
}