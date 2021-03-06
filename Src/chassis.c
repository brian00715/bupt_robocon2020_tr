/*******************************************************************************
Copyright:      BUPT
File name:      chassis.c
Description:    全向轮底盘控制代码
Author:         ZX & ZH & LEO & smkk
Version：       2.0
Data:           2020/10/13
*******************************************************************************/
#include "chassis.h"
#include "chassis_handle.h"
#include "laser.h"
#include "point_parser.h"

#define MAX_CHASSIS_MOVE_SPEED 500  //! chassis_move中对移动速度做限幅
#define MAX_CHASSIS_ANGLE_SPEED 100 //! chassis_move中对自转速度做限幅

Chassis chassis;
Chassis_Status chassis_status;
// 500 0 走直线
PID_Struct y_pid = {3000, 125000, 0, 0, 0, 5000, 0, 0.005}; //速度方向控制
PID_Struct angle_pid = {1000, 0, 0, 0, 0, 5000, 0, 0.005};  //偏航角控制

float ARRIVE_CIRCLE = 0.01; // 到达终点的范围阈值

/*****************************初始化*************************/
/**底盘状态机初始化*/
void chassis_init_status()
{
  chassis_status.go_to_point = -1; // go_to_point_for_test函数控制变量，1为开启，0为关闭
  chassis_status.count = 0;        // 跑点计数，初试为0
  chassis_status.trace_count = -2; // -2不执行任何操作
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
  Chassis_LockYaw_Value = 0;
  chassis_init_pos(0, 0);
  chassis_init_status();
}

extern int Handle_LeftRocker_Length;
int Chassis_LockYaw_Flag = 0;
float Chassis_LockYaw_Value = 0;
/**底盘执行函数*/
void chassis_exe()
{
  chassis_pos_update(); // 更新底盘位姿

  if (flag.chassis_auto_flag == 1 && flag.chassis_handle_flag == 0) // 使用自动控制
  {
    // Handle_LeftRockerLength并不干预flag.chassis_handle_flag，故手柄干预后直接按按键就可以继续跑点，无需切换模式
    // 如果自动跑点期间使用摇杆进行干预，则终止自动跑点,点集序号置-1
    // if (abs(Handle_LeftRocker_Length) > 10 || abs(Handle_RightRocker_Length > 108))
    // {
    //   chassis_status.trace_count = -1;
    // }
    chassis_move_traces(chassis_status.trace_count);
  }

  if (flag.chassis_handle_flag == 1 && flag.chassis_auto_flag == 0) // 使用手动控制
  {
    chassis_status.trace_count = -1;
    handle_exe(); // 使用手柄控制底盘，按键始终可用
  }
}

/**更新底盘位姿*/
void chassis_pos_update()
{
  // 使用全场定位装置更新底盘坐标
  Chassis_VegaDimTransform();
  chassis.pos_x = chassis.vega_pos_x + chassis.vega_init_pos_x;                //m
  chassis.pos_y = chassis.vega_pos_y + chassis.vega_init_pos_y;                //m
  chassis.angle = (chassis.vega_angle / 180.f) * PI + chassis.vega_init_angle; //角度转为弧度
  if (Chassis_ResetVegaOrigin_Flag)                                            // 开启原点重置
  {
    chassis.pos_x -= Chassis_OriginPosture.x;
    chassis.pos_y -= Chassis_OriginPosture.y;
    chassis.angle -= Chassis_OriginPosture.angle;
  }

  // 全场定位can的发送时间间隔为5ms，因此用坐标差除以0.005就是瞬时速度
  chassis.speed_x = (chassis.pos_x - chassis.last_pos_x) / 0.005;              // m/s
  chassis.speed_y = (chassis.pos_y - chassis.last_pos_y) / 0.005;              // m/s
  chassis.speed_angle = (chassis.angle - chassis.last_angle) / 0.005;          // 弧度/s
  chassis.now_speed = vec_model(vec_create(chassis.speed_x, chassis.speed_y)); // 合成速度

  chassis.last_pos_x = chassis.pos_x;
  chassis.last_pos_y = chassis.pos_y;
  chassis.last_angle = chassis.angle;
}

ChassisPosture Chassis_OriginPosture; // 相对原点位姿
int Chassis_ResetVegaOrigin_Flag = 0;
/*重置全场定位的原点*/
void Chassis_ResetVegaOrigin()
{
  // Chassis_OriginPosture.x = chassis.vega_pos_x; // 记录此时的坐标值，作为之后坐标的偏移补偿量
  // Chassis_OriginPosture.y = chassis.vega_pos_y;
  // Chassis_OriginPosture.angle = (chassis.vega_angle / 180.f) * PI;
  Chassis_OriginPosture.x = chassis.pos_x;
  Chassis_OriginPosture.y = chassis.pos_y;
  Chassis_OriginPosture.angle = chassis.angle;
  uprintf("--origin_x:%.6f origin_y:%.6f origin_angle:%.6f\r\n",
          Chassis_OriginPosture.x, Chassis_OriginPosture.y, Chassis_OriginPosture.angle);
}

/*全场定位坐标轴变换,在函数体中定义转换规则*/
void Chassis_VegaDimTransform()
{
  // float temp = chassis.vega_pos_y;
  // chassis.vega_pos_y = chassis.vega_pos_x;
  // chassis.vega_pos_x = -temp;
}

void Chassis_PrintPos()
{
  uprintf("--Chassis Pos:\r\n");
  uprintf("  x:%.6f  y:%.6f  angle:%.6f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
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

  // int int_speed = (int)((start_speed - final_speed) * distance_to_target + final_speed);

  int int_speed = (int)distance_to_target; // 直接用

  // if (int_speed < 0)
  // {
  //   int_speed = -int_speed;
  // }
  Limit_From_To(int_speed, 0, max_speed);
  return int_speed;
}

int Chassis_CalculateLineSpeedPlus(float start_x, float start_y, float point_x, float point_y,
                                   int start_speed, int final_speed, int max_speed)
{
  float distance_to_target = 1000 * sqrtf((point_x - chassis.pos_x) * (point_x - chassis.pos_x) +
                                          (point_y - chassis.pos_y) * (point_y - chassis.pos_y));
  float total_distance = 1000 * sqrtf((point_x - start_x) * (point_x - start_x) +
                                      (point_y - start_y) * (point_y - start_y));
  float distance_offset = fabs(total_distance - distance_to_target); // distance_offset单调递增
  int speed = 0;
  if (distance_offset < total_distance / 6) // 前1/6加速过程
  {
    speed = (int)((6 * (float)(max_speed - start_speed) / total_distance) * distance_offset + start_speed);
  }
  else if (distance_offset < total_distance * 4 / 6) // 匀速过程
  {
    speed = max_speed;
  }
  else if (distance_offset <= total_distance) //减速过程
  {
    // speed = (int)((4 * (float)(max_speed - final_speed) / total_distance) *
    //              (distance_offset - total_distance * 3 / 4) +
    //          final_speed);
    speed = (int)((6 / 2 * (float)(max_speed - final_speed) / total_distance) * distance_to_target);
  }
  Limit_From_To(speed, 0, max_speed);
  // uprintf("speed:%d distance_to_target:%.2f distance_offset:%.2f\r\n", speed, distance_to_target, distance_offset);
  return speed;
}

/***********************************【驱动】***********************************
* 控制架构为：
* 顶层驱动（跑所有点）→上层驱动（当前点跑到下一个目标点）→中层驱动（跑向量，如何跑到下一个点）→
* 底层驱动（跑速度，实时速度与方向调整）
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

  can_send_msg(send_id.motor2_id, &can_TX_data[2]);
  can_send_msg(send_id.motor0_id, &can_TX_data[0]);
  can_send_msg(send_id.motor1_id, &can_TX_data[1]);
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

  can_send_msg(send_id.motor1_id, &can_TX_data[1]);
  can_send_msg(send_id.motor2_id, &can_TX_data[2]);
  can_send_msg(send_id.motor0_id, &can_TX_data[0]);
}

 /**
 * @brief  底盘底层驱动(跑速度)
 * @param speed 速度
 * @param direction 速度方向
 * @param target_angle 偏航角/弧度
 **/
void chassis_move(int speed, float direction, float target_angle)
{
  float ERR_angle_m2 = PI, ERR_angle_m1 = PI / 3, ERR_angle_m0 = -PI / 3; //坐标轴与电机运动方向的夹角
  Limit(speed, MAX_CHASSIS_MOVE_SPEED);

  float absolute_angle_offset = (Chassis_PosMode == ABSOLUTE ? chassis.angle : 0); // 使用绝对坐标时，根据全场定位测得的偏航角进行补偿

  float speed_out_0 = -(speed * cos((ERR_angle_m0 + absolute_angle_offset) - direction));
  float speed_out_1 = -(speed * cos((ERR_angle_m1 + absolute_angle_offset) - direction));
  float speed_out_2 = -(speed * cos((ERR_angle_m2 + absolute_angle_offset) - direction));
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
    if (Chassis_LockYaw_Flag == 1) // 开启锁定偏航角
    {
      angle_output = -PID_Release(&angle_pid, target_angle, chassis.angle);
    }
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

  float dynamic_arrive_distance = 6 * ARRIVE_CIRCLE; //可修改
  int arriveJudge = (distance_to_next <= ARRIVE_CIRCLE || distance_to_next <= dynamic_arrive_distance);

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

float Chassis_ProperLaserDistance_BehindL = 0;
float Chassis_ProperLaserDistance_BehindR = 0;
float Chassis_ProperLaserDistance_Right = 0;
Point2D Chassis_MovePoint;
static int print_count = 1;
void chassis_move_traces(int trace_num)
{
  switch (trace_num)
  {
  case -1:
    // 占位，不进行任何操作
    Chassis_MovePoint.x = 0;
    Chassis_MovePoint.y = -0.756119; // 到达6米线前的距离,待调试
    break;
  case 0: // 6米线
    Kickball2_Ready_Flag = 0;
    Kickball2_Kick_Flag = 0;
    if (Chassis_AutoArrivedAtSpecifiedPoint_Flag == 0)
    {
      // chassis_goto_point(0, 0);
      Chassis_GoToPoint_Plus(0, 0, Chassis_LockYaw_Value);
      print_count = 1;
    }
    else // 到达终点
    {
      if (print_count)
      {
        uprintf("--arrive at 6m line.\r\n");
        print_count = 0;
      }
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
      Handle_LeftRocker_SpeedTransRatio = 1;
      DistanceToBallSocketOK_Flag = 0;
    }
    break;
  case 1: // 前进到5米线
    if (Chassis_AutoArrivedAtSpecifiedPoint_Flag == 0)
    {
      // chassis_goto_point(Chassis_MovePoint.x, Chassis_MovePoint.y);
      Chassis_GoToPoint_Plus(Chassis_MovePoint.x, Chassis_MovePoint.y, Chassis_LockYaw_Value);
      print_count = 1;
    }
    else // 到达终点
    {
      if (print_count)
      {
        uprintf("--distance to ball socket is ok.\r\n");
        print_count = 0;
      }
      Handle_LeftRocker_SpeedTransRatio = 0.25; // 到达5米线后调整左摇杆速度换算比例以进行微调
      Handle_RightRocker_SpeedTransRatio = 5;
      DistanceToBallSocketOK_Flag = 1;
      flag.chassis_auto_flag = 0;
      flag.chassis_handle_flag = 1;
    }
    break;
  case 2:
    break;
  case 3:
    break;
  default:
    break;
  }
}

/**
 * @brief 使用激光精确调整到踢球点
 **/
void Chassis_AdjustToKickPoint()
{

}

/*运动到6米线*/
void Chassis_MoveTo6m()
{
  Chassis_GoToPoint_Plus(0, chassis.vega_pos_y - 0.2, 0);
  float angle_output = -PID_Release(&angle_pid, 0, chassis.angle); // 控制偏航角
  Limit(angle_output, MAX_CHASSIS_ANGLE_SPEED);
}

int Chassis_AutoArrivedAtSpecifiedPoint_Flag = 0; // 由chassis_goto_point置1,手柄置0
int Chassis_GoToPointStart_Flag = 0;              // 累加标志位，可能会溢出，待修改
float start_point_x = 0, start_point_y = 0;
/**测试用：随距离减速到某一目标点*/
void chassis_goto_point(float point_x, float point_y)
{
  float distance = sqrtf((chassis.pos_x - point_x) * (chassis.pos_x - point_x) + (chassis.pos_y - point_y) * (chassis.pos_y - point_y));
  if (distance >= ARRIVE_CIRCLE)
  {
    chassis_status.go_to_point = 1;
    chassis.fangle = chassis_calculate_traceangle(point_x, point_y);
    // chassis.fspeed = chassis_calculate_linespeed(point_x, point_y, 0, 0, 340);
    if (Chassis_GoToPointStart_Flag == 1) // 保存起点
    {
      start_point_x = chassis.pos_x;
      start_point_y = chassis.pos_y;
      Chassis_GoToPointStart_Flag = 0;
    }
    chassis.fspeed = Chassis_CalculateLineSpeedPlus(start_point_x, start_point_y, point_x, point_y, 20, 0, 380);
    chassis_move(chassis.fspeed, chassis.fangle, 0);
    // uprintf("chassis pos:%f,%f,%f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
  }
  else // 到达终点后锁定偏航角
  {
    chassis_status.go_to_point = 0;
    Chassis_AutoArrivedAtSpecifiedPoint_Flag = 1;
    // uprintf("arrive:%f,%f,%f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
    chassis_move(0, 0, 0);
  }
  return;
}

void Chassis_GoToPoint_Plus(float point_x, float point_y, float target_angle)
{
  float distance = sqrtf((chassis.pos_x - point_x) * (chassis.pos_x - point_x) + (chassis.pos_y - point_y) * (chassis.pos_y - point_y));
  if (distance >= ARRIVE_CIRCLE)
  {
    chassis_status.go_to_point = 1;
    chassis.fangle = chassis_calculate_traceangle(point_x, point_y);
    // chassis.fspeed = chassis_calculate_linespeed(point_x, point_y, 0, 0, 340);
    if (Chassis_GoToPointStart_Flag == 1) // 保存起点
    {
      start_point_x = chassis.pos_x;
      start_point_y = chassis.pos_y;
      Chassis_GoToPointStart_Flag = 0;
    }
    chassis.fspeed = Chassis_CalculateLineSpeedPlus(start_point_x, start_point_y, point_x, point_y, 20, 0, 380);
    chassis_move(chassis.fspeed, chassis.fangle, target_angle);
    // uprintf("chassis pos:%f,%f,%f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
  }
  else // 到达终点后锁定偏航角
  {
    chassis_status.go_to_point = 0;
    Chassis_AutoArrivedAtSpecifiedPoint_Flag = 1;
    // uprintf("arrive:%f,%f,%f\r\n", chassis.pos_x, chassis.pos_y, chassis.angle);
    chassis_move(0, 0, 0);
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

/**
 * @brief  底盘顶层驱动(跑全场轨迹)
 * @param trace_num 
 **/
/*void chassis_move_traces(int trace_num)
{
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
}*/

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
}*/