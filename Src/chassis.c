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


#define MAX_CHASSIS_MOTOR_SPEED 301           //! chassis_canset_motorspeed中对电机速度限幅
#define MAX_CHASSIS_ANGLE_SPEED 350                 //! chassis_move中对自转速度做限幅

Chassis chassis;
Chassis_Status chassis_status;

PID_Struct y_pid = {3000,175000,0,0,0,5000,0,0.005};//速度方向控制
PID_Struct angle_pid = {-1000,0,0,0,0,5000,0,0.005};//偏高角控制

float Arrive_distance = 0.005;


/*****************************初始化*************************/

/**底盘状态初始化*/
void chassis_init_status(){
  chassis_status.go_to_point = 0;           //go_to_point_for_test函数控制变量，1为开启，0为关闭
  chassis_status.count = 0;                //跑点计数，初试为0
  chassis_status.trace_count = 0;
  chassis_status.run_point = 0;
  chassis_status.run_point_test = 0;
  chassis_status.vega_is_ready = 0;       //初始化前不ready
  chassis_status.is_begin = 1;
  flag.chassis_control_flag = 0;//控制周期5ms
}
/**底盘位置初始化*/
void chassis_init_pos(float x,float y){
  chassis.fspeed = 0;    
  chassis.fangle = 0;    
  chassis.fturn = 0;
  chassis.vega_init_pos_x=0;
  chassis.vega_init_pos_y=0;
  chassis.vega_init_angle=0;
  chassis.vega_init_pos_x += x - chassis.pos_x;
  chassis.vega_init_pos_y += y - chassis.pos_y;
}
/**底盘初始化*/
void chassis_init(void){
  //chassis_init_pos(0.439f,0.320f); //全场定位坐标偏移
  //启用轨迹时的函数初始化
  chassis_swap_xy(points_pos0,150);//将xy坐标进行一次交换以适应场地
  chassis_init_pos(points_pos0[0].x , points_pos0[0].y);
  chassis_init_status();
  point_print_path(); //轨迹排序打印
}
/****************************计算**************************/
//交换x、y坐标的函数，由于轨迹规划上位机坐标轴与实际坐标轴不符合而使用
//参数：待交换轨迹集，轨迹集内点个数
//author：zohycao
void chassis_swap_xy(Point points_pos[],int point_num){
  int i;
  float temp_pos;
  for(i=0;i<point_num;i++){
    temp_pos=points_pos[i].x;
    points_pos[i].x = points_pos[i].y;
    points_pos[i].y = temp_pos;
  }
  return;
}

/**计算到目标点的角度*/
float chassis_calculate_traceangle(float point_x , float point_y){
  return Angle_Between_Points(chassis.pos_x, chassis.pos_y, point_x, point_y);
}
/**计算直线跑点速度*/
int chassis_calculate_linespeed(float point_x , float point_y , int start_speed , int final_speed , int max_speed){
  float distance_to_target = sqrtf( (point_x - chassis.pos_x)*(point_x - chassis.pos_x) + (point_y - chassis.pos_y)*(point_y - chassis.pos_y) );
  int int_speed = (int)((final_speed - start_speed)*distance_to_target + start_speed);
  Limit_From_To(int_speed,0,max_speed);
  return int_speed;
}
/****************************驱动**************************/

/**底盘电机驱动*/ 
void chassis_canset_motorduty(int s1,int s2,int s3){
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

void chassis_canset_motorspeed(int s1,int s2,int s3){
    can_msg can_TX_data[3];
    Limit(s1,MAX_CHASSIS_MOTOR_SPEED);
    Limit(s2,MAX_CHASSIS_MOTOR_SPEED);
    Limit(s3,MAX_CHASSIS_MOTOR_SPEED);

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
/**底盘底层驱动(跑速度):speed 速度;direction 速度方向;target_angle 偏航角*/
void chassis_move(int speed , float direction, float target_angle){

  //TODO  角度
float ERR_angle_m2 = PI/3, ERR_angle_m1 = -PI/3, ERR_angle_m0 = PI; //三轮与全场定位模块安装偏角
  //FIXME  角度解算
  float speed_out_0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - direction));
  float speed_out_1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - direction));
  float speed_out_2 = -(speed*cos((ERR_angle_m2 + chassis.angle) - direction));  
  float angle_output = 0;
  //TODO 此处将angle_output改为了两种，即自动时为偏航角，手动时为自旋角速度，后续需改成根据MODE再判断，
  if(flag.chassis_auto_flag == 1 && flag.chassis_handle_flag == 0){
    angle_output = PID_Release(&angle_pid, target_angle, chassis.angle);
    Limit(angle_output,MAX_CHASSIS_ANGLE_SPEED);  
  }
  else if(flag.chassis_auto_flag == 0 && flag.chassis_handle_flag == 1){
    angle_output = target_angle;
    Limit(angle_output,MAX_CHASSIS_ANGLE_SPEED);  
  }

  float motor0 = speed_out_0 + angle_output;
  float motor1 = speed_out_1 + angle_output;
  float motor2 = speed_out_2 + angle_output;
  chassis_canset_motorspeed((int)motor0,(int)motor1,(int)motor2);
}
/**底盘中层驱动(跑向量):now_speed_vec 当前速度;target_speed_vec 目标速度;distance_vec 位移向量; target_angle偏航角*/
void chassis_move_vector(vec now_speed_vec, vec target_speed_vec, vec distance_vec, float target_angle){
  if(vec_is_zero(target_speed_vec)){
    //TODO 目标点速度为0无法运动 需修改
    chassis_move(0,0,chassis.angle);  
  }
  else{
  float vector_d = 0;  
  vector_d = vec_mul(distance_vec,vec_normal(target_speed_vec));//速度投影

  float vx_output = vec_model(target_speed_vec);
  //TODO :pid控制量改为distance
  float vy_output = -PID_Release(&y_pid,0,vector_d);
  //TODO 向量法向为顺时针90度是否正确
  vec output = vec_add(target_speed_vec , vec_mul_i(vec_normal(target_speed_vec),vy_output));
  
  chassis.fspeed = (int)vec_model(output);
  Limit(chassis.fspeed,2100);
  chassis.fangle = atan2(output.y,output.x);
  chassis_move(chassis.fspeed ,chassis.fangle, target_angle);
  }
}
/**底盘上册层驱动(跑轨迹)：points_pos point 轨迹点集; point_num 轨迹点数 */
int chassis_move_trace(Point points_pos[],int point_num){
  /*只在每一段路径第一次调用这个函数的时候，赋予chassis_status.point_num初值*/
  if(chassis_status.is_begin == 1){
    chassis_status.point_num = point_num;
    chassis_status.is_begin = 0;
  }

  

  static vec now_speed_vec, target_speed_vec, distance_vec;
  now_speed_vec = vec_create(chassis.speed_x,chassis.speed_y);
  target_speed_vec = vec_create((float)points_pos[chassis_status.count].speed * cos(points_pos[chassis_status.count].direct),
                                (float)points_pos[chassis_status.count].speed * sin(points_pos[chassis_status.count].direct));
  distance_vec = vec_create(points_pos[chassis_status.count].x - chassis.pos_x, points_pos[chassis_status.count].y - chassis.pos_y);
  
  
  double distance_to_next = vec_model(distance_vec);
    
  float dynamic_arrive_distance = Arrive_distance; //可修改
  int arriveJudge = (( distance_to_next <= Arrive_distance || distance_to_next <= dynamic_arrive_distance )
                    &&( chassis_status.count >= chassis_status.point_num - 2));
  if(arriveJudge)//判断经过此点 
  {
    uprintf("到%d点---(%f,%f,%f)\r\n",chassis_status.count,chassis.pos_x,chassis.pos_y,chassis.angle);
    chassis_status.count += 1;
    
    if(chassis_status.count >= chassis_status.point_num - 1){ //到达目的地
      chassis_finish_onetrace();      
      uprintf("到trace%d终点(%f,%f)\r\n",chassis_status.trace_count,chassis.pos_x,chassis.pos_y);
      chassis_move(0,0,points_pos[point_num - 1].target_angle); 
      return 1;
    }
  }

  if(chassis_status.count < chassis_status.point_num - 3){
    chassis_move_vector(now_speed_vec, target_speed_vec, distance_vec, points_pos[chassis_status.count].target_angle);
  }
  /*由于速度慢，最后三个点采用其它跑点方法*/
  else if(chassis_status.count >= chassis_status.point_num - 3 && chassis_status.count <= chassis_status.point_num - 2)
  {
    chassis.fangle = chassis_calculate_traceangle(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_linespeed(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y,60,50,60);
    chassis_move(chassis.fspeed,chassis.fangle,points_pos[chassis_status.count].target_angle);
  }
  else if(chassis_status.count == chassis_status.point_num - 1)
  {
    chassis.fangle = chassis_calculate_traceangle(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y);
    chassis.fspeed = chassis_calculate_linespeed(points_pos[chassis_status.count].x ,points_pos[chassis_status.count].y,30,0,30);
    chassis_move(chassis.fspeed,chassis.fangle,points_pos[chassis_status.count].target_angle);
  }  
  return 0;
}
/**底盘顶层驱动(跑全场轨迹)*/
void chassis_move_traces(int trace_num){
  switch (trace_num)
  {
  case 0:
    //do nothing
    break;
  case 1:
    chassis_move_trace(points_pos0,150);
    uprintf("TraceOne Done");
    break;
  default:
    break;
  }

}
/****************************测试**************************/

/**测试用：随距离减速到某一目标点*/
void chassis_goto_point(float point_x , float point_y){
  //TODO goto待修改
  float distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  if( distance >= Arrive_distance ){     
    chassis.fangle = chassis_calculate_traceangle(point_x , point_y);
    chassis.fspeed = chassis_calculate_linespeed(point_x ,point_y,150,0,450);
    chassis_move( chassis.fspeed , chassis.fangle , 0);
  }  
  else{
    chassis_status.go_to_point = 0;
    uprintf("arrive:%f,%f,%f\r\n",chassis.pos_x, chassis.pos_y, chassis.angle); 
    chassis_move(0,0,chassis.angle);
  }    
  return;
}
/**测试用：按向量跑*/
void chassis_goto_vector(vec target_position){
  chassis.fturn = 0;  
  vec distance_vec = vec_create(target_position.x - chassis.pos_x,target_position.y - chassis.pos_y);
  vec target_speed_vec = vec_create(0,0);
  vec now_speed_vec = vec_create(chassis.speed_x,chassis.speed_y);
  chassis_move_vector(now_speed_vec, target_speed_vec, distance_vec, chassis.angle);
}
/****************************状态&执行**************************/

/**跑完每一段路径后,标志位的改变*/
void chassis_finish_onetrace(){
  //TODO: 内容待更改
  chassis_status.run_point = 0;
  chassis_status.count = 0;
  chassis.fspeed = 0;
  chassis_status.is_begin = 1;//开始下一段时使用
  flag.chassis_laser_flag = 0;
  chassis_status.trace_count = (chassis_status.trace_count + 1);
  //chassis_status.trace_count = chassis_status.trace_count + 1;//开始下一段
}
/**底盘更新坐标*/
void chassis_pos_update(){ 
  chassis.pos_x = chassis.vega_pos_x + chassis.vega_init_pos_x;    //m
  chassis.pos_y = chassis.vega_pos_y + chassis.vega_init_pos_y;    //m
  chassis.angle = (chassis.vega_angle / 180.f) * PI + chassis.vega_init_angle;    //弧度

  chassis.speed_x = (chassis.pos_x - chassis.last_pos_x) / 0.005;    //m/s
  chassis.speed_y = (chassis.pos_y - chassis.last_pos_y) / 0.005;    //m/s
  chassis.speed_angle = (chassis.angle - chassis.last_angle) / 0.005;    //弧度/s
  chassis.now_speed = vec_model(vec_create(chassis.speed_x,chassis.speed_y)); 

  chassis.last_pos_x = chassis.pos_x; 
  chassis.last_pos_y = chassis.pos_y; 
  chassis.last_angle = chassis.angle; 
}
/**执行函数*/
void chassis_exe(){
  chassis_pos_update();
  if(flag.chassis_auto_flag == 1 && flag.chassis_handle_flag == 0){
    chassis_move_traces(chassis_status.trace_count);
    return;
  }
  if(flag.chassis_handle_flag == 1 && flag.chassis_auto_flag == 0){
    handle_exe();
  }
}

