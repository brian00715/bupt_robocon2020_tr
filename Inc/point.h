#ifndef __point_H
#define __point_H
#ifdef __cplusplus
extern "C" {
#endif

#include "point.h"
#include <math.h>
#include "chassis.h"

#define POINTS_NUM 200 
typedef struct point
{
  float x;
  float y;
  int speed;
  float direct;
  float target_angle;
}Point;

extern Point points_pos[POINTS_NUM];
extern Point points_pos0[POINTS_NUM];
extern Point points_pos1[POINTS_NUM];
extern Point points_pos2[POINTS_NUM];    
extern Point points_pos3[POINTS_NUM];
extern Point points_pos4[POINTS_NUM];
extern Point points_pos5[POINTS_NUM];
extern Point points_pos6[POINTS_NUM];
extern Point points_pos7[POINTS_NUM];
extern Point points_pos8[POINTS_NUM];
extern Point points_pos9[POINTS_NUM];

    
#ifdef __cplusplus
}
#endif
#endif /*__ points_H */