#ifndef __point_H
#define __point_H
#ifdef __cplusplus
extern "C" {
#endif

#define POINTS_NUM 200 
typedef struct point
{
  float x;
  float y;
  int speed;
  float direct;
  float target_angle;
}Point;

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
extern Point points_pos10[POINTS_NUM];
extern Point points_pos11[POINTS_NUM];
extern Point points_pos12[POINTS_NUM];
// extern Point points_pos13[POINTS_NUM];
// extern Point points_pos14[POINTS_NUM];
// extern Point points_pos15[POINTS_NUM];
// extern Point points_pos16[POINTS_NUM];
// extern Point points_pos17[POINTS_NUM];
// extern Point points_pos18[POINTS_NUM];
// extern Point points_pos19[POINTS_NUM];
// extern Point points_pos20[POINTS_NUM];



#ifdef __cplusplus
}
#endif
#endif /*__ points_H */