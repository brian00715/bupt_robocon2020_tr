/*******************************************************************************
Copyright:      Bupt
File name:      vec.c
Description:    定义向量运算
Author:         19th
Version：       1.0
Data:           2019/11/24
*******************************************************************************/
#include "vec.h"
#include "math.h"


/**向量加法*/
vec vec_add(vec a, vec b)
{
  vec tmp;
  tmp.x = a.x + b.x;
  tmp.y = a.y + b.y;
  return tmp;
}
/**向量点乘*/
double vec_mul(vec a, vec b)
{
  return a.x * b.x + a.y * b.y;
}
/**向量取模*/
double vec_model(vec a)
{
  return sqrt(a.x * a.x + a.y * a.y);
}
/**生成向量*/
vec vec_create(float x, float y)
{
  vec tmp;
  tmp.x = x;
  tmp.y = y;
  return tmp;
}
/**向量数乘*/
vec vec_mul_i(vec a, double b)
{
  a.x *= b;
  a.y *= b;
  return a;
}
/**向量法向*/
vec vec_normal(vec a)//顺时针90度法向 
{
  vec tmp;
  tmp.x = a.y;
  tmp.y = -a.x;
  return vec_mul_i(tmp,1/vec_model(tmp));
}
/**向量归一化*/
vec vec_unit(vec a)
{
  double b = vec_model(a);
  return vec_mul_i(a,1/b);
}
/**向量判空*/
int vec_is_zero(vec a)
{
  if(fabs(a.x) < 1e-6 && fabs(a.y) < 1e-6)
    return 1;
  return 0;
}