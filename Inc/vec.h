#ifndef __vec_H
#define __vec_H
#ifdef __cplusplus
 extern "C" {
#endif
/*Struct Area*/
typedef struct
{
    float x;
    float y;
}vec;
/*Function Area*/
vec vec_create(float x, float y);   // 向量生成
vec vec_add(vec a, vec b);          // 向量加法
double vec_mul(vec a, vec b);       // 向量点乘
double vec_model(vec a);            // 向量取模
vec vec_mul_i(vec a, double b);     // 向量数乘
vec vec_normal(vec a);              // 向量顺时针90度法向
vec vec_unit(vec a);                // 向量归一化
int vec_is_zero(vec a);             // 向量模是否为0
float vec_angle_sub(vec b,vec a);
float vec_angle(vec a);

#ifdef __cplusplus
}
#endif
#endif
