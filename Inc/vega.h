#ifndef __vega_H
#define __vega_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "simplelib.h"

#define VEGA_USART huart5

typedef union{
  char ch[8];
  uint8_t ui8[8];
  uint16_t ui16[4];
  int16_t i16[4];
  int in[2];
  float fl[2];
  double df;
}Vega_Correct;


/*Variable Area*/


/*Function Area*/

void vega_calibration();
void vega_reset();
void vega_coordinate(float pos[3]);
void vega_set_position(float Dx ,float Dy ,float Dangle );
void vega_correct_pos(char *pos,float correctvalue);
void vega_print_pos();


#ifdef __cplusplus
}
#endif
#endif /*__ vega_H */