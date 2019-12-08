#ifndef __vega_H
#define __vega_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "simplelib.h"

#define VEGA_USART huart5
/*Variable Area*/

extern float vega_position[3];

/*Function Area*/

void vega_calibration();
void vega_reset();
void vega_coordinate(float pos[3]);
void vega_set_position(float Dx ,float Dy ,float Dangle );
void vega_print_pos();


#ifdef __cplusplus
}
#endif
#endif /*__ vega_H */