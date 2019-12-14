#ifndef __sensor_gpio_H
#define __sensor_gpio_H
#ifdef __cplusplus
 extern "C" {
#endif
extern int microswitch_state;
extern int infrared_state;
extern int magnet_state;
extern int cylinder_state;

int gpio_microswitch();
int gpio_infrared();
void gpio_magnet(int pinstate);
void gpio_cylinder(int pinstate);

void gpio_sensor_exe();

#ifdef __cplusplus
}
#endif
#endif