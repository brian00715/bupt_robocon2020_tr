#include "main.h"
#include "sensor_gpio.h"
#include "cmd.h"
int microswitch_state = 0;
int infrared_state = 1; //没有东西挡住时 为1
int magnet_state = 0;  // 值为1上电，0断电
int cylinder_state = 0;


/**
 * @brief 获取微动开关的电平
 **/
int gpio_microswitch()
{
    int state = (int)HAL_GPIO_ReadPin(M_KEY1_GPIO_Port, M_KEY1_Pin);
    //uprintf("Key switch is %d\r\n",state);
    return state;
}

/**
 * @brief 获取达阵上红外线的电平
 **/
// int gpio_infrared()
// {
//     int state = (int)HAL_GPIO_ReadPin(GDDG_GPIO_Port, GDDG_Pin);
//     //uprintf("Infrared state is %d\r\n",state);
//     return state;
// }

/**
 * @brief 翻转电磁铁的电平
 **/
void gpio_magnet(int pinstate)
{
    if (pinstate)
        HAL_GPIO_WritePin(DCT_GPIO_Port, DCT_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DCT_GPIO_Port, DCT_Pin, GPIO_PIN_RESET);

    //uprintf("Manget state is %d\r\n",pinstate);
}

/**气缸*/
// void gpio_cylinder(int pinstate)
// {
//     if (pinstate)
//         HAL_GPIO_WritePin(QG_GPIO_Port, QG_Pin, GPIO_PIN_SET);
//     else
//         HAL_GPIO_WritePin(QG_GPIO_Port, QG_Pin, GPIO_PIN_RESET);

//     //uprintf("Cylinder state is %d\r\n",pinstate);
// }

void gpio_sensor_exe()
{
    int current_microswitch = gpio_microswitch();
    // int current_infrared = gpio_infrared();
    if (current_microswitch != microswitch_state)
    {
        if (current_microswitch == 1)
            uprintf("Microswitch closed(0 to 1)\r\n");
        else
            uprintf("Microswitch disclosed(1 to 0)\r\n");
    }
    // if (current_infrared != infrared_state)
    // {
    //     if (current_infrared == 0)
    //         uprintf("Infrared blocked(1 to 0)\r\n");
    //     else
    //         uprintf("Infrared unblocked(0 to 1)\r\n");
    // }
    microswitch_state = current_microswitch; //微动开关
    // infrared_state = current_infrared;       //红外对管

    gpio_magnet(magnet_state);     //电磁铁
    // gpio_cylinder(cylinder_state); //气缸
}
