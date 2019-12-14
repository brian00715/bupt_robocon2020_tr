#include"main.h"
int key_state=0;
int infrared_state=0;
int magnet_state=0;
int cylinder_state=0;

int gpio_microkey(){
    int state = (int) HAL_GPIO_ReadPin(M_KEY1_GPIO_Port,M_KEY1_Pin);    
    //uprintf("Key state is %d\r\n",state);
    return state;
}
int gpio_infrared(){
    int state = (int)  HAL_GPIO_ReadPin(GDDG_GPIO_Port,GDDG_Pin); 
    //uprintf("Infrared state is %d\r\n",state);
    return state;
}
void gpio_magnet(int pinstate){
    if(pinstate)
    HAL_GPIO_WritePin(DCT_GPIO_Port,DCT_Pin,GPIO_PIN_SET);
    else
    HAL_GPIO_WritePin(DCT_GPIO_Port,DCT_Pin,GPIO_PIN_RESET); 

    //uprintf("Manget state is %d\r\n",pinstate);
}
void gpio_cylinder(int pinstate){
    if(pinstate)
    HAL_GPIO_WritePin(QG_GPIO_Port,QG_Pin,GPIO_PIN_SET);
    else
    HAL_GPIO_WritePin(QG_GPIO_Port,QG_Pin,GPIO_PIN_RESET); 

    //uprintf("Cylinder state is %d\r\n",pinstate);
}

void gpio_sensor_exe(){

    key_state=gpio_microkey();//微动开关
    infrared_state=gpio_infrared();//红外对管

    gpio_magnet(magnet_state);//电磁铁
    gpio_cylinder(cylinder_state);//气缸

}

