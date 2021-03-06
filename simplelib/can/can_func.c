
#include "can_func.h"
#ifdef SL_CAN
#include "cmd.h"
#include "chassis_handle.h"
#include "mtr_driver.h"
#include "robomaster.h"

void can_get_mag_mtr(can_msg *data);

void can_func_init()
{
    can_callback_add(230, can_suc_rx);  // 测试用can id
    can_callback_add(324, Handle_Rocker);
    can_callback_add(325, Handle_Button_New);
    can_callback_add(0x201, can_robomaster_rcv_1); // 大疆电机反馈消息的回调函数

#ifdef SL_DEBU
    can_callback_add(1, can_suc_rx);
    can_callback_add(325, can_show_button);
    can_callback_add(324, can_show_rocker);
#endif
}

__weak void can_rx_callback(can_msg *data) {}

void can_suc_rx(can_msg *data)
{
    uprintf("can rx ok\r\n");
}

int can_data_show_flag = 1;
/**
 * @brief 串口打印手柄摇杆的信息
 **/
void can_show_rocker(can_msg *data)
{
    if (can_data_show_flag)
    {
        uprintf("%4d %4d %4d %4d\r", (int16_t)data->ui16[0],
                (int16_t)data->ui16[1],
                (int16_t)data->ui16[2],
                (int16_t)data->ui16[3]);
    }
}

/**
 * @brief 串口打印手柄按钮的信息
 **/
void can_show_button(can_msg *data)
{
    uprintf("%d %d %c\r\n", data->ui8[0], data->ui8[1], data->ui8[2]);
}

#ifdef SL_DEBUG
void can_suc_rx(can_msg *data)
{
    uprintf("can rx ok\r\n");
}

void can_show_button(can_msg *data)
{
    uprintf("%d %d %c\r\n", data->ui8[0], data->ui8[1], data->ui8[2]);
}

int can_data_show_flag = 0;
void can_show_rocker(can_msg *data)
{
    if (can_data_show_flag)
    {
        uprintf("%4d %4d %4d %4d\r", (int16_t)data->ui16[0],
                (int16_t)data->ui16[1],
                (int16_t)data->ui16[2],
                (int16_t)data->ui16[3]);
    }
}
#endif // SL_DEBUG

#endif // SL_CAN