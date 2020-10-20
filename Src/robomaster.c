#include "robomaster.h"

RoboMaster robomaster[4];                          // �󽮵���Ĳ����ṹ�壬����ת�ǡ��ٶȵȲ�������can_robomaster_rcv_1�и�����ֵ
int Robomaster_RPMValue[4] = {50, 0, 0, 0};        // 4���󽮵�����ٶ��趨ֵ����
int Robomaster_PositionValue[4] = {1000, 0, 0, 0}; // 4���󽮵����λ�ý��趨ֵ����
int Robomaster_PositionOffset = 0;
// �󽮵�����ٶȻ���PID����
PID_Struct Robomaster_Speed_PID[4] = {1.4, 0.9, 0.6, 0, 0, 5000, 0, 0.005}; // = {1.4,0.9,0.6,0,0,5000,0,0.005};
// �󽮵����λ�û���PID����
PID_Struct Robomaster_Position_PID[4] = {0.13, 0, 0.082, 0, 0, 5000, 0, 0.005}; // = {0.13,0,0.082,0,0,5000,0,0.005};
int robomaster_flag = 0;
int ver_slide_error = 0;
int Robomaster_PrintInfo_Flag = 0;
int Robomaster_RPMControl_Flag = 0;
int Robomaster_PosControl_Flag = 0;

/*����1�ŵ�����͵ķ���CAN��Ϣ*/
void can_robomaster_rcv_1(can_msg *pRxMsg)
{
  //pRxMsg->StdId
  static int first_flag = 1;
  if (first_flag == 1)
  {
    robomaster[0].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
    robomaster[0].offset_angle = robomaster[0].angle;
    first_flag = 0;
    robomaster[0].round_cnt = 0;
    return;
  }
  robomaster[0].last_angle = robomaster[0].angle;
  robomaster[0].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
  robomaster[0].speed_rpm = (int16_t)(pRxMsg->ui8[2] << 8 | pRxMsg->ui8[3]);
  robomaster[0].real_current = (pRxMsg->ui8[4] << 8 | pRxMsg->ui8[5]) * 5.f / 16384.f;

  if (robomaster[0].angle - robomaster[0].last_angle > 4096)
    robomaster[0].round_cnt--;
  else if (robomaster[0].angle - robomaster[0].last_angle < -4096)
    robomaster[0].round_cnt++;
  robomaster[0].total_angle = robomaster[0].round_cnt * 8192 + robomaster[0].angle - robomaster[0].offset_angle;
  // if (Robomaster_PrintInfo_Flag == 1)
  // {
  //   uprintf("--robomaster[0] info:\r\n");
  //   uprintf("  angle:%d speed_rpm:%d current:%d\r\n",
  //           robomaster[0].angle, robomaster[0].speed_rpm, robomaster[0].real_current);
  // }
}

/*����2�ŵ�����͵ķ���CAN��Ϣ*/
void can_robomaster_rcv_2(can_msg *pRxMsg)
{
  //pRxMsg->StdId
  //����id��û��
  static int first_flag = 1;
  if (first_flag == 1)
  {
    robomaster[1].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
    robomaster[1].offset_angle = robomaster[1].angle;
    first_flag = 0;
    robomaster[1].round_cnt = 0;
    return;
  }
  robomaster[1].last_angle = robomaster[1].angle;
  robomaster[1].angle = (uint16_t)(pRxMsg->ui8[0] << 8 | pRxMsg->ui8[1]);
  robomaster[1].speed_rpm = (int16_t)(pRxMsg->ui8[2] << 8 | pRxMsg->ui8[3]);
  robomaster[1].real_current = (pRxMsg->ui8[4] << 8 | pRxMsg->ui8[5]) * 5.f / 16384.f;

  if (robomaster[1].angle - robomaster[1].last_angle > 4096)
    robomaster[1].round_cnt--;
  else if (robomaster[1].angle - robomaster[1].last_angle < -4096)
    robomaster[1].round_cnt++;
  robomaster[1].total_angle = robomaster[1].round_cnt * 8192 + robomaster[1].angle - robomaster[1].offset_angle;
}

/*���ٶȻ�*/
void Robomaster_RPMControl()
{
  if (Robomaster_RPMControl_Flag == 0)
  {
    return;
  }
  float speed_out[4];
  for (int i = 0; i < 4; i++)
  {
    speed_out[i] = robomaster_speed_pid_control(i); // �����趨�ٶȵó�Ӧ�÷��͵ĵ���ֵ
  }
  robomaster_set_current((int16_t)speed_out[0], (int16_t)speed_out[1], (int16_t)speed_out[2], (int16_t)speed_out[3]);
}

/*��λ�û�*/
void Robomaster_PositionControl()
{
  if (Robomaster_PosControl_Flag == 0)
  {
    return;
  }
  float pos_out[4];
  for (int i = 0; i < 4; i++)
  {
    pos_out[i] = robomaster_position_pid_control(i); // �����趨�ٶȵó�Ӧ�÷��͵ĵ���ֵ
    if (pos_out[i] > 1000)                           // ���Ƶ�����С
    {
    }
  }
}

/**
 * @brief �󽮵�����ٶȻ���pid���㺯��
 * @param id Ҫ���õĵ��can id
 * @return ��ǰӦ���͸�������ٶ�ֵ
 **/
float robomaster_speed_pid_control(int id)
{
  float speed_out = 0;
  speed_out = PID_Release(&Robomaster_Speed_PID[id], Robomaster_RPMValue[id], (float)robomaster[id].speed_rpm);
  //speed_out=PID_Release(&Robomaster_Speed_PID[id],(float)robomaster[id].target_speed,(float)robomaster[id].speed_rpm);
  return speed_out;
}

/**
 * @brief �󽮵����λ�û���pid���㺯��
 * @param id Ҫ���õĵ��can id
 * @return ��ǰӦ���͸�������ٶ�ֵ
 **/
float robomaster_position_pid_control(int id)
{
  float speed_out = 0;
  float position_out = 0;
  position_out = PID_Release(&Robomaster_Position_PID[id],
                             (float)robomaster[id].target_position + ver_slide_error, (float)robomaster[id].total_angle);
  speed_out = PID_Release(&Robomaster_Speed_PID[id], position_out, (float)robomaster[id].speed_rpm);
  return speed_out;
}

/**
 * @brief ���ô󽮵������
 * @param iq1 ����ֵ/mA
 **/
void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  uint8_t Data[8];
  Data[0] = (iq1 >> 8);
  Data[1] = iq1;
  Data[2] = (iq2 >> 8);
  Data[3] = iq2;
  Data[4] = iq3 >> 8;
  Data[5] = iq3;
  Data[6] = iq4 >> 8;
  Data[7] = iq4;
  can_msg dji_data;
  for (int i = 0; i <= 7; i++)
  {
    dji_data.ui8[i] = Data[i];
  }
  can_send_msg(0x200, &dji_data); // ǰ4�������can_idΪ0x200
}

/**
 * @brief ��ӡ�󽮵����Ϣ
 * @param index ������
 *        @arg 0 1 2 3
 **/
void Robomaster_PrintInfo(int index)
{
  if (Robomaster_PrintInfo_Flag == 1)
  {
    // uprintf("--robomaster[%d] info:\r\n", index);
    uprintf("  now_angle:%d total_angle:%d speed_rpm:%d current:%d\r\n",
            (uint32_t)robomaster[index].angle * 360 / 8192, (uint32_t)robomaster[index].total_angle * 360 / 8192,
            robomaster[index].speed_rpm, robomaster[index].real_current);
  }
}

uint32_t Robomaster_OpenAngleControl_Flag = 0; // CMD�����Ƿ�����ת��ֹͣ����1����
uint32_t Robomaster_TargetOffsetAngle = 0;     // CMDָ��Ŀ��ת�Ǳ仯������λ�ȣ����ǻ��ȣ�
static uint32_t now_angle = 0;
static uint32_t robomaster_origin_angle = 0;
static int first_flag = 0;
/**
 * @brief ���ƴ󽮵��ת��һ���ǶȺ�ֹͣ,��Ҫ��m2006_exe()�����ʹ��
 *        Ŀ��ת�Ǳ仯����ͨ��CMDָ����ȫ�ֱ���
 * @param index �󽮵�����
 *    @arg 0 1 2 3
 * @return none
 **/
void Robomaster_StopByAngle(int index)
{
  if (Robomaster_OpenAngleControl_Flag == 0)
  {
    return;
  }

  if (first_flag == 0) // ��¼��ʼ�Ƕ�ֵ
  {
    robomaster_origin_angle = (uint32_t)robomaster[index].total_angle * 360 / 8192;
    first_flag = 1;
  }
  else
  {
    now_angle = (uint32_t)robomaster[index].total_angle * 360 / 8192;
    int offset_angle = now_angle - robomaster_origin_angle; // ����������ٶ��Ƕ�ֵ�������������
    if (offset_angle >= Robomaster_TargetOffsetAngle)
    {
      robomaster_set_current(0, 0, 0, 0); // ����ִ��
      MoterDriver_M2006_Current = 0;      // ��m2006_exe()�е�ִ�е�����0
      /*TODO:���ڿ��԰��������иĳɷ������������ֹͣ�ź�*/
      robomaster_origin_angle = 0;
      offset_angle = 0;
      first_flag = 0;
      Robomaster_OpenAngleControl_Flag = 0; // �������ʹ�ã���ͨ��CMD�ٴ���1
    }
  }
}

void M2006_init(int id)
{
  reset_PID(&Robomaster_Speed_PID[id]);
  reset_PID(&Robomaster_Position_PID[id]);
  Robomaster_Speed_PID[id].KP = 1.4;
  Robomaster_Speed_PID[id].KI = 0.9;
  Robomaster_Speed_PID[id].KD = 0.6;
  Robomaster_Speed_PID[id].i_max = 5000;
  Robomaster_Speed_PID[id].I_TIME = 0.005;

  Robomaster_Position_PID[id].KP = 0.13;
  Robomaster_Position_PID[id].KI = 0;
  Robomaster_Position_PID[id].KD = 0.082;
  Robomaster_Position_PID[id].i_max = 5000;
  Robomaster_Position_PID[id].I_TIME = 0.005;
}

void M3508_init(int id)
{
  reset_PID(&Robomaster_Speed_PID[id]);
  reset_PID(&Robomaster_Position_PID[id]);
  Robomaster_Speed_PID[id].KP = 1.6;
  Robomaster_Speed_PID[id].KI = 0.52;
  Robomaster_Speed_PID[id].KD = 0.6;
  Robomaster_Speed_PID[id].i_max = 5000;
  Robomaster_Speed_PID[id].I_TIME = 0.005;

  Robomaster_Position_PID[id].KP = 0.1;
  Robomaster_Position_PID[id].KI = 0;
  Robomaster_Position_PID[id].KD = 0.8;
  Robomaster_Position_PID[id].i_max = 5000;
  Robomaster_Position_PID[id].I_TIME = 0.005;
}
