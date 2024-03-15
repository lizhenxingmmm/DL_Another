#ifndef FIX_YAW_H
#define FIX_YAW_H

#include "INS_task.h"
#include "PID.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "exchange.h"

//����һЩ�����ļ���ȫ�ֱ���
extern ins_data_t ins_data;		//���������ǽ������������
extern RC_ctrl_t rc_ctrl;		//ң��������
typedef struct
{
      
      int16_t speed;
      int16_t sendspeed;
      int16_t target_speed;
      int16_t current; 
      float target_angle;
      int16_t bmz;
      float angle;
      float anglen;
      int16_t temp;
      int16_t torque_current;
      

} motor;

//����һЩ�궨��
void pitch_cal();
void Yaw_angle_task(void const *pvParameters);

//���庯��
void diy_control();
void Yaw_task(void const *pvParameters);
void Yaw_angle_task(void const *pvParameters);

#endif