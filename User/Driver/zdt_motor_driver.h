#ifndef __ZDT_MOTOR_DRIVER_H__
#define __ZDT_MOTOR_DRIVER_H__

#include "MyDefine.h"

/* ������ƺ궨�� */
#define MOTOR_X_ADDR        0x02          // X������ַ
#define MOTOR_Y_ADDR        0x01          // Y������ַ
#define MOTOR_X_UART        &huart2        // X��������
#define MOTOR_Y_UART        &huart4        // Y��������
#define MOTOR_SYNC_FLAG     false         // ���ͬ����־
  
void Motor_Vel_Synchronous_Control(int x_vel, int y_vel);

#endif

