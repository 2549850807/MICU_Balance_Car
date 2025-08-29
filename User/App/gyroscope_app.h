#ifndef __GYROSCOPE_APP_H__
#define __GYROSCOPE_APP_H__

#include "MyDefine.h"

#define BNO08x_ON 0 // �Ƿ����� BNO08x ������̬��������Ĭ��ʹ�� ICM20608-G ���ᴫ����

#if BNO08x_ON == 0

  typedef struct ICM20608 
  {
    Vector3f gyro;        // ������ԭʼ���� (deg/s)
    Vector3f accel;       // ���ٶȼ�ԭʼ���� (g)
    EulerAngles_t euler;  // ������ŷ���� (deg)
    float Roll;           // �����Ա��� - �����
    float Pitch;          // �����Ա��� - ������
    float Yaw;            // �����Ա��� - ƫ����
    float temperature;    // �¶� (��)
  } ICM20608;

  extern ICM20608 icm20608;

#else

  typedef struct BNO08x {
    float Roll;
    float Pitch;
    float Yaw;
  } BNO08x;

  extern BNO08x bno08x;

#endif

void Gyroscope_Init(void);
void Gyroscope_Task(void);
  
extern uint8_t first_gyroscope_flag;

#endif
