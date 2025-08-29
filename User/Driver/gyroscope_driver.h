#ifndef __GYROSCOPE_DRIVER_H__
#define __GYROSCOPE_DRIVER_H__

#include "main.h"
#include "icm20608.h"
#include "bno08x_hal.h"
#include "stdbool.h"

// ŷ���ǽṹ�嶨��
typedef struct {
    float roll;     // ����� (��X����ת)
    float pitch;    // ������ (��Y����ת)
    float yaw;      // ƫ���� (��Z����ת)
} EulerAngles_t;

// �����˲��������ṹ��
typedef struct {
    float alpha;              // �����˲���ϵ�� (0-1֮�䣬ͨ��0.02-0.05)
    float dt;                 // �������ʱ��(��)
    EulerAngles_t angles;     // ��ǰŷ����
    uint8_t initialized;      // ��ʼ����־
} ComplementaryFilter_t;

// ȫ�ֻ����˲���ʵ��
extern ComplementaryFilter_t complementary_filter;

// ŷ���ǽ�����غ�������
void Gyroscope_Filter_Init(float alpha, float sample_rate_hz);
void Calculate_Euler_Angles(Vector3f *gyro, Vector3f *accel, EulerAngles_t *euler);
void Complementary_Filter_Update(Vector3f *gyro, Vector3f *accel);
EulerAngles_t* Get_Current_Euler_Angles(void);
void Reset_Euler_Angles(void);

// ��������
float Constrain_Angle(float angle);
float Vector_Magnitude(Vector3f *vec);
void Normalize_Vector(Vector3f *vec);

float convert_to_continuous_yaw(float current_yaw);

#endif

