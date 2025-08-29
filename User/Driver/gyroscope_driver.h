#ifndef __GYROSCOPE_DRIVER_H__
#define __GYROSCOPE_DRIVER_H__

#include "main.h"
#include "icm20608.h"
#include "bno08x_hal.h"
#include "stdbool.h"

// 欧拉角结构体定义
typedef struct {
    float roll;     // 横滚角 (绕X轴旋转)
    float pitch;    // 俯仰角 (绕Y轴旋转)
    float yaw;      // 偏航角 (绕Z轴旋转)
} EulerAngles_t;

// 互补滤波器参数结构体
typedef struct {
    float alpha;              // 互补滤波器系数 (0-1之间，通常0.02-0.05)
    float dt;                 // 采样间隔时间(秒)
    EulerAngles_t angles;     // 当前欧拉角
    uint8_t initialized;      // 初始化标志
} ComplementaryFilter_t;

// 全局互补滤波器实例
extern ComplementaryFilter_t complementary_filter;

// 欧拉角解算相关函数声明
void Gyroscope_Filter_Init(float alpha, float sample_rate_hz);
void Calculate_Euler_Angles(Vector3f *gyro, Vector3f *accel, EulerAngles_t *euler);
void Complementary_Filter_Update(Vector3f *gyro, Vector3f *accel);
EulerAngles_t* Get_Current_Euler_Angles(void);
void Reset_Euler_Angles(void);

// 辅助函数
float Constrain_Angle(float angle);
float Vector_Magnitude(Vector3f *vec);
void Normalize_Vector(Vector3f *vec);

float convert_to_continuous_yaw(float current_yaw);

#endif

