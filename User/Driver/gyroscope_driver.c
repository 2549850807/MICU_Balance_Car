#include "gyroscope_driver.h"

// ʹ�þ�̬������������һ�ε�״̬
float g_last_yaw = 0.0f;
int g_revolution_count = 0;
bool g_is_yaw_initialized = false;

/**
 * @brief ��һ����[-180, 180]��Χ�ڵ�yaw�Ƕ�ת��Ϊ�����ĽǶ�ֵ��
 * 
 * @param current_yaw �Ӵ�������ȡ�ĵ�ǰyawֵ (-180 to 180)��
 * @return float ������yaw�Ƕ�ֵ (���� 370, -450 ��)��
 */
float convert_to_continuous_yaw(float current_yaw) 
{
    // ����һ����ֵ����⡰���䡱�����ֵӦ�ô���180��ͨ��ȡ270��300�Ƚϰ�ȫ��
    const float WRAP_AROUND_THRESHOLD = 300.0f;

    // �״ε���ʱ���г�ʼ��
    if (!g_is_yaw_initialized) {
        g_last_yaw = current_yaw;
        g_is_yaw_initialized = true;
        g_revolution_count = 0;
    }

    // �������ϴζ����Ĳ���
    float diff = current_yaw - g_last_yaw;

    // ����Ƿ����ˡ����䡱
    if (diff > WRAP_AROUND_THRESHOLD) {
        // �����Ƕ��������Ƕ� (����, �� 170�� �� -175��), ʵ��������ת, Ȧ��Ӧ������
        // ��ʱ diff �ӽ� -360 (���� -175 - 170 = -345)
        // ����߼�������Ǵ�-180���䵽+180�������˵��������ת����
        g_revolution_count--;
    } else if (diff < -WRAP_AROUND_THRESHOLD) {
        // �Ӹ��Ƕ��������Ƕ� (����, �� -170�� �� 175��), ʵ��������ת, Ȧ��Ӧ�ü�С
        // ��ʱ diff �ӽ� 360 (���� 175 - (-170) = 345)
        // ����߼�������Ǵ�+180���䵽-180�������˵��������ת����
        g_revolution_count++;
    }

    // �����ϴε�yawֵ�Ա��´ε���
    g_last_yaw = current_yaw;

    // ����������yawֵ
    float continuous_yaw = current_yaw + (float)g_revolution_count * 360.0f;

    return continuous_yaw;
}

#define PI 3.14159265358979f

// Ԥ������
void PreprocessForMadgwick(Vector3f *gyro, Vector3f *accel) 
{
    // 1. �����ǵ�λת������/�� �� ����/�룩
    gyro->x *= PI / 180.0f;
    gyro->y *= PI / 180.0f;
    gyro->z *= PI / 180.0f;
    
    // 2. ����ϵ������������� ICM20608 ��װ����
    // ��������� ICM20608G ������
    float temp;
    switch(IMU_ID) {
        case WHO_AM_I_ICM20608D:
        case WHO_AM_I_ICM20608G:
        case WHO_AM_I_ICM20602:
            // ���� X �� Y �ᣬ��ת Z ��
            temp = accel->x;
            accel->x = accel->y;
            accel->y = temp;
            accel->z = -accel->z;
            
            temp = gyro->x;
            gyro->x = gyro->y;
            gyro->y = temp;
            gyro->z = -gyro->z;
            break;
            
        default:
            // ��������������Ĭ��
            break;
    }
}
