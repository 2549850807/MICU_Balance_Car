#include "gyroscope_driver.h"

// ������ѧ����
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ȫ�ֻ����˲���ʵ��
ComplementaryFilter_t complementary_filter = {0};

/**
 * @brief ��ʼ����������̬�˲���
 * @param alpha �����˲���ϵ�� (�Ƽ�ֵ: 0.02-0.05)
 * @param sample_rate_hz ����Ƶ�� (Hz)
 * @retval None
 */
void Gyroscope_Filter_Init(float alpha, float sample_rate_hz)
{
    complementary_filter.alpha = alpha;
    complementary_filter.dt = 1.0f / sample_rate_hz;
    complementary_filter.angles.roll = 0.0f;
    complementary_filter.angles.pitch = 0.0f;
    complementary_filter.angles.yaw = 0.0f;
    complementary_filter.initialized = 0;
}

/**
 * @brief ���ƽǶ���-180��180�ȷ�Χ��
 * @param angle ����Ƕ�
 * @retval ���ƺ�ĽǶ�
 */
float Constrain_Angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief ����������ģ��
 * @param vec ��������
 * @retval ����ģ��
 */
float Vector_Magnitude(Vector3f *vec)
{
    return sqrtf(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

/**
 * @brief ������һ��
 * @param vec �����������
 * @retval None
 */
void Normalize_Vector(Vector3f *vec)
{
    float magnitude = Vector_Magnitude(vec);
    if (magnitude > 0.0f) {
        vec->x /= magnitude;
        vec->y /= magnitude;
        vec->z /= magnitude;
    }
}

/**
 * @brief �Ӽ��ٶȼ����ݼ����ʼ��̬��
 * @param accel ���ٶȼ����� (��λ: g)
 * @param euler �����ŷ����
 * @retval None
 */
void Calculate_Initial_Angles_From_Accel(Vector3f *accel, EulerAngles_t *euler)
{
    // ȷ�����ٶȼ�������Ч
    float accel_magnitude = Vector_Magnitude(accel);
    if (accel_magnitude < 0.5f || accel_magnitude > 2.0f) {
        // ���ٶȼ������쳣��ʹ��Ĭ��ֵ
        euler->roll = 0.0f;
        euler->pitch = 0.0f;
        euler->yaw = 0.0f;
        return;
    }
    
    // �Ӽ��ٶȼƼ������Ǻ͸�����
    euler->roll = atan2f(accel->y, sqrtf(accel->x * accel->x + accel->z * accel->z)) * 180.0f / M_PI;
    euler->pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z)) * 180.0f / M_PI;
    
    // ƫ�����޷��Ӽ��ٶȼƻ�ã���Ϊ0
    euler->yaw = 0.0f;
}

/**
 * @brief ʹ�����������ݻ��ּ���Ƕȱ仯
 * @param gyro ���������� (��λ: deg/s)
 * @param euler ��ǰŷ���� (�������)
 * @param dt ʱ���� (��)
 * @retval None
 */
void Integrate_Gyro_Data(Vector3f *gyro, EulerAngles_t *euler, float dt)
{
    // �����ǻ��ּ���Ƕȱ仯
    euler->roll += gyro->x * dt;
    euler->pitch += gyro->y * dt;
    euler->yaw += gyro->z * dt;
    
    // ���ƽǶȷ�Χ
    euler->roll = Constrain_Angle(euler->roll);
    euler->pitch = Constrain_Angle(euler->pitch);
    euler->yaw = Constrain_Angle(euler->yaw);
}

/**
 * @brief �����˲���������̬��
 * @param gyro ���������� (��λ: deg/s)
 * @param accel ���ٶȼ����� (��λ: g)
 * @retval None
 */
void Complementary_Filter_Update(Vector3f *gyro, Vector3f *accel)
{
    EulerAngles_t accel_angles = {0};
    
    // �״�����ʱ��ʼ��
    if (!complementary_filter.initialized) {
        Calculate_Initial_Angles_From_Accel(accel, &complementary_filter.angles);
        complementary_filter.initialized = 1;
        return;
    }
    
    // 1. �����ǻ��ֵõ��Ƕȱ仯
    Integrate_Gyro_Data(gyro, &complementary_filter.angles, complementary_filter.dt);
    
    // 2. �Ӽ��ٶȼƼ��㵱ǰ��̬��
    Calculate_Initial_Angles_From_Accel(accel, &accel_angles);
    
    // 3. �����˲����ں�
    // ֻ���ڼ��ٶȼ����ݿ���ʱ�Ž����ں� (����Ƿ�ӽ��������ٶ�)
    float accel_magnitude = Vector_Magnitude(accel);
    if (accel_magnitude > 0.7f && accel_magnitude < 1.3f) {
        // �����˲�: angle = �� * gyro_angle + (1-��) * accel_angle
        complementary_filter.angles.roll = complementary_filter.alpha * complementary_filter.angles.roll + 
                                          (1.0f - complementary_filter.alpha) * accel_angles.roll;
        complementary_filter.angles.pitch = complementary_filter.alpha * complementary_filter.angles.pitch + 
                                           (1.0f - complementary_filter.alpha) * accel_angles.pitch;
        // ƫ����ֻ��ͨ�������ǻ��ֵõ�
    }
    
    // �������սǶȷ�Χ
    complementary_filter.angles.roll = Constrain_Angle(complementary_filter.angles.roll);
    complementary_filter.angles.pitch = Constrain_Angle(complementary_filter.angles.pitch);
    complementary_filter.angles.yaw = Constrain_Angle(complementary_filter.angles.yaw);
}

/**
 * @brief ����ŷ���� (����ӿ�)
 * @param gyro ���������� (��λ: deg/s)
 * @param accel ���ٶȼ����� (��λ: g)
 * @param euler �����ŷ����
 * @retval None
 */
void Calculate_Euler_Angles(Vector3f *gyro, Vector3f *accel, EulerAngles_t *euler)
{
    // ���»����˲���
    Complementary_Filter_Update(gyro, accel);
    
    // ���ƽ��
    euler->roll = complementary_filter.angles.roll;
    euler->pitch = complementary_filter.angles.pitch;
    euler->yaw = complementary_filter.angles.yaw;
}

/**
 * @brief ��ȡ��ǰŷ����
 * @retval ��ǰŷ����ָ��
 */
EulerAngles_t* Get_Current_Euler_Angles(void)
{
    return &complementary_filter.angles;
}

/**
 * @brief ����ŷ����Ϊ��
 * @retval None
 */
void Reset_Euler_Angles(void)
{
    complementary_filter.angles.roll = 0.0f;
    complementary_filter.angles.pitch = 0.0f;
    complementary_filter.angles.yaw = 0.0f;
    complementary_filter.initialized = 0;
}

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
