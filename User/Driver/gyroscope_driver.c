#include "gyroscope_driver.h"

// 定义数学常数
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 全局互补滤波器实例
ComplementaryFilter_t complementary_filter = {0};

/**
 * @brief 初始化陀螺仪姿态滤波器
 * @param alpha 互补滤波器系数 (推荐值: 0.02-0.05)
 * @param sample_rate_hz 采样频率 (Hz)
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
 * @brief 限制角度在-180到180度范围内
 * @param angle 输入角度
 * @retval 限制后的角度
 */
float Constrain_Angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 计算向量的模长
 * @param vec 输入向量
 * @retval 向量模长
 */
float Vector_Magnitude(Vector3f *vec)
{
    return sqrtf(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
}

/**
 * @brief 向量归一化
 * @param vec 输入输出向量
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
 * @brief 从加速度计数据计算初始姿态角
 * @param accel 加速度计数据 (单位: g)
 * @param euler 输出的欧拉角
 * @retval None
 */
void Calculate_Initial_Angles_From_Accel(Vector3f *accel, EulerAngles_t *euler)
{
    // 确保加速度计数据有效
    float accel_magnitude = Vector_Magnitude(accel);
    if (accel_magnitude < 0.5f || accel_magnitude > 2.0f) {
        // 加速度计数据异常，使用默认值
        euler->roll = 0.0f;
        euler->pitch = 0.0f;
        euler->yaw = 0.0f;
        return;
    }
    
    // 从加速度计计算横滚角和俯仰角
    euler->roll = atan2f(accel->y, sqrtf(accel->x * accel->x + accel->z * accel->z)) * 180.0f / M_PI;
    euler->pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z)) * 180.0f / M_PI;
    
    // 偏航角无法从加速度计获得，设为0
    euler->yaw = 0.0f;
}

/**
 * @brief 使用陀螺仪数据积分计算角度变化
 * @param gyro 陀螺仪数据 (单位: deg/s)
 * @param euler 当前欧拉角 (输入输出)
 * @param dt 时间间隔 (秒)
 * @retval None
 */
void Integrate_Gyro_Data(Vector3f *gyro, EulerAngles_t *euler, float dt)
{
    // 陀螺仪积分计算角度变化
    euler->roll += gyro->x * dt;
    euler->pitch += gyro->y * dt;
    euler->yaw += gyro->z * dt;
    
    // 限制角度范围
    euler->roll = Constrain_Angle(euler->roll);
    euler->pitch = Constrain_Angle(euler->pitch);
    euler->yaw = Constrain_Angle(euler->yaw);
}

/**
 * @brief 互补滤波器更新姿态角
 * @param gyro 陀螺仪数据 (单位: deg/s)
 * @param accel 加速度计数据 (单位: g)
 * @retval None
 */
void Complementary_Filter_Update(Vector3f *gyro, Vector3f *accel)
{
    EulerAngles_t accel_angles = {0};
    
    // 首次运行时初始化
    if (!complementary_filter.initialized) {
        Calculate_Initial_Angles_From_Accel(accel, &complementary_filter.angles);
        complementary_filter.initialized = 1;
        return;
    }
    
    // 1. 陀螺仪积分得到角度变化
    Integrate_Gyro_Data(gyro, &complementary_filter.angles, complementary_filter.dt);
    
    // 2. 从加速度计计算当前姿态角
    Calculate_Initial_Angles_From_Accel(accel, &accel_angles);
    
    // 3. 互补滤波器融合
    // 只有在加速度计数据可信时才进行融合 (检查是否接近重力加速度)
    float accel_magnitude = Vector_Magnitude(accel);
    if (accel_magnitude > 0.7f && accel_magnitude < 1.3f) {
        // 互补滤波: angle = α * gyro_angle + (1-α) * accel_angle
        complementary_filter.angles.roll = complementary_filter.alpha * complementary_filter.angles.roll + 
                                          (1.0f - complementary_filter.alpha) * accel_angles.roll;
        complementary_filter.angles.pitch = complementary_filter.alpha * complementary_filter.angles.pitch + 
                                           (1.0f - complementary_filter.alpha) * accel_angles.pitch;
        // 偏航角只能通过陀螺仪积分得到
    }
    
    // 限制最终角度范围
    complementary_filter.angles.roll = Constrain_Angle(complementary_filter.angles.roll);
    complementary_filter.angles.pitch = Constrain_Angle(complementary_filter.angles.pitch);
    complementary_filter.angles.yaw = Constrain_Angle(complementary_filter.angles.yaw);
}

/**
 * @brief 计算欧拉角 (对外接口)
 * @param gyro 陀螺仪数据 (单位: deg/s)
 * @param accel 加速度计数据 (单位: g)
 * @param euler 输出的欧拉角
 * @retval None
 */
void Calculate_Euler_Angles(Vector3f *gyro, Vector3f *accel, EulerAngles_t *euler)
{
    // 更新互补滤波器
    Complementary_Filter_Update(gyro, accel);
    
    // 复制结果
    euler->roll = complementary_filter.angles.roll;
    euler->pitch = complementary_filter.angles.pitch;
    euler->yaw = complementary_filter.angles.yaw;
}

/**
 * @brief 获取当前欧拉角
 * @retval 当前欧拉角指针
 */
EulerAngles_t* Get_Current_Euler_Angles(void)
{
    return &complementary_filter.angles;
}

/**
 * @brief 重置欧拉角为零
 * @retval None
 */
void Reset_Euler_Angles(void)
{
    complementary_filter.angles.roll = 0.0f;
    complementary_filter.angles.pitch = 0.0f;
    complementary_filter.angles.yaw = 0.0f;
    complementary_filter.initialized = 0;
}

// 使用静态变量来保存上一次的状态
float g_last_yaw = 0.0f;
int g_revolution_count = 0;
bool g_is_yaw_initialized = false;

/**
 * @brief 将一个在[-180, 180]范围内的yaw角度转换为连续的角度值。
 * 
 * @param current_yaw 从传感器读取的当前yaw值 (-180 to 180)。
 * @return float 连续的yaw角度值 (例如 370, -450 等)。
 */
float convert_to_continuous_yaw(float current_yaw) 
{
    // 定义一个阈值来检测“跳变”。这个值应该大于180，通常取270或300比较安全。
    const float WRAP_AROUND_THRESHOLD = 300.0f;

    // 首次调用时进行初始化
    if (!g_is_yaw_initialized) {
        g_last_yaw = current_yaw;
        g_is_yaw_initialized = true;
        g_revolution_count = 0;
    }

    // 计算与上次读数的差异
    float diff = current_yaw - g_last_yaw;

    // 检测是否发生了“跳变”
    if (diff > WRAP_AROUND_THRESHOLD) {
        // 从正角度跳到负角度 (例如, 从 170° 到 -175°), 实际是向右转, 圈数应该增加
        // 此时 diff 接近 -360 (例如 -175 - 170 = -345)
        // 这段逻辑处理的是从-180跳变到+180的情况，说明是向左转过界
        g_revolution_count--;
    } else if (diff < -WRAP_AROUND_THRESHOLD) {
        // 从负角度跳到正角度 (例如, 从 -170° 到 175°), 实际是向左转, 圈数应该减小
        // 此时 diff 接近 360 (例如 175 - (-170) = 345)
        // 这段逻辑处理的是从+180跳变到-180的情况，说明是向右转过界
        g_revolution_count++;
    }

    // 更新上次的yaw值以备下次调用
    g_last_yaw = current_yaw;

    // 计算连续的yaw值
    float continuous_yaw = current_yaw + (float)g_revolution_count * 360.0f;

    return continuous_yaw;
}

#define PI 3.14159265358979f

// 预处理函数
void PreprocessForMadgwick(Vector3f *gyro, Vector3f *accel) 
{
    // 1. 陀螺仪单位转换（度/秒 → 弧度/秒）
    gyro->x *= PI / 180.0f;
    gyro->y *= PI / 180.0f;
    gyro->z *= PI / 180.0f;
    
    // 2. 坐标系调整（根据你的 ICM20608 安装方向）
    // 以下是针对 ICM20608G 的配置
    float temp;
    switch(IMU_ID) {
        case WHO_AM_I_ICM20608D:
        case WHO_AM_I_ICM20608G:
        case WHO_AM_I_ICM20602:
            // 交换 X 和 Y 轴，反转 Z 轴
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
            // 其他传感器保持默认
            break;
    }
}
