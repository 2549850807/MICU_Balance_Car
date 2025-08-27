#include "motor_driver.h"

#define Motor_ABS(x) ((x) >= 0 ? (x) : -(x))

void Motor_Config_Init(MOTOR* motor, 
                       TIM_HandleTypeDef *htim_in1, uint32_t pwm_channel_in1,
                       TIM_HandleTypeDef *htim_in2, uint32_t pwm_channel_in2,
                       unsigned char reverse, int dead_band_speed)
{
    // ����IN1ͨ��
    motor->config.in1.htim = htim_in1;
    motor->config.in1.pwm_channel = pwm_channel_in1;
    
    // ����IN2ͨ��
    motor->config.in2.htim = htim_in2;
    motor->config.in2.pwm_channel = pwm_channel_in2;
    
    motor->config.reverse = reverse;
    motor->dead_band_speed = dead_band_speed;
    motor->speed = 0;
  
    // ��������PWMͨ������ʼ��Ϊֹͣ״̬
    HAL_TIM_PWM_Start(motor->config.in1.htim, motor->config.in1.pwm_channel);
    HAL_TIM_PWM_Start(motor->config.in2.htim, motor->config.in2.pwm_channel);
    
    __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, 0);
}

// ����ٶ��޷�
int Motor_Limit_Speed(MOTOR* motor, int speed, int max_speed, int min_speed)
{
    if(speed > max_speed)
        speed = max_speed;
    else if(speed < min_speed)
        speed = min_speed;

    return speed;
}

// �����������
int Motor_Dead_Compensation(MOTOR* motor)
{
    if(motor->speed > 0 && motor->speed < motor->dead_band_speed)
        return motor->dead_band_speed;
    else if(motor->speed < 0 && motor->speed > -motor->dead_band_speed) 
        return -motor->dead_band_speed;
    else
        return motor->speed;
}

// �ٶȿ���
void Motor_Set_Speed(MOTOR* motor, int speed)
{
    // ����ٶ��޷�
    int max_speed = motor->config.in1.htim->Init.Period; // ʹ��IN1�Ķ�ʱ��������Ϊ����ٶ�
    motor->speed = Motor_Limit_Speed(motor, speed, max_speed, -max_speed);
  
    // �����������
    motor->speed = Motor_Dead_Compensation(motor);
    
    // ����ʵ��PWMֵ������ֵ��
    uint32_t pwm_value = Motor_ABS(motor->speed);
    
    // ���ݷ���ͷ�ת��־����˫PWMͨ��
    if((motor->speed >= 0 && motor->config.reverse == 0) || 
       (motor->speed < 0 && motor->config.reverse == 1)) 
    {
        // ������ת��IN1=PWM, IN2=0
        __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, pwm_value);
        __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, 0);
    }
    else 
    {
        // ������ת��IN1=0, IN2=PWM
        __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, pwm_value);
    }
}

// ���ֹͣ
void Motor_Stop(MOTOR* motor)
{
    // ����ͨ��������Ϊ0���͵�ƽ��
    __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, 0);
    
    motor->speed = 0;
}

// ���ɲ��
void Motor_Brake(MOTOR* motor)
{
    // ��ȡ���PWMֵ����ʱ������ֵ��
    uint32_t max_pwm = motor->config.in1.htim->Init.Period;
    
    // ����ͨ��������Ϊ���ֵ���ߵ�ƽ��
    __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, max_pwm);
    __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, max_pwm);
    
    motor->speed = 0;
}
