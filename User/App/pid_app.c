#include "pid_app.h"

int basic_speed = 200; // 基础速度

/* PID 控制器实例 */
PID_T pid_speed_left;  // 左轮速度环
PID_T pid_speed_right; // 右轮速度环

PID_T pid_balance;    // 平衡环

/* PID 参数定义 */


PidParams_t pid_params_left = {
    .kp = 3.7f * 0.6f,        
    .ki = 0.5000f,      
    .kd = 0.00f,      
    .out_min = -999.0f,
    .out_max = 999.0f,
};

PidParams_t pid_params_right = {
    .kp = 3.7f * 0.6f,        
    .ki = 0.5000f,      
    .kd = 0.00f,      
    .out_min = -999.0f,
    .out_max = 999.0f,
};

PidParams_t pid_params_balance = {
    .kp = 2.0f,        
    .ki = 0.0000f,      
    .kd = 0.00f,      
    .out_min = -999.0f,
    .out_max = 999.0f,
};

void PID_Init(void)
{
  pid_init(&pid_speed_left,
           pid_params_left.kp, pid_params_left.ki, pid_params_left.kd,
           0.0f, pid_params_left.out_max);
  
  pid_init(&pid_speed_right,
           pid_params_right.kp, pid_params_right.ki, pid_params_right.kd,
           0.0f, pid_params_right.out_max);
  
  pid_init(&pid_balance,
           pid_params_balance.kp, pid_params_balance.ki, pid_params_balance.kd,
           0.0f, pid_params_balance.out_max);
  
  pid_set_target(&pid_speed_left, basic_speed);
  pid_set_target(&pid_speed_right, basic_speed);
  pid_set_target(&pid_balance, 0);
}

unsigned char pid_running = 0; // PID 控制使能开关

void Balance_Control(void) // 平衡环控制
{
  int balance_output = 0;
  
  balance_output = pid_calculate_positional(&pid_balance, icm20608.Roll);
  
  pid_set_target(&pid_speed_left, basic_speed + balance_output);
  pid_set_target(&pid_speed_right, basic_speed + balance_output);
}

void PID_Task(void)
{
    if(pid_running == 0) return;
  
    int output_left = 0, output_right = 0;
  
//    Balance_Control();
  
    // 使用位置式 PID 计算利用速度环计算输出
    output_left = pid_calculate_positional(&pid_speed_left, left_encoder.rpm);
    output_right = pid_calculate_positional(&pid_speed_right, right_encoder.rpm);
  
    // 设置电机速度
    Motor_Set_Speed(&left_motor, output_left);
    Motor_Set_Speed(&right_motor, output_right);
}




