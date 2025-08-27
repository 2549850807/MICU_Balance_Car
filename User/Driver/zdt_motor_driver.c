#include "zdt_motor_driver.h"

/**
  * @brief    �Բ��������̨�ϵ�������������ٶ��˶����ƣ��ٶ� = 10�����ٶ� = 0
  * @param    x_vel     ���ٶ�   ����Χ -5000RPM - 5000RPM
  * @param    y_vel     ���ٶ�   ����Χ -5000RPM - 5000RPM
  */
void Motor_Vel_Synchronous_Control(int x_vel, int y_vel)
{
  Emm_V5_Vel_Control(MOTOR_X_UART, MOTOR_X_ADDR, (x_vel >= 0 ? 0 : 1), ABS(x_vel), 0, MOTOR_SYNC_FLAG);
  Emm_V5_Vel_Control(MOTOR_Y_UART, MOTOR_Y_ADDR, (y_vel >= 0 ? 0 : 1), ABS(y_vel), 0, MOTOR_SYNC_FLAG);
}
