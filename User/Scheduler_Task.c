/*
 * @������������ģ������״� STM32F407VET6 ��ذ� �� HAL ����б�д
 * @���ߣ�
 * @���ܣ�
 * @ע�⣺
*/

#include "Scheduler_Task.h"

void System_Init(void)
{
  Uart_Printf(DEBUG_UART, "==== System Init ====\r\n");
  Led_Init();
  Key_Init();
  Uart_Init();
  Oled_Init();
  Gray_Init();
  Motor_Init();
  Encoder_Init();
  Gyroscope_Init();
  ZDT_Motor_Init();
  PID_Init();
  Aht20_Init();
  Flash_Init();
  Uart_Printf(DEBUG_UART, "==== Finish ====\r\n");
  HAL_TIM_Base_Start_IT(&htim2);
}

unsigned char key_timer10ms = 0;
unsigned char gyroscope_timer10ms = 0;
unsigned char encoder_timer10ms = 0;
unsigned char pid_timer10ms = 0;

// TIM2 �жϷ�������1ms �жϣ�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance != htim2.Instance) return;
  
  /* ����ɨ������ */
  if(++key_timer10ms >= 10)
  {
    key_timer10ms = 0;
    Key_Task();
  }
  
  /* ��̬���������� */
  if(++gyroscope_timer10ms >= 10)
  {
    gyroscope_timer10ms = 0;
    Encoder_Task();
    Gyroscope_Task();
    PID_Task();
  }
}


