#include "gyroscope_app.h"

uint8_t first_gyroscope_flag = 0;

float frist_roll = 0;
float frist_pitch = 0;
float frist_yaw = 0;

#if BNO08x_ON == 0

#define ICM20608_TIME 0.0004f

uint32_t icm20608_first_tick;

ICM20608 icm20608;

void Gyroscope_Init(void)
{
  Uart_Printf(DEBUG_UART, "ICM20608 Gyroscope_Init ......\r\n");
  
  ICM206xx_Init();
  
  icm20608_first_tick = HAL_GetTick();
}

void Gyroscope_Task(void)
{
  ICM206xx_Read_Data(&icm20608.gyro,&icm20608.accel,&icm20608.temperature);
  PreprocessForMadgwick(&icm20608.gyro, &icm20608.accel);
  imu_filter(icm20608.accel.x, icm20608.accel.y, icm20608.accel.z, icm20608.gyro.x, icm20608.gyro.y, icm20608.gyro.z);
  eulerAngles(q_est, &icm20608.Roll, &icm20608.Pitch, &icm20608.Yaw);
  
  /* ����ǰλ�ù��� */
  if(first_gyroscope_flag == 0)
  {
    first_gyroscope_flag = 1;
    frist_roll = icm20608.Roll;
    frist_pitch = icm20608.Pitch;
    frist_yaw = icm20608.Yaw;
  }
  
  icm20608.Roll = icm20608.Roll - frist_roll;
  icm20608.Pitch = icm20608.Pitch - frist_pitch;
  icm20608.Yaw = icm20608.Yaw - frist_yaw;
  
  icm20608.Yaw = convert_to_continuous_yaw(icm20608.Yaw); 
  
  if(icm20608.Yaw >= 0)
    icm20608.Yaw -= (HAL_GetTick() - icm20608_first_tick) * ICM20608_TIME;
  else
    icm20608.Yaw += (HAL_GetTick() - icm20608_first_tick) * ICM20608_TIME;
  
}

#else

BNO08x bno08x;

void Gyroscope_Init(void)
{
  Uart_Printf(DEBUG_UART, "BNO08x Gyroscope_Init ......\r\n");
  
  // 1. Ӳ����ʼ��
  BNO080_Init(&hi2c1, BNO080_DEFAULT_ADDRESS);
  Uart_Printf(DEBUG_UART, "BNO080 I2C��ʼ�����\n");

  // 2. Ӳ����λ���Ƽ�ʹ�ã����ɿ���
  if (BNO080_HardwareReset() == 0) {
      Uart_Printf(DEBUG_UART, "BNO080Ӳ����λ�ɹ�\n");
  } else {
      Uart_Printf(DEBUG_UART, "BNO080Ӳ����λʧ�ܣ����������λ\n");
      // ���÷����������λ
      softReset();
      HAL_Delay(100);
      Uart_Printf(DEBUG_UART, "BNO080�����λ���\n");
  }

  // 3. �������������ã��û���ѡ����ע��/ȡ��ע�ͣ�
  enableRotationVector(100);        // ������ת������100ms���
  Uart_Printf(DEBUG_UART, "��������ת�������� (100ms)\n");

  // enableAccelerometer(50);          // ���ü��ٶȼƣ�50ms���
  // Uart_Printf(DEBUG_UART, "�����ü��ٶȼƱ��� (50ms)\n");

  // enableGyro(50);                   // ���������ǣ�50ms���
  // Uart_Printf(DEBUG_UART, "�����������Ǳ��� (50ms)\n");

  // enableMagnetometer(100);          // ���ô����ƣ�100ms���
  // Uart_Printf(DEBUG_UART, "�����ô����Ʊ��� (100ms)\n");

  // enableLinearAccelerometer(50);    // �������Լ��ٶȣ�50ms���
  // Uart_Printf(DEBUG_UART, "���������Լ��ٶȱ��� (50ms)\n");

  // enableStepCounter(1000);          // ���ò���������1000ms���
  // Uart_Printf(DEBUG_UART, "�����ò��������� (1000ms)\n");

  // 4. �߼��������ã���ѡ��
  // enableGameRotationVector(20);     // ��Ϸģʽ��ת������20ms���
  // Uart_Printf(DEBUG_UART, "��������Ϸ��ת���� (20ms)\n");

  // enableStabilityClassifier(500);   // �ȶ��Է�������500ms���
  // Uart_Printf(DEBUG_UART, "�������ȶ��Է����� (500ms)\n");

  HAL_Delay(200); // �ȴ�������������Ч
  Uart_Printf(DEBUG_UART, "BNO080��������ʼ����ɣ�\n");
  
}

void Gyroscope_Task(void)
{
  // ����Ƿ���������
    if (dataAvailable()) 
    {

      // 1. ��Ԫ������̬���ݣ��������ܣ�
      float quat_i = getQuatI();
      float quat_j = getQuatJ();
      float quat_k = getQuatK();
      float quat_real = getQuatReal();
    
      QuaternionToEulerAngles(quat_i, quat_j, quat_k, quat_real, &bno08x.Roll, &bno08x.Pitch, &bno08x.Yaw);
      
      /* ����ǰλ�ù��� */
      if(first_gyroscope_flag == 0)
      {
        first_gyroscope_flag = 1;
        frist_roll = bno08x.Roll;
        frist_pitch = bno08x.Pitch;
        frist_yaw = bno08x.Yaw;
      }
      
      bno08x.Roll = bno08x.Roll - frist_roll;
      bno08x.Pitch = bno08x.Pitch - frist_pitch;
      bno08x.Yaw = bno08x.Yaw - frist_yaw;
    
      bno08x.Yaw = convert_to_continuous_yaw(bno08x.Yaw); 
      
      // 2. ���ٶ����ݣ���ѡ��
      // float accel_x = getAccelX();
      // float accel_y = getAccelY();
      // float accel_z = getAccelZ();
      // Uart_Printf(&huart1, "Accel: %.3f, %.3f, %.3f g\n", accel_x, accel_y, accel_z);

      // 3. ���������ݣ���ѡ��
      // float gyro_x = getGyroX();
      // float gyro_y = getGyroY();
      // float gyro_z = getGyroZ();
      // Uart_Printf(&huart1, "Gyro: %.3f, %.3f, %.3f rad/s\n", gyro_x, gyro_y, gyro_z);

      // 4. ���������ݣ���ѡ��
      // float mag_x = getMagX();
      // float mag_y = getMagY();
      // float mag_z = getMagZ();
      // Uart_Printf(&huart1, "Mag: %.1f, %.1f, %.1f ��T\n", mag_x, mag_y, mag_z);

      // 5. ���ȼ�أ������ã�
      // uint8_t quat_accuracy = getQuatAccuracy();
      // uint8_t accel_accuracy = getAccelAccuracy();
      // uint8_t gyro_accuracy = getGyroAccuracy();
      // uint8_t mag_accuracy = getMagAccuracy();
      // Uart_Printf(&huart1, "Accuracy: Q=%d A=%d G=%d M=%d\n",
      //           quat_accuracy, accel_accuracy, gyro_accuracy, mag_accuracy);

      // 6. �߼��������ݣ���ѡ��
      // uint16_t steps = getStepCount();
      // uint8_t stability = getStabilityClassifier();
      // Uart_Printf(&huart1, "Steps: %d, Stability: %d\n", steps, stability);
  }
}

#endif

