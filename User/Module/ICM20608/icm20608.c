#include "icm20608.h"
#include <math.h>

IMU_ID_READ IMU_ID=WHO_AM_I_MPU6050;
static uint8_t imu_address=ICM20689_ADRESS;


/***************************************
������:	void ICM206xx_Init(void)
˵��: ���ٶȼ�/�����ǳ�ʼ��
���:	��
����:	uint8_t ����ʧ�ܱ�־
��ע:	��
����:	��������
***************************************/
uint8_t ICM206xx_Init(void)//ICM20689��ʼ��
{
	uint8_t fault=0;
	i2cWrite(imu_address,PWR_MGMT_1, 0x81);//���ǿ�Ƹ�λ81
	HAL_Delay(100);	
	IMU_ID=(IMU_ID_READ)(Single_ReadI2C(imu_address,WHO_AM_I));
	if(IMU_ID==0xFF)
	{
		imu_address=ICM20689_ADRESS+1;
		i2cWrite(imu_address,PWR_MGMT_1, 0x81);//���ǿ�Ƹ�λ81
		HAL_Delay(100);	
		IMU_ID=(IMU_ID_READ)(Single_ReadI2C(imu_address,WHO_AM_I));
	}
	switch(IMU_ID)
	{
		case WHO_AM_I_MPU6050:
		{
			i2cWrite(imu_address,PWR_MGMT_1  , 0x80);//�����λ
			HAL_Delay(200);
			i2cWrite(imu_address,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
			i2cWrite(imu_address,PWR_MGMT_1  , 0x03); //ʱ��Դ PLL with Z Gyro reference
			i2cWrite(imu_address,MPU_CONFIG  , 0x02); //�ڲ���ͨ�˲�Ƶ�ʣ����ٶȼ�184hz��������188hz  //Ĭ��0x03	  
			i2cWrite(imu_address,GYRO_CONFIG , 0x08); //500deg/s
			i2cWrite(imu_address,ACCEL_CONFIG, 0x08); // Accel scale 4g (8192 LSB/g)			
		}
		break;
		case WHO_AM_I_ICM20689:
		{
			i2cWrite(imu_address,PWR_MGMT_1  , 0x00);//�ر������ж�,�������
			i2cWrite(imu_address,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
			HAL_Delay(100);			
			i2cWrite(imu_address,MPU_CONFIG  , 0x02);//0x00���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������250hz����������306.6hz���¶�4000hz
																														//0x01���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������176hz����������177hz���¶�188hz
																														//0x02���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������92hz����������108.6hz���¶�98hz
																														//0x03���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������41hz����������59hz���¶�42hz		
																														//0x04���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������20hz����������30.5hz���¶�20hz
																														//0x05���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������10hz����������15.6hz���¶�10hz
			i2cWrite(imu_address,GYRO_CONFIG , 0x08);//�������������̣�500deg/s
			i2cWrite(imu_address,ACCEL_CONFIG, 0x08);// Accel scale 4g (8192 LSB/g)	
			i2cWrite(imu_address,ACCEL_CONFIG2,0x03);
																										 //0x00���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�218.1hz����������235hz		
																										 //0x01���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�218.1hz����������235hz
																										 //0x02���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�99.0hz����������121.3hz		
																										 //0x03���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�44.8hz����������61.5hz
																										 //0x04���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�21.2hz����������31.0hz
																										 //0x05���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�10.2hz����������15.5hz	
			//Single_WriteI2C(imu_address,ACCEL_CONFIG2,0xC3);//���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�1046.0hz����������1100.0hz
			HAL_Delay(500);		
		}
		break;	
		case WHO_AM_I_ICM20608D:
		case WHO_AM_I_ICM20608G:
		case WHO_AM_I_ICM20602:				
		{
			i2cWrite(imu_address,PWR_MGMT_1,0X80);	//��λICM20608
			HAL_Delay(100);
			i2cWrite(imu_address,PWR_MGMT_1, 0X01);	//����ICM20608
			
			i2cWrite(imu_address,0x19, 0x00);   /* ����������ڲ������� */
			i2cWrite(imu_address,0x1A, 0x02);   /* �����ǵ�ͨ�˲�BW=92Hz */
			i2cWrite(imu_address,0x1B, 0x08);   /* �����ǡ�500dps���� */
			i2cWrite(imu_address,0x1C, 0x08);   /* ���ٶȼơ�16G���� */
			i2cWrite(imu_address,0x1D, 0x03);   /* ���ٶȼƵ�ͨ�˲�BW=44.8Hz */
			
			i2cWrite(imu_address,0x6C, 0x00);   /* �򿪼��ٶȼƺ������������� */
			i2cWrite(imu_address,0x1E, 0x00);   /* �رյ͹��� */
			i2cWrite(imu_address,0x23, 0x00);   /* �ر�FIFO */ 	
		}
		break;
		default:
		{
			fault=1;
		}			
	}
  HAL_Delay(500);

	return fault;
}

/***************************************
������:	void ICM206xx_Read_Data(vector3f *gyro,vector3f *accel,float *temperature)
˵��: ��ȡ���������ٶȼ�/������/�¶�����
���:	vector3f *gyro-��ȡ��������������ָ��
			vector3f *accelgyro-��ȡ������ٶ�����ָ��
			float *temperaturegyro-��ȡ�¶�����ָ��
����:	��
��ע:	�����ǵ�λdeg/s,���ٶȼƵ�λg,�¶ȵ�λ��
����:	��������
***************************************/
void ICM206xx_Read_Data(Vector3f *gyro,Vector3f *accel,float *temperature)
{
	uint8_t buf[14];
	int16_t temp;
	i2cReadData(imu_address,ACCEL_XOUT_H,buf,14);
	switch(IMU_ID)
	{
		case WHO_AM_I_MPU6050:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=36.53f+(float)(temp/340.0f);	
		}
		break;
		case WHO_AM_I_ICM20689:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=25.0f+(double)((temp-25.0f)/326.8f);	
		}
		break;	
		case WHO_AM_I_ICM20608D:
		case WHO_AM_I_ICM20608G:
		case WHO_AM_I_ICM20602:				
		{
			accel->y=(int16_t)((buf[0]<<8)|buf[1]);
			accel->x=(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->y	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->x	=(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			
			*temperature=25.0f+(double)((temp-25.0f)/326.8f);		
		}
		break;
		default:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=36.53f+(float)(temp/340.0f);				
		}			
	}
  
	gyro->x*=GYRO_CALIBRATION_COFF;
	gyro->y*=GYRO_CALIBRATION_COFF;
	gyro->z*=GYRO_CALIBRATION_COFF;
	
  accel->x/=GRAVITY_RAW;
	accel->y/=GRAVITY_RAW;
	accel->z/=GRAVITY_RAW;
}


