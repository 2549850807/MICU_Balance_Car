#include "icm20608_iic.h"


extern I2C_HandleTypeDef hi2c1;  // ���ⲿ����� HAL I2C ���
extern I2C_HandleTypeDef hi2c2;  // ����ʵ��ʹ�õ� I2C �޸�

/***********************************************************
@��������Init_I2C
@��ڲ�������
@���ڲ�������
����������STM32 I2C ��ʼ�������� main ��ͨ�� HAL ��ʼ����
@˵����STM32 �� I2C ��ʼ��ͨ���� HAL ��ʼ���׶���ɣ��˴�����
*************************************************************/
void Init_I2C(void) {
    // STM32 �� I2C ��ʼ��ͨ���� HAL_I2C_MspInit �����
    // ���� GPIO ���ú� I2C ����ʱ��ʹ��
    // �˺�������Ϊ�ջ�����������ʼ��
}

/***********************************************************
@��������i2cWriteData
@���ܣ�I2C д���ֽ�����
@������addr-7λ�豸��ַ, regAddr-�Ĵ�����ַ, data-����ָ��, length-���ݳ���
*************************************************************/
void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
    HAL_I2C_Mem_Write(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/***********************************************************
@��������i2cRead
@���ܣ�I2C ��ȡ���ֽ�
@������addr-7λ�豸��ַ, regAddr-�Ĵ�����ַ
@����ֵ����ȡ��������
*************************************************************/
uint8_t i2cRead(uint8_t addr, uint8_t regAddr) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

/***********************************************************
@��������i2cWrite
@���ܣ�I2C д���ֽ�����
@������addr-7λ�豸��ַ, regAddr-�Ĵ�����ַ, data-��д������
*************************************************************/
void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/***********************************************************
@��������i2cReadData
@���ܣ�I2C ��ȡ���ֽ�����
@������addr-7λ�豸��ַ, regAddr-�Ĵ�����ַ, data-���ݻ�����, length-��ȡ����
*************************************************************/
void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
    HAL_I2C_Mem_Read(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/***********************************************************
@��������Single_WriteI2C
@���ܣ���װ���ֽ�д��
*************************************************************/
void Single_WriteI2C(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data) {
    i2cWrite(SlaveAddress, REG_Address, REG_data);
}

/***********************************************************
@��������Single_ReadI2C
@���ܣ���װ���ֽڶ�ȡ
@����ֵ����ȡ��������
*************************************************************/
uint8_t Single_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address) {
    return i2cRead(SlaveAddress, REG_Address);
}

/***********************************************************
@��������Double_ReadI2C
@���ܣ���ȡ�����ֽڲ����Ϊ16λ����
@����ֵ����Ϻ��16λ����
*************************************************************/
int16_t Double_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address) {
    uint8_t data[2];
    i2cReadData(SlaveAddress, REG_Address, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}
