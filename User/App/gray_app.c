#include "gray_app.h"

unsigned char gray_digtal; // �Ҷȴ�����������

void Gray_Init(void)
{
  Uart_Printf(DEBUG_UART, "Gray_Init ......\r\n");
}

void Gray_Task(void)
{
    //��ȡ���������������
    gray_digtal = ~IIC_Get_Digtal();
//    Uart_Printf(&huart5, "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(gray_digtal>>0)&0x01,(gray_digtal>>1)&0x01,(gray_digtal>>2)&0x01,(gray_digtal>>3)&0x01,
//                                                              (gray_digtal>>4)&0x01,(gray_digtal>>5)&0x01,(gray_digtal>>6)&0x01,(gray_digtal>>7)&0x01);
}
