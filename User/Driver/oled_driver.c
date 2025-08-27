#include "oled_driver.h"

/**
 * @brief	ʹ������printf�ķ�ʽ��ʾ�ַ�������ʾ6x8��С��ASCII�ַ�
 * @param x  String start position on the X-axis  range��0 - 127
 * @param y  String start position on the Y-axis  range��0 - 3
 * @param {const char *} *format ����ʾ�ĸ�ʽ���ַ���
 * ���磺Oled_Printf(0, 0, 12, 0, "Data = %d", dat);
**/
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
	char buffer[128]; // ��ʱ�洢��ʽ������ַ���
	va_list arg;      // ����ɱ����
	int len;          // �����ַ�������

	va_start(arg, format);
	// ��ȫ�ظ�ʽ���ַ����� buffer
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);

	OLED_ShowStr(x, y, buffer, 8);

	return len;
}
