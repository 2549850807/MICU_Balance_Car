#include "key_app.h"

void Key_Init()
{
    Uart_Printf(DEBUG_UART, "Key_Init ......\r\n");
    Ebtn_Init();
}

void Key_Task()
{
    ebtn_process(HAL_GetTick());
}

/* �������¼��Ļص����� */
void my_handle_key_event(struct ebtn_btn *btn, ebtn_evt_t evt) {
    uint16_t key_id = btn->key_id;                 // ��ȡ�����¼��İ��� ID
    uint16_t click_cnt = ebtn_click_get_count(btn); // ��ȡ�������� (���� ONCLICK �¼�ʱ������)
    // uint16_t kalive_cnt = ebtn_keepalive_get_count(btn); // ��ȡ�������� (���� KEEPALIVE �¼�ʱ������)

    // �����¼����ͽ��д���
    switch (evt) {
        case EBTN_EVT_ONPRESS: // �����¼� (�����ɹ��󴥷�һ��)
            switch(key_id)
            {
              case 1:
                pid_running = 1;
                break;
              case 2:
                pid_set_target(&pid_speed_left, 500);
                pid_set_target(&pid_speed_right, 500);
                break;
              case 3:
                pid_set_target(&pid_speed_left, 1000);
                pid_set_target(&pid_speed_right, 1000);
                break;
              case 4:
                pid_set_target(&pid_speed_left, -500);
                pid_set_target(&pid_speed_right, -500);
                break;
            }
            
            Uart_Printf(DEBUG_UART, "Key%d Down\r\n", (int)key_id);
            led_buf[key_id - 1] ^= 1;
            break;
        case EBTN_EVT_ONRELEASE: // �ͷ��¼� (�����ɹ��󴥷�һ��)
            break;

        case EBTN_EVT_ONCLICK: // ����/�����¼� (���ͷź󣬻�ﵽ�������������ʱ�󴥷�)

            break;
        case EBTN_EVT_KEEPALIVE: // ���ֻ/�����¼� (���³���ʱ�䳬����ֵ�󣬰����ڴ���)
            led_buf[key_id - 1] ^= 1;
            break;

        default: // δ֪�¼� (�����ϲ�Ӧ����)
            // printf(" - Unknown Event\n");
            break;
    }
}

