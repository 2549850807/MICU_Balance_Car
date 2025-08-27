#include "key_driver.h"

/* ����Ĭ������ */
static const ebtn_btn_param_t defaul_ebtn_param = EBTN_PARAMS_INIT(
    20,     // time_debounce: �����ȶ� 20ms
    20,     // time_debounce_release: �ͷ��ȶ� 20ms
    50,     // time_click_pressed_min: ��̵������� 50ms������Ϊ0�����������Сֵ
    500,    // time_click_pressed_max: ��������� 500ms (�������㵥��)������Ϊ0xFFFF������������ֵ���������ֳ����Ͱ����¼���
    200,    // time_click_multi_max: ��ε�������� 300ms (���ε��������������¼���)
    200,    // time_keepalive_period: �����¼����� 500ms (���³��� 500ms ��ÿ 500ms ����һ��)
    5       // max_consecutive: ���֧�� 5 ����
);

typedef enum
{
    USER_BUTTON_1 = 1,
    USER_BUTTON_2 = 2,
    USER_BUTTON_3 = 3,
    USER_BUTTON_4 = 4,
    USER_BUTTON_MAX,

    USER_BUTTON_COMBO_MAX,
} user_button_t;

/* 2. ���徲̬�����б� */
// ��: EBTN_BUTTON_INIT(����ID, ����ָ��)
static ebtn_btn_t static_buttons[] = {
        EBTN_BUTTON_INIT(USER_BUTTON_1, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_BUTTON_2, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_BUTTON_3, &defaul_ebtn_param),
        EBTN_BUTTON_INIT(USER_BUTTON_4, &defaul_ebtn_param),
};

// /* 3. ���徲̬��ϰ����б� (��ѡ) */
// // ��: EBTN_BUTTON_COMBO_INIT(����ID, ����ָ��)
// ebtn_btn_combo_t static_combos[] = {
//     // ���� KEY1+KEY2 ��ϼ�
//     EBTN_BUTTON_COMBO_INIT(USER_BUTTON_COMBO_0, &defaul_ebtn_param), // ��ϼ�, ID=USER_BUTTON_COMBO_0 (��������ͨ����ID��ͬ)
  
//     EBTN_BUTTON_COMBO_INIT(USER_BUTTON_COMBO_1, &defaul_ebtn_param), 
  
//     EBTN_BUTTON_COMBO_INIT(USER_BUTTON_COMBO_2, &defaul_ebtn_param), 
// };

/* 1. ʵ�ֻ�ȡ����״̬�Ļص����� */
// ����ԭ��: uint8_t (*ebtn_get_state_fn)(struct ebtn_btn *btn);
uint8_t my_get_key_state(struct ebtn_btn *btn) {
    // ���ݴ���İ�ťʵ���е� key_id �ж����ĸ�������
    switch (btn->key_id) {
        case 1: // �����ȡ KEY1 ��״̬
            // ���谴��Ϊ�ߵ�ƽ (���� 1 ������)
            return (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
        case 2: 
          return (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);
        case 3: 
          return (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET);
        case 4: 
          return (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET);
              
        // ... ������Ӹ��ఴ���Ķ�ȡ�߼� ...
        default:
            // ���ڿ��ڲ�������ϼ������������δ֪�� key_id����ȫ������� 0 (δ����)
            return 0;
    }
    // ע�⣺����ֵ 1 ��ʾ "�/����"��0 ��ʾ "�ǻ/�ͷ�"
}

int Ebtn_Init(void)
{
  // ��ʼ�� ebtn ��
    int init_ok = ebtn_init(
        static_buttons,                 // ��̬���������ָ��
        EBTN_ARRAY_SIZE(static_buttons), // ��̬�������� (�ú����)
        NULL,// static_combos,                  // ��̬��ϰ��������ָ�� (���û�У��� NULL, 0)
        0,// EBTN_ARRAY_SIZE(static_combos), // ��̬��ϰ������� (���û�У��� 0)
        my_get_key_state,               // ���״̬��ȡ�ص�����
        my_handle_key_event             // ����¼�����ص�����
    );

    if (!init_ok) {
        // ��ʼ��ʧ�ܣ������ǲ�������
        return -1; // ���߽�������������
    }
    
    // ������ϼ����ȴ���ģʽ����ֹ��ϼ��͵�����ͻ
    ebtn_set_config(EBTN_CFG_COMBO_PRIORITY);

    // // --- ������ϼ� (���ʹ������ϼ�) ---
    // // 1. �ҵ�������ϵ���ͨ�������ڲ����� (Index)
    // //    ע�⣺����ڲ�������һ�����������õ� key_id��
    // int key1_index = ebtn_get_btn_index_by_key_id(1); // ��ȡ KEY1 (ID=1) ���ڲ�����
    // int key2_index = ebtn_get_btn_index_by_key_id(2); // ��ȡ KEY2 (ID=2) ���ڲ�����

    // // 2. ����Щ������Ӧ�İ�����ӵ���ϼ�������
    // //    ȷ��������Ч (>= 0)
    // if (key1_index >= 0 && key2_index >= 0) {
    //     // ���� static_combos[0] �����Ƕ���� ID=101 ����ϼ�
    //     ebtn_combo_btn_add_btn_by_idx(&static_combos[0], key1_index); // �� KEY1 ��ӵ���ϼ�
    //     ebtn_combo_btn_add_btn_by_idx(&static_combos[0], key2_index); // �� KEY2 ��ӵ���ϼ�
    // } 
    // else {
    //     // ������Ҫ��� key_id �Ƿ���ȷ������ ebtn_init �Ƿ�ɹ�
    // }
    
    return 0;
}
