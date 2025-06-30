#include "ch32v20x_can.h"

// ���ߧڧ�ڧѧݧڧ٧ѧ�ڧ� CAN1 (PA11-RX, PA12-TX)
void CAN1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    CAN_InitTypeDef CAN_InitStructure = {0};

    // ���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    // ���ѧ����ۧܧ� CAN RX (PA11) �� TX (PA12)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ����ߧ�ڧԧ��ѧ�ڧ� CAN
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
    CAN_InitStructure.CAN_Prescaler = 6; // 96MHz / (1+9+4)/6 = 1 ���ҧڧ�/��
    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_Init(CAN1, ENABLE);
}
