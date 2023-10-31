//
// Created by Yanzhenbo on 2023/4/1.
//

#include "dgm.h"
#include "can_rec.h"

void DGM_SET_TARGET_VELOCITY(CAN_HandleTypeDef *_hcan, uint16_t dgm_id, float target_speed)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t mbox;         //����ʹ�õ���can����
    union_32 txdata;

    TxHeader.StdId = dgm_id<<6 | CAN_CMD_SET_VELOCITY;	         // id��ֵ
    TxHeader.IDE = 0;                     // ��׼֡
    TxHeader.RTR = 0;                     //
    TxHeader.DLC = 4;                     // 8�ֽ�����֡

    txdata.data_f = target_speed;

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, &txdata.data_8[0], &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void DGM_CAN_CMD_MOTOR_ENABLE(CAN_HandleTypeDef *_hcan, uint16_t dgm_id)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t mbox;         //����ʹ�õ���can����
    union_32 txdata;

    TxHeader.StdId = dgm_id<<6 | CAN_CMD_MOTOR_ENABLE;	         // id��ֵ
    TxHeader.IDE = 0;                     //��׼֡
    TxHeader.RTR = 0;                     //
    TxHeader.DLC = 0;

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, &txdata.data_8[0], &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
