#include "can_rec.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "laser.h"


/**
  * @brief  ʹ��canʱ�����ʼ�����Ĺ�������cubemx�����æ��ʼ�������ͬʱ����������������can��canͨ��
  *         ��Ҫ�����Լ����շ��������������������Ĭ�ϵĽ��ղ�����������ȫ������can�����ϵ�������Ϣ
  *         �����������can�ĳ�ʼ���������
  * @param  *hcan Ҫ��ʼ����Ŀ��can
  */
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    if (_hcan == &hcan1)
    {
        CAN_FilterConfigStructure.FilterBank = 0;
    }
    else if (_hcan == &hcan2)
    {
        CAN_FilterConfigStructure.FilterBank = 14;
    }

    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the CAN peripheral */
    if (HAL_CAN_Start(_hcan) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}


/**
 * @brief ʹ��can���²���ӷ���ָ��
 * @param command �޷���������8bit
 */
void CANSendData(CAN_HandleTypeDef *_hcan, uint32_t ID, uint8_t length, uint8_t *data)    //����̰��ӷ���ָ��
{
    static CAN_TxHeaderTypeDef TxHeader;    //���ͱ��Ľṹ�嶨��
    static uint8_t TxData[8];
    static uint32_t mbox;

    TxHeader.StdId = ID;           // id��ֵ
    TxHeader.IDE=0;                     // ��׼֡
    TxHeader.RTR=0;                     //
    TxHeader.DLC=length;                // 1�ֽ�����֡

    //���ݸ�ֵ
    for(int i = 0; i < length; i++)
    {
        TxData[i] = *(data + i);
    }

    //��һ��������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //�ж��Ƿ��ͳɹ�
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}


void SendShootSpeed(int speed)
{
    CANSendData(&hcan1, COMMAND_SHOOT_SPD_ID, 4, (uint8_t*)(&speed));
}

void SendShootSiganl(void)
{
    CANSendData(&hcan1, COMMAND_SHOOT_SIGAL_ID, 0, NULL);
}

void SendClawPostion(int position)
{
    CANSendData(&hcan1, COMMAND_CLAW_POS_ID, 4, (uint8_t*)(&position));
}


/*����һЩqueue��task*/
extern osMessageQId ResShootSpeedQueueHandle;
extern osMessageQId ResShootSignalQueueHandle;
extern osMessageQId ResClawPosQueueHandle;


/**
  * @brief  ����fifo0�Ļص������������൱��can��rx0�ж�
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //���ջص�����
{
    static HAL_StatusTypeDef HAL_RetVal;
    static CAN_RxHeaderTypeDef RxHeader;
    static uint8_t RxData[8] = {0};

    if(hcan == &hcan2)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);      //��CAN2�������ݣ�ͨ�������������FIFO0,����RxMessage����֡

        if(HAL_RetVal == HAL_OK)
        {
            switch (RxHeader.StdId) {
                case laser_01:
                    for (int i = 0; i < 8; ++i) {
                       laser_data_01.uint_8[i] = RxData[i];
                    }
                    break;
                case laser_23:
                    for (int i = 0; i < 8; ++i) {
                        laser_data_23.uint_8[i] = RxData[i];
                    }
                    break;
                default:
                    break;
            }

        }
        __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);   //��һ�£���Ȼ�Ϳ�ס��
    }
    else if(hcan == &hcan1)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
        if(HAL_RetVal == HAL_OK)
        {
            switch (RxHeader.StdId)
            {
                case RESPOND_ID:
                    switch (RxData[0])
                    {
                        case RESPOND_SHOOT_SPD:
                            xQueueOverwriteFromISR(ResShootSpeedQueueHandle,RxData,0);
                            break;
                        case RESPOND_SHOOT_SIGAL:
                            xQueueOverwriteFromISR(ResShootSignalQueueHandle,RxData,0);
                            break;
                        case RESPOND_CLAW_POS:
                            xQueueOverwriteFromISR(ResClawPosQueueHandle,RxData,0);
                            break;
                        default:
                            break;
                    }
                    break;
                case STATE_ID:
                    break;

            }
        }
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

}