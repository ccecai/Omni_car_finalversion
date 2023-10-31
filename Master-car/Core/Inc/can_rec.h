#ifndef __CAN_REC_H__
#define __CAN_REC_H__
#include "can.h"


//canָ��궨��
#define COMMAND_SHOOT_SPD_ID        0x100
#define COMMAND_SHOOT_SIGAL_ID      0x101
#define COMMAND_CLAW_POS_ID         0x102

#define RESPOND_ID                  0x0FF
#define RESPOND_SHOOT_SPD           0xF0
#define RESPOND_SHOOT_SIGAL         0xF1
#define RESPOND_CLAW_POS            0xF2

#define STATE_ID                    0x0FE

/*�����Ǻ�ץȡװ�����ָ��ĺ궨��*/
#define CLAW_POS_HEIHEST        -217000       //���������
#define CLAW_POS_LOWEST         0             //�½�����Ͷ�
#define CLAW_POS_ROUTINE        -160000       //�ص�Ĭ��λ��


/*ȫ�ֱ���������*/

/*����������*/

void CANSendData(CAN_HandleTypeDef *_hcan, uint32_t ID, uint8_t length, uint8_t *data);

void SendShootSpeed(int speed);
void SendShootSiganl(void);
void SendClawPostion(int position);
void Set_Omni_Can(CAN_HandleTypeDef* hcan,int v1);


void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan);

#endif /* __CAN_REC_H__ */