#ifndef __24L01_H__
#define __24L01_H__

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "main.h"

//��������ĺ궨��
//#define back_to_origin 0x13
//#define ring_point_left 0x11
//#define ring_point_right 0x12

//#define point_1 0x0d
//#define point_2 0x0f
//#define point_3 0x0e
//#define point_4 0x10

//#define flip 0x07
//#define reset 0x08

#define NRF_TAKE_RING 0x0b
#define NRF_ROBOT_LOCK 0x0a

#define NRF_CLAW_HEIGHT_3 0x14
#define NRF_CLAW_HEIGHT_2 0x15
#define NRF_CLAW_HEIGHT_1 0x16
#define NRF_CLAW_HEIGHT_0 0x17

#define NRF_SHOOT_RING 0x01

#define NRF_CLAW_CLOSE 0x05
#define NRF_CLAW_OPEN 0x06



#define NRF_PIONT_1 0xa1
#define NRF_PIONT_2 0xa2
#define NRF_PIONT_3 0xa3
#define NRF_PIONT_4 0xa4
#define NRF_PIONT_5 0xa5
#define NRF_PIONT_6 0xa6
#define NRF_PIONT_7 0xa7
#define NRF_PIONT_8 0xa8
#define NRF_PIONT_9 0xa9
#define NRF_PIONT_10 0xaa
#define NRF_PIONT_11 0xab
#define NRF_PIONT_12 0xac
#define NRF_PIONT_13 0xad
#define NRF_PIONT_14 0xae


#define NRF_TAKE_RING_0 0xc0
#define NRF_TAKE_RING_1 0xc1
#define NRF_TAKE_RING_2 0xc2


#define NRF_Laser_Loc   0x08
#define NRF_HAND_CTRL 0x0c

//����״̬�ĺ궨��
#define NRF_STATUS_SPEED  0x01
#define NRF_STATUS_ANGLE  0x02
#define NRF_STATUS_AUTO 0x04
#define NRF_STATUS_VISION 0x08




//ң�������սṹ��
typedef struct
{
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t command;
    uint8_t status;
    uint8_t speed;
    uint8_t height;
    int8_t yaw;
    uint16_t midValue;
}RemoteRXSturct;

extern uint16_t midValue;

// NRF24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH 5    // 5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH 5    // 5�ֽڵĵ�ַ���


#define TX_PLOAD_WIDTH 32 // 32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH 32//10    // 32�ֽڵ��û����ݿ��

// NRF24L01�Ĵ�����������
#define NRF_READ_REG 0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG 0x20 //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD 0x61   //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD 0xA0   //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX 0xE1      //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX 0xE2      //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL 0xE3   //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP 0xFF           //�ղ���,����������״̬�Ĵ���
// SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG 0x00     //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
// bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA 0x01      //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR 0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW 0x03   //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR 0x04 //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH 0x05      // RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP 0x06   // RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS 0x07     //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
// bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX 0x10     //�ﵽ����ʹ����ж�
#define TX_OK 0x20      // TX��������ж�
#define RX_OK 0x40      //���յ������ж�

#define OBSERVE_TX 0x08      //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD 0x09              //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0 0x0A      //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1 0x0B      //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2 0x0C      //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3 0x0D      //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4 0x0E      //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5 0x0F      //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR 0x10         //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0 0x11        //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1 0x12        //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2 0x13        //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3 0x14        //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4 0x15        //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5 0x16        //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS 0x17 // FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
// bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;

extern uint8_t NRFHeartBeat;


void NRF24L01_SPI_Init(void);
void NRF24L01_RX_Mode(void);                                              //����Ϊ����ģʽ
void NRF24L01_TX_Mode(void);                                              //����Ϊ����ģʽ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts); //д������
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);  //��������
uint8_t NRF24L01_Read_Reg(uint8_t reg);                                   //���Ĵ���
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);                   //д�Ĵ���
uint8_t NRF24L01_Check(void);                                             //���24L01�Ƿ����
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);                                //����һ����������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);                                //����һ����������

uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte);


#endif

