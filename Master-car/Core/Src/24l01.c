#include "24L01.h"
#include "main.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"



/* ��չ���� ------------------------------------------------------------------*/

#define CE_LOW 	 HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_RESET)
#define CE_HIGH  HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_SET)
#define CS_LOW 	 HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)
#define CS_HIGH  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)

#define IRQ_READ HAL_GPIO_ReadPin(SPI1_IQRN_GPIO_Port, SPI1_IQRN_Pin)

#define ABS(x)  ((x>0)? (x): -(x))

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0x01}; //���͵�ַ
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0x01}; //���͵�ַ

uint16_t midValue = 0;

uint8_t NRFHeartBeat = 0;

/**
 * ��������: ������Flash��ȡд��һ���ֽ����ݲ�����һ���ֽ�����
 * �������: byte������������
 * �� �� ֵ: uint8_t�����յ�������
 * ˵    ������
 */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte)
{
    uint8_t d_read, d_send = byte;
    if (HAL_SPI_TransmitReceive(&hspi1, &d_send, &d_read, 1, 0xFFF) != HAL_OK)
    {
        d_read = 0xFF;
    }
    return d_read;
}

/**
 * ��������: ���24L01�Ƿ����
 * �������: ��
 * �� �� ֵ: 0���ɹ�;1��ʧ��
 * ˵    ������
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;

    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); //д��5���ֽڵĵ�ַ.
    NRF24L01_Read_Buf(TX_ADDR, buf, 5);                  //����д��ĵ�ַ
    for (i = 0; i < 5; i++)
        if (buf[i] != 0XA5)
            break;
    if (i != 5)
        return 1; //���24L01����
    return 0;     //��⵽24L01
}

/**
 * ��������: SPIд�Ĵ���
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ����reg:ָ���Ĵ�����ַ
 *
 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    CS_LOW;                                   //ʹ��SPI����
    status = SPIx_ReadWriteByte(&hspi1, reg); //���ͼĴ�����
    SPIx_ReadWriteByte(&hspi1, value);        //д��Ĵ�����ֵ
    CS_HIGH;                                  //��ֹSPI����
    return (status);                          //����״ֵ̬
}

/**
 * ��������: ��ȡSPI�Ĵ���ֵ
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ����reg:Ҫ���ļĴ���
 *
 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    CS_LOW;                                     //ʹ��SPI����
    SPIx_ReadWriteByte(&hspi1, reg);            //���ͼĴ�����
    reg_val = SPIx_ReadWriteByte(&hspi1, 0XFF); //��ȡ�Ĵ�������
    CS_HIGH;                                    //��ֹSPI����
    return (reg_val);                           //����״ֵ̬
}

/**
 * ��������: ��ָ��λ�ö���ָ�����ȵ�����
 * �������: ��
 * �� �� ֵ: �˴ζ�����״̬�Ĵ���ֵ
 * ˵    ������
 *
 */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;

    CS_LOW;                                   //ʹ��SPI����
    status = SPIx_ReadWriteByte(&hspi1, reg); //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        pBuf[uint8_t_ctr] = SPIx_ReadWriteByte(&hspi1, 0XFF); //��������
    }
    CS_HIGH;       //�ر�SPI����
    return status; //���ض�����״ֵ̬
}

/**
 * ��������: ��ָ��λ��дָ�����ȵ�����
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ����reg:�Ĵ���(λ��)  *pBuf:����ָ��  len:���ݳ���
 *
 */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;
    CS_LOW;                                   //ʹ��SPI����
    status = SPIx_ReadWriteByte(&hspi1, reg); //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        SPIx_ReadWriteByte(&hspi1, *pBuf++); //д������
    }
    CS_HIGH;       //�ر�SPI����
    return status; //���ض�����״ֵ̬
}

/**
 * ��������: ����NRF24L01����һ������
 * �������: ��
 * �� �� ֵ: �������״��
 * ˵    ����txbuf:�����������׵�ַ
 *
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t sta;
    CE_LOW;
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //д���ݵ�TX BUF  9���ֽ�
    CE_HIGH;                                                //��������

    while (IRQ_READ != 0)
        ; //�ȴ��������

    sta = NRF24L01_Read_Reg(STATUS);                 //��ȡ״̬�Ĵ�����ֵ
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //���TX_DS��MAX_RT�жϱ�־
    if (sta & MAX_TX)                                //�ﵽ����ط�����
    {
        NRF24L01_Write_Reg(FLUSH_TX, 0xff); //���TX FIFO�Ĵ���
        return MAX_TX;
    }
    if (sta & TX_OK) //�������
    {
        return TX_OK;
    }
    return 0xff; //����ԭ����ʧ��
}

/**
 * ��������:����NRF24L01����һ������
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ������
 *
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;
    sta = NRF24L01_Read_Reg(STATUS);                 //��ȡ״̬�Ĵ�����ֵ
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //���TX_DS��MAX_RT�жϱ�־
    if (sta & RX_OK)                                 //���յ�����
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); //��ȡ����         //���RX FIFO�Ĵ���
        NRF24L01_Write_Reg(FLUSH_RX,0xff);
        return 0;
    }
    return 1; //û�յ��κ�����
}

/**
 * ��������: �ú�����ʼ��NRF24L01��RXģʽ
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ������
 *
 */
void NRF24L01_RX_Mode(void)
{
    uint8_t sta;
    CE_LOW;
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //дRX�ڵ��ַ
    //NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //дRX�ڵ��ַ
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);     //ʹ��ͨ��0���Զ�Ӧ��

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01); //ʹ��ͨ��0�Ľ��յ�ַ

    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 0);       //����RFͨ��Ƶ��

    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //ѡ��ͨ��0����Ч���ݿ��

    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);  //����TX�������,0db����,2Mbps,���������濪��

    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC

    CE_HIGH; // CEΪ��,�������ģʽ
    //HAL_Delay(1);

    sta = NRF24L01_Read_Reg(STATUS);                 //��ȡ״̬�Ĵ�����ֵ
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //���TX_DS��MAX_RT�жϱ�־
}

/*����һЩqueue��task�����õ���Ŷ*/
extern osMessageQId NRF_RXQueueHandle;
extern osThreadId ShootTaskHandle;

/**
 * @brief ������ⲿ�жϵĻص�������nrf���ж����Ż����յ���Ϣʱ���ͣ���������һ����Ϣ
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //�жϴ�������
{

    //������ң�����ĳ���
//

}
