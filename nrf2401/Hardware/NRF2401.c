#include "NRF2401.h"
#include "spi.h"
#include "main.h"
#include "stdio.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0x01}; //���͵�ַ
uint8_t RX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0x01}; //���͵�ַ


uint8_t NRF2401_Write_buf(uint8_t regaddr,uint8_t* pdata,uint8_t len)
{
	 uint8_t status;
   NRF24L01_CSN_0;
	 status=SPI2_Send_byte(regaddr);//��˵����д�����Լ��Ĵ�����ַ
	 HAL_SPI_Transmit(&hspi2,pdata,len,0x10); //��д����
	 NRF24L01_CSN_1;
	 return status;
}


uint8_t NRF2401_Read_buf(uint8_t regaddr,uint8_t* pdata,uint8_t len)
{
	 uint8_t status;
   NRF24L01_CSN_0;
	 status=SPI2_Send_byte(regaddr);
	 HAL_SPI_Receive(&hspi2,pdata,len,0x1f); 
	 NRF24L01_CSN_1;
	 return status;
}

uint8_t NRF24L01_TxPacket(uint8_t* txdata)//��������
{
   uint8_t status;
   NRF24L01_CE_0;//ʹ��Ƭѡ�ź�
	 NRF2401_Write_buf(WR_TX_PLOAD,txdata,TX_PLOAD_WIDTH);//����WR_TX_PLOAD�����txdataд��TX FIFO��
	 NRF24L01_CE_1;//��������
   while(NRF24L01_IRQ!=0);//���յ��Զ�Ӧ��������ж�NRF24L01_IRQ��Ϊ0
	 
	 status=SPI_Read_Reg(STATUS);
	 SPI_Write_Reg(NRF_WRITE_REG+STATUS,status);//�����������ж�
	 if(status&MAX_TX)//status��4λΪ1����ʾ�ﵽ����ط�����
	 {
	     SPI_Write_Reg(FLUSH_TX,0XFF);//���TX_FIFO
		 return MAX_TX;
	 }
		if(status&TX_OK)//status��5λΪ1����ʾ����ɹ�
	 {
		  return TX_OK;
	 }
	   return 0xff;
}

void NRF24L01_TX_Mode(void)//����ģʽ
{														 
	  NRF24L01_CE_0;	//Ƭѡ�ź���Ч    
  	NRF2401_Write_buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//NRF_WRITE_REGΪ���Ĵ������TX_ADDR��ʾ�Ĵ�����ַ
  	NRF2401_Write_buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);

  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);//ʹ�����ݹܵ�0�Զ�Ӧ��    
  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ�����ݹܵ�0����
  	
	  SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�Զ��ط�����,00001->500uS�ط�һ�Σ�1010->�ط�10��
 
  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,0);//��Ƶ�ŵ���2.4G+chMHz      
  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//��Ƶ����,2Mbps,���͹�������0dBm(11��ʾ0dbm)  
	
	
  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0E);//�����ж�ʹ��,�����ж�ʹ��,����ط������ж�ʹ��,����16λCRCУ��,����ģʽ������ģʽ
	
	  SPI_Write_Reg(FLUSH_TX,0XFF);//���TX FIFO�Ĵ���
	  SPI_Write_Reg(FLUSH_RX,0XFF);//���RX FIFO�Ĵ���
	
	  NRF24L01_CE_1;
}



void test_tx(void)//���Է���ģʽ	
{
 
	 NRF24L01_TX_Mode();//����ģʽ����һ��Ҫ����check�������棬����ģʽ����һ��Ҫ����check�������棬����ģʽ����һ��Ҫ����check��������
	 uint8_t buf[5]={4,22,33,44,20};
	 while(1)
   {
	    NRF24L01_TxPacket(buf);
		  printf("ok\r\n");
		  HAL_Delay(100);
	 };
}





void  NRF24L01_RX_Mode(void)//����ģʽ
{
   	NRF24L01_CE_0;	
    SPI_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//����P0ͨ�����ݿ�ȣ�����ȱ��	
	  NRF2401_Write_buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); //���÷����ַ����������Ӧ��  
  	NRF2401_Write_buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//����ͨ��0���յ�ַ

  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0�Զ�Ӧ��
  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //����ͨ��0����
  	
	  SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط�ʱ�䣬����
 
  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,0);        //���ù���Ƶ��
  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //�������ݴ������ʣ�����
	
	
  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0f);//����ģʽ
		
	  SPI_Write_Reg(FLUSH_RX,0XFF);//���RX FIFO�Ĵ���
	  
	  NRF24L01_CE_1;
}




uint8_t NRF24L01_RxPacket(uint8_t* rxdata)//��������
{
  unsigned char state;
	state=SPI_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)                            //���յ�����
	{
		NRF2401_Read_buf(RD_RX_PLOAD,rxdata,RX_PLOAD_WIDTH);//��ȡ����
		SPI_Write_Reg(FLUSH_RX,0xff);          //���RX FIFO�Ĵ���
		return 0;
	}
	return 1;        
}




void test_rx(void)//���Խ���ģʽ	
{
	 NRF24L01_RX_Mode(); 
	 uint8_t buf[10];
	 uint8_t datalen,i;
  while (1)
  {
		if(NRF24L01_RxPacket(buf)==0)
	 {
	    datalen=buf[0];
		  for(i=1;i<=datalen;i++)
		 {
		   	printf("%d\r\n",buf[i]);	
        HAL_Delay(10);			 
		 }
	 }
  }
}




uint8_t NRF24L01_Check(void)//���SPI��д�Ƿ�����
{
	uint8_t buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	uint8_t buf1[5];
	uint8_t i;
	NRF2401_Write_buf(NRF_WRITE_REG+TX_ADDR,buf,5);
	NRF2401_Read_buf(TX_ADDR,buf1,5); 
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0xA5)
		break;	 							   
	}
	if(i!=5)return 1;
	return 0;		
}	






