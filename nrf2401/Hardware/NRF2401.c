#include "NRF2401.h"
#include "spi.h"
#include "main.h"
#include "stdio.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0x01}; //发送地址
uint8_t RX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0x01}; //发送地址


uint8_t NRF2401_Write_buf(uint8_t regaddr,uint8_t* pdata,uint8_t len)
{
	 uint8_t status;
   NRF24L01_CSN_0;
	 status=SPI2_Send_byte(regaddr);//先说明是写命令以及寄存器地址
	 HAL_SPI_Transmit(&hspi2,pdata,len,0x10); //再写数据
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

uint8_t NRF24L01_TxPacket(uint8_t* txdata)//发送数据
{
   uint8_t status;
   NRF24L01_CE_0;//使能片选信号
	 NRF2401_Write_buf(WR_TX_PLOAD,txdata,TX_PLOAD_WIDTH);//调用WR_TX_PLOAD命令，将txdata写到TX FIFO中
	 NRF24L01_CE_1;//发送数据
   while(NRF24L01_IRQ!=0);//接收到自动应答，则调用中断NRF24L01_IRQ变为0
	 
	 status=SPI_Read_Reg(STATUS);
	 SPI_Write_Reg(NRF_WRITE_REG+STATUS,status);//清除发射完成中断
	 if(status&MAX_TX)//status第4位为1，表示达到最大重发次数
	 {
	     SPI_Write_Reg(FLUSH_TX,0XFF);//清空TX_FIFO
		 return MAX_TX;
	 }
		if(status&TX_OK)//status第5位为1，表示发射成功
	 {
		  return TX_OK;
	 }
	   return 0xff;
}

void NRF24L01_TX_Mode(void)//发送模式
{														 
	  NRF24L01_CE_0;	//片选信号有效    
  	NRF2401_Write_buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//NRF_WRITE_REG为读寄存器命令，TX_ADDR表示寄存器地址
  	NRF2401_Write_buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);

  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);//使能数据管道0自动应答    
  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能数据管道0接收
  	
	  SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//自动重发配置,00001->500uS重发一次，1010->重发10次
 
  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,0);//射频信道，2.4G+chMHz      
  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//射频配置,2Mbps,发送功率增益0dBm(11表示0dbm)  
	
	
  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0E);//接收中断使能,发射中断使能,最大重发计数中断使能,开启16位CRC校验,开机模式，发射模式
	
	  SPI_Write_Reg(FLUSH_TX,0XFF);//清空TX FIFO寄存器
	  SPI_Write_Reg(FLUSH_RX,0XFF);//清空RX FIFO寄存器
	
	  NRF24L01_CE_1;
}



void test_tx(void)//测试发射模式	
{
 
	 NRF24L01_TX_Mode();//设置模式函数一定要放在check函数后面，设置模式函数一定要放在check函数后面，设置模式函数一定要放在check函数后面
	 uint8_t buf[5]={4,22,33,44,20};
	 while(1)
   {
	    NRF24L01_TxPacket(buf);
		  printf("ok\r\n");
		  HAL_Delay(100);
	 };
}





void  NRF24L01_RX_Mode(void)//接收模式
{
   	NRF24L01_CE_0;	
    SPI_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//设置P0通道数据宽度，不能缺少	
	  NRF2401_Write_buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); //设置发射地址，用来发送应答  
  	NRF2401_Write_buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//设置通道0接收地址

  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0自动应答
  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //允许通道0接收
  	
	  SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//配置自动重发时间，次数
 
  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,0);        //配置工作频率
  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //配置数据传输速率，增益
	
	
  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0f);//接收模式
		
	  SPI_Write_Reg(FLUSH_RX,0XFF);//清空RX FIFO寄存器
	  
	  NRF24L01_CE_1;
}




uint8_t NRF24L01_RxPacket(uint8_t* rxdata)//接收数据
{
  unsigned char state;
	state=SPI_Read_Reg(STATUS);                //读取状态寄存器的值
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	if(state&RX_OK)                            //接收到数据
	{
		NRF2401_Read_buf(RD_RX_PLOAD,rxdata,RX_PLOAD_WIDTH);//读取数据
		SPI_Write_Reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器
		return 0;
	}
	return 1;        
}




void test_rx(void)//测试接收模式	
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




uint8_t NRF24L01_Check(void)//检查SPI读写是否正常
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






