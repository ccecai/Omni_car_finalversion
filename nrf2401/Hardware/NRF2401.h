#ifndef NRF2401_H
#define NRF2401_H

#include "main.h"
#include "spi.h"


#define NRF_READ_REG    0x00  //读寄存器命令，后面需要加上寄存器地址
#define NRF_WRITE_REG   0x20  //写寄存器命令，后面需要加上寄存器地址
#define RD_RX_PLOAD     0x61  //从RX_FIFO中读收到的数据命令，1-32字节，读出后FIFO数据被删除。适用于接收模式。
#define WR_TX_PLOAD     0xA0  //向TX_FIFO写发射负载数据命令，大小为1-32字节，适用于发射模式。 
#define FLUSH_TX        0xE1  
#define FLUSH_RX        0xE2  
#define REUSE_TX_PL     0xE3  
#define NOP             0xFF   
#define CONFIG          0x00   //配置寄存器，8位
#define R_RX_PL_WID     0x60  //读取收到的数据字节数命令 


#define EN_AA           0x01  
#define EN_RXADDR       0x02  
#define SETUP_AW        0x03  
#define SETUP_RETR      0x04  
#define RF_CH           0x05  
#define RF_SETUP        0x06  
#define STATUS          0x07  
                             
#define MAX_TX  		0x10  
#define TX_OK   		0x20  
#define RX_OK   		0x40  
#define OBSERVE_TX      0x08  
#define CD              0x09  
#define RX_ADDR_P0      0x0A  
#define RX_ADDR_P1      0x0B  
#define RX_ADDR_P2      0x0C  
#define RX_ADDR_P3      0x0D  
#define RX_ADDR_P4      0x0E  
#define RX_ADDR_P5      0x0F  
#define TX_ADDR         0x10    
#define RX_PW_P0        0x11  
#define RX_PW_P1        0x12  
#define RX_PW_P2        0x13  
#define RX_PW_P3        0x14  
#define RX_PW_P4        0x15  
#define RX_PW_P5        0x16  
#define NRF_FIFO_STATUS 0x17  
#define TX_ADR_WIDTH    5   	
#define RX_ADR_WIDTH    5   	
#define TX_PLOAD_WIDTH  32  	
#define RX_PLOAD_WIDTH  32

#define NRF24L01_CSN_0 HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET)
#define NRF24L01_CSN_1 HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET)
#define NRF24L01_CE_0  HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_RESET)
#define NRF24L01_CE_1  HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_SET)
#define NRF24L01_IRQ   HAL_GPIO_ReadPin(IRQ_GPIO_Port,IRQ_Pin)


uint8_t NRF24L01_Check(void);

void NRF24L01_RX_Mode(void);
uint8_t NRF24L01_RxPacket(uint8_t* rxdata);
void test_rx(void);


void NRF24L01_TX_Mode(void);
uint8_t NRF24L01_TxPacket(uint8_t* txdata);
void test_tx(void);

#endif

