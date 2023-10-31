/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"
uint8_t Usart_TxPacket[256];				//FF 01 02 03 04 FE
uint8_t Usart_RxPacket[256];
uint8_t Usart_RxData;
uint8_t Usart_RxFlag=0;

uint8_t RxState = 0;
uint8_t pRxPacket = 0;
////现在
uint8_t i=0;
uint8_t send_buf[33] = {0};  

uint8_t state_mode=0;
uint8_t  clear_flag;
//////原来
uint8_t  mode = 0;                   //判断遥控模式或者取射模式
uint8_t  mode1 = 0;                  //具体判断

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
	 HAL_UART_Receive_IT(&huart1, (uint8_t*)&Usart_RxData, 1);  
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(pRxPacket>= 255)  //溢出判断
	{
		pRxPacket = 0;
		memset(Usart_RxPacket,0x00,sizeof(Usart_RxPacket));
	}
	else
	{
		 Usart_RxFlag=1;
	   Usart_RxPacket[pRxPacket++]= Usart_RxData;
		 Usart_RxFlag=1;
		 HAL_UART_Receive_IT(&huart1, (uint8_t*)&Usart_RxData, 1);  
//    if(Usart_RxPacket[pRxPacket-1] == 0xFE) 
//			{
//				switch(Usart_RxPacket[0])
//				{
//					case 0xFF:
//					{
//						send_buf[4]=Usart_RxPacket[1];
//						break;
//					}
//					case 0xBB:
//					{
//						state_mode=(state_mode)^(Usart_RxPacket[1]);
//						send_buf[5]=state_mode;
//						break;
//					}
//					case 0xAA:
//					{
//						if(Usart_RxPacket[1]<=255) send_buf[6]=Usart_RxPacket[1];
//						break;
//					}
//					case 0xCC:
//					{
//						if(Usart_RxPacket[1]<=100) send_buf[7]=Usart_RxPacket[1];
//						break;
//					}
//					case 0xEF:
//					{
//						send_buf[8]=Usart_RxPacket[1];
//						break;
//					}
//					case 0xDD:
//					{
//						send_buf[9]=Usart_RxPacket[1];
//						break;
//					}
//	     }
    if(Usart_RxPacket[pRxPacket-1] == 0xFF) 
			{
				switch(Usart_RxPacket[1])
				{
					case 0x01:
					{
					   printf("t1.txt=\"OK\"\xff\xff\xff");	
						 printf("n0.val=1\xff\xff\xff");	
             break;						
					}
					case 0x02:
					{
					   printf("t1.txt=\"error\"\xff\xff\xff");
						 printf("n0.val=2\xff\xff\xff");	
             break;						
					} 					
				}
          pRxPacket=0;
			    memset(Usart_RxPacket,0x00,sizeof(Usart_RxPacket));        //清空数组等待下一次接收
    }  
   }

}




void send_byte(uint8_t* data)
{
   HAL_UART_Transmit(&huart1,data,1,0xff);
}
int fputc(int ch, FILE *f) 
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xff);
  return ch;
}

/* USER CODE END 1 */
