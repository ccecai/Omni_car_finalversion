#ifndef _KEY_H
#define _KEY_H
#include "main.h"

#define KEY1  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)//��ȡ����1
#define KEY2  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)//��ȡ����2
#define KEY3  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)//��ȡ����3
#define KEY4  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)//��ȡ����4
#define KEY5  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)//��ȡ����5
#define KEY6  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)//��ȡ����6
#define KEY7  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)//��ȡ����7
#define KEY8  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)//��ȡ����8


uint8_t KEY_Scan(uint8_t mode);

#endif