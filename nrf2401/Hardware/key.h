#ifndef _KEY_H
#define _KEY_H
#include "main.h"

#define KEY1  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)//读取按键1
#define KEY2  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)//读取按键2
#define KEY3  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)//读取按键3
#define KEY4  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)//读取按键4
#define KEY5  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)//读取按键5
#define KEY6  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)//读取按键6
#define KEY7  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)//读取按键7
#define KEY8  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)//读取按键8


uint8_t KEY_Scan(uint8_t mode);

#endif