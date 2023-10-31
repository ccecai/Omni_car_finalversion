#include "kinematic.h"
#include "stm32f4xx_it.h"

/**
 * @brief ��λ��ʼ��
 * @param position ��λ�ṹ��
 */
void Position_Init(PositionStructure *position)
{
	position->x=0;
	position->y=0;
	position->theta=0;
	
	position->lastx=0.0f;
	position->lasty=0.0f;
}