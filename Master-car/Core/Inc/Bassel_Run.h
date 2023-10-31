//
// Created by hyz on 2023/10/25.
//

#ifndef __BASSEL_RUN__H
#define __BASSEL_RUN__H
#include "locater.h"


typedef struct
{
    float x;
    float y;
    float t;
}Point_On_Bassel;

typedef struct
{
    pointStruct control_point[4];
    float x_coefficient[4];  // x��4����Ӧ�ݴ�ϵ��
    float y_coefficient[4];  // y��4����Ӧ�ݴ�ϵ��
    pointStruct start_pos;   // ������ʼ�������
    float total_t;           // ��ʱ������
    pointStruct Now2Last_Err;  // ��ǰ������һ�ε�ľ����
    pointStruct Next2last_Err; // ��һ�ε�����һ�ε�ľ����

    Point_On_Bassel Last_locate; // ��һ�ε�����
    Point_On_Bassel Next_Point;  // ��һ�ε������
}BasselLine_3;



extern void Init_Bassel_controlpos(BasselLine_3 *bassel_line,pointStruct *control_pos,uint8_t num);

extern void Update_Bassel_Line(BasselLine_3 *bassel_line , pointStruct *control_pos,uint8_t num);
pointStruct Update_Target_Point(BasselLine_3 *bassel_line);
extern pointStruct Control_point[4];
extern BasselLine_3  Bassel_Line3;
#endif //MASTER_RR_BASSEL_RUN_H
