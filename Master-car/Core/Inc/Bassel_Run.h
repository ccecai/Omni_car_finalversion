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
    float x_coefficient[4];  // x的4个对应幂次系数
    float y_coefficient[4];  // y的4个对应幂次系数
    pointStruct start_pos;   // 曲线起始点的坐标
    float total_t;           // 总时间因子
    pointStruct Now2Last_Err;  // 当前点与上一次点的距离差
    pointStruct Next2last_Err; // 下一次点与上一次点的距离差

    Point_On_Bassel Last_locate; // 上一次点坐标
    Point_On_Bassel Next_Point;  // 下一次点的坐标
}BasselLine_3;



extern void Init_Bassel_controlpos(BasselLine_3 *bassel_line,pointStruct *control_pos,uint8_t num);

extern void Update_Bassel_Line(BasselLine_3 *bassel_line , pointStruct *control_pos,uint8_t num);
pointStruct Update_Target_Point(BasselLine_3 *bassel_line);
extern pointStruct Control_point[4];
extern BasselLine_3  Bassel_Line3;
#endif //MASTER_RR_BASSEL_RUN_H
