#include "Bassel_Run.h"
#include "locater.h"
// 控制点的坐标
pointStruct Control_point[4]={{0.0f,0.0f,0},
                              {500.0f,-500.0f,0},
                              {700.0f,700.0f,0},
                              {700.0f,-700.0f,0}};
BasselLine_3  Bassel_Line3;
#define TIM_STEP 0.05f  // 时间步长
/**
  * @brief  根据起始点建立控制点相对起始点的相对坐标
  */
void Init_Bassel_controlpos(BasselLine_3 *bassel_line,pointStruct *control_pos,uint8_t num)
{
    for(uint8_t i=0; i<num;i++)
    {
        bassel_line->control_point[i].x = control_pos[i].x - bassel_line->start_pos.x;
        bassel_line->control_point[i].y = control_pos[i].y - bassel_line->start_pos.y;
    }
}
/**
  * @brief  因为贝塞尔曲线上的点是关于t的函数，采用先计算出该曲线的各项式系数再传参t计算点位置的方法  (相较直接传t减少了很多的算力消耗)
  * @param bassel_line 要计算的贝塞尔曲线
  */

void Bassel3_coefficient_cal(BasselLine_3 *bassel_line)
{
    bassel_line->x_coefficient[0] = bassel_line->control_point[0].x;
    bassel_line->x_coefficient[1] = 3 * (bassel_line->control_point[1].x-bassel_line->control_point[0].x);
    bassel_line->x_coefficient[2] = 3 * (bassel_line->control_point[2].x - 2 * bassel_line->control_point[1].x -bassel_line->control_point[0].x);
    bassel_line->x_coefficient[3] = bassel_line->control_point[3].x - 3 * bassel_line->control_point[2].x + 3*bassel_line->control_point[1].x - bassel_line->control_point[0].x;

    bassel_line->y_coefficient[0] = bassel_line->control_point[0].y;
    bassel_line->y_coefficient[1] = 3 * (bassel_line->control_point[1].y-bassel_line->control_point[0].y);
    bassel_line->y_coefficient[2] = 3 * (bassel_line->control_point[2].y - 2 * bassel_line->control_point[1].y -bassel_line->control_point[0].y);
    bassel_line->y_coefficient[3] = bassel_line->control_point[3].y - 3 * bassel_line->control_point[2].y + 3*bassel_line->control_point[1].y - bassel_line->control_point[0].y;
}
/**
  * @brief  根据t更新这个点在曲线上的位置
  * @param  point 要更新的点
  */
void ReNew_PosBy_t(BasselLine_3 *bassel_line , Point_On_Bassel *point)
{
    float t_1 = point->t;
    float t_2 = t_1*t_1;
    float t_3 = t_2 * t_1;
    point->x = bassel_line->x_coefficient[3]*t_3 + bassel_line->x_coefficient[2]*t_2 + bassel_line->x_coefficient[1]*t_1 + bassel_line->x_coefficient[0];
    point->y = bassel_line->y_coefficient[3]*t_3 + bassel_line->y_coefficient[2]*t_2 + bassel_line->y_coefficient[1]*t_1 + bassel_line->y_coefficient[0];
}
/**
  * @brief  初始化（更新(用于拼接)）贝塞尔曲线路径   先写在这版
  * @param  *control_pos 场上的控制点
  * @param  num 控制点个数
  */
void Update_Bassel_Line(BasselLine_3 *bassel_line , pointStruct *control_pos,uint8_t num)
{

    // 初始化曲线起始坐标
    bassel_line->start_pos.x = locater.pos_x;
    bassel_line->start_pos.y = locater.pos_y;
    //设置曲线的约束点位
    Init_Bassel_controlpos(bassel_line,control_pos,num);
    // 计算系数
    Bassel3_coefficient_cal(bassel_line);
    // 两个向量误差初始化（可能因为上一次的曲线而残留数据）
    bassel_line->Now2Last_Err.x = bassel_line->Next2last_Err.x = 0.0f;
    bassel_line->Now2Last_Err.y = bassel_line->Next2last_Err.y = 0.0f;
    // 总时间因子和上一次点的时间因子都置0
    bassel_line->total_t  = 0.0f;
    bassel_line->Last_locate.x = locater.pos_x;  // 更新曲线的时候就
    bassel_line->Last_locate.y = locater.pos_y;
    // 下一个目标点的时间因子
    bassel_line->Next_Point.t = 0.05f;
    //更新下一次目标点的位置 即所追的点位
    ReNew_PosBy_t(bassel_line,&bassel_line->Next_Point); // 这句可能没有实际效果
}
/**
  * @brief  生成时间因子t的规则 综合考虑x,y
  */
float Generate_t(float x1, float x2, float y1, float y2)
{
//    return TIM_STEP * 0.5f * (x1 / (x2 + 0.0001f)) + (y2 / (y2 + 0.0001f))
    return TIM_STEP * (x1 * x2 + y1 * y2)/(x2 * x2+y2 * y2 + 0.0001f);  // 0.0001f是为了防止出现0/0的现象 导致数据失效
}
/**
  * @brief  产生相应贝塞尔曲线上的目标点(所追的点)
  */
pointStruct Update_Target_Point(BasselLine_3 *bassel_line)
{
    // 输出的目标点位
    pointStruct out_point;
    bassel_line->Now2Last_Err.x = locater.pos_x - bassel_line->Last_locate.x;
    bassel_line->Now2Last_Err.y = locater.pos_y - bassel_line->Last_locate.y;
    bassel_line->Next2last_Err.x = bassel_line->Next_Point.x - bassel_line->Last_locate.x + bassel_line->start_pos.x;
    bassel_line->Next2last_Err.y = bassel_line->Next_Point.y - bassel_line->Last_locate.y + bassel_line->start_pos.y;

    //  衡量当前位置与下一时刻位置相较于上一时刻位置的马氏距离 产生时间因子
    bassel_line->total_t += Generate_t(bassel_line->Now2Last_Err.x,bassel_line->Next2last_Err.x,bassel_line->Now2Last_Err.y,bassel_line->Next2last_Err.y);
    if(bassel_line->total_t>1)
    {bassel_line->total_t = 1;}
    else if(bassel_line->total_t < 0)
    {bassel_line->total_t = 0.0f;}

    bassel_line->Last_locate.t = bassel_line->total_t;
    ReNew_PosBy_t(bassel_line,&bassel_line->Last_locate); // 更新点位
    bassel_line->Last_locate.x += bassel_line->start_pos.x;
    bassel_line->Last_locate.y += bassel_line->start_pos.y;

    bassel_line->Next_Point.t = bassel_line->Last_locate.t + TIM_STEP;
    if(bassel_line->Next_Point.t>1)
    {bassel_line->Next_Point.t = 1;}
    else if(bassel_line->Next_Point.t < 0)
    {bassel_line->Next_Point.t = TIM_STEP;} // 即使反向溢出下一次点的时间因子始终比上一次多一个时间步长
    ReNew_PosBy_t(bassel_line,&bassel_line->Next_Point);

    // 目标点位是下一次点位的相对坐标+起始坐标 = 在绝对世界坐标系下的位置
    out_point.x = bassel_line->Next_Point.x + bassel_line->start_pos.x;
    out_point.y = bassel_line->Next_Point.y + bassel_line->start_pos.y;

    return out_point;
}



