#include "Bassel_Run.h"
#include "locater.h"
// ���Ƶ������
pointStruct Control_point[4]={{0.0f,0.0f,0},
                              {500.0f,-500.0f,0},
                              {700.0f,700.0f,0},
                              {700.0f,-700.0f,0}};
BasselLine_3  Bassel_Line3;
#define TIM_STEP 0.05f  // ʱ�䲽��
/**
  * @brief  ������ʼ�㽨�����Ƶ������ʼ����������
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
  * @brief  ��Ϊ�����������ϵĵ��ǹ���t�ĺ����������ȼ���������ߵĸ���ʽϵ���ٴ���t�����λ�õķ���  (���ֱ�Ӵ�t�����˺ܶ����������)
  * @param bassel_line Ҫ����ı���������
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
  * @brief  ����t����������������ϵ�λ��
  * @param  point Ҫ���µĵ�
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
  * @brief  ��ʼ��������(����ƴ��)������������·��   ��д�����
  * @param  *control_pos ���ϵĿ��Ƶ�
  * @param  num ���Ƶ����
  */
void Update_Bassel_Line(BasselLine_3 *bassel_line , pointStruct *control_pos,uint8_t num)
{

    // ��ʼ��������ʼ����
    bassel_line->start_pos.x = locater.pos_x;
    bassel_line->start_pos.y = locater.pos_y;
    //�������ߵ�Լ����λ
    Init_Bassel_controlpos(bassel_line,control_pos,num);
    // ����ϵ��
    Bassel3_coefficient_cal(bassel_line);
    // ������������ʼ����������Ϊ��һ�ε����߶��������ݣ�
    bassel_line->Now2Last_Err.x = bassel_line->Next2last_Err.x = 0.0f;
    bassel_line->Now2Last_Err.y = bassel_line->Next2last_Err.y = 0.0f;
    // ��ʱ�����Ӻ���һ�ε��ʱ�����Ӷ���0
    bassel_line->total_t  = 0.0f;
    bassel_line->Last_locate.x = locater.pos_x;  // �������ߵ�ʱ���
    bassel_line->Last_locate.y = locater.pos_y;
    // ��һ��Ŀ����ʱ������
    bassel_line->Next_Point.t = 0.05f;
    //������һ��Ŀ����λ�� ����׷�ĵ�λ
    ReNew_PosBy_t(bassel_line,&bassel_line->Next_Point); // ������û��ʵ��Ч��
}
/**
  * @brief  ����ʱ������t�Ĺ��� �ۺϿ���x,y
  */
float Generate_t(float x1, float x2, float y1, float y2)
{
//    return TIM_STEP * 0.5f * (x1 / (x2 + 0.0001f)) + (y2 / (y2 + 0.0001f))
    return TIM_STEP * (x1 * x2 + y1 * y2)/(x2 * x2+y2 * y2 + 0.0001f);  // 0.0001f��Ϊ�˷�ֹ����0/0������ ��������ʧЧ
}
/**
  * @brief  ������Ӧ�����������ϵ�Ŀ���(��׷�ĵ�)
  */
pointStruct Update_Target_Point(BasselLine_3 *bassel_line)
{
    // �����Ŀ���λ
    pointStruct out_point;
    bassel_line->Now2Last_Err.x = locater.pos_x - bassel_line->Last_locate.x;
    bassel_line->Now2Last_Err.y = locater.pos_y - bassel_line->Last_locate.y;
    bassel_line->Next2last_Err.x = bassel_line->Next_Point.x - bassel_line->Last_locate.x + bassel_line->start_pos.x;
    bassel_line->Next2last_Err.y = bassel_line->Next_Point.y - bassel_line->Last_locate.y + bassel_line->start_pos.y;

    //  ������ǰλ������һʱ��λ���������һʱ��λ�õ����Ͼ��� ����ʱ������
    bassel_line->total_t += Generate_t(bassel_line->Now2Last_Err.x,bassel_line->Next2last_Err.x,bassel_line->Now2Last_Err.y,bassel_line->Next2last_Err.y);
    if(bassel_line->total_t>1)
    {bassel_line->total_t = 1;}
    else if(bassel_line->total_t < 0)
    {bassel_line->total_t = 0.0f;}

    bassel_line->Last_locate.t = bassel_line->total_t;
    ReNew_PosBy_t(bassel_line,&bassel_line->Last_locate); // ���µ�λ
    bassel_line->Last_locate.x += bassel_line->start_pos.x;
    bassel_line->Last_locate.y += bassel_line->start_pos.y;

    bassel_line->Next_Point.t = bassel_line->Last_locate.t + TIM_STEP;
    if(bassel_line->Next_Point.t>1)
    {bassel_line->Next_Point.t = 1;}
    else if(bassel_line->Next_Point.t < 0)
    {bassel_line->Next_Point.t = TIM_STEP;} // ��ʹ���������һ�ε��ʱ������ʼ�ձ���һ�ζ�һ��ʱ�䲽��
    ReNew_PosBy_t(bassel_line,&bassel_line->Next_Point);

    // Ŀ���λ����һ�ε�λ���������+��ʼ���� = �ھ�����������ϵ�µ�λ��
    out_point.x = bassel_line->Next_Point.x + bassel_line->start_pos.x;
    out_point.y = bassel_line->Next_Point.y + bassel_line->start_pos.y;

    return out_point;
}



