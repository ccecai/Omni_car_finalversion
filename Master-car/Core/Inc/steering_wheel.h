#ifndef __STEER_WHEEL__H
#define __STEER_WHEEL__H


#include "main.h"
#include "stm32f4xx.h"
#include "math.h"

#define ABS(x)  ((x>0)? (x): -(x))

//���������ĵ����Ӿ���
#define DIS_CENTER2WHEEL  16.472f

#define angleBase_Front  (180.0f)
#define angleBase_Left   (-60.0f)
#define angleBase_Right  (60.0f)

#define WHEEL_FRONT_ID        (0x301)
#define WHEEL_LEFT_ID         (0x302)
#define WHEEL_RIGHT_ID        (0x303)

//�����֡��ٶȴ�С������
typedef struct
{
    uint32_t CAN_ID;
    float positon_angle;
	float velocity;              //�����ٶȴ�С
	float direction_angle;        //���ַ���
	float last_direction_angle;
}WheelStructTypedef;

//�������ֽṹ����Ϣ
typedef struct
{
    WheelStructTypedef FrontWheel;
    WheelStructTypedef LeftWheel;
    WheelStructTypedef RightWheel;
}RobotWheelStruct;

/*//�������ˡ��ٶȴ�С�뷽����3��4���������ٶ�ʸ������ͬ�������ṹ��
typedef struct
{
//	float vel;      //�������ٶȴ�С
//	float direction;//�������ٶȷ���
    float vel_x;    //�������ٶ�x����
    float vel_y;    //�������ٶ�y����
	float omega;    //��������ת���ٶȣ�����ͼ��ʱ��Ϊ������
}robot_vel_s;*/

union union_float
{
    uint8_t data_8[4];
    float data_32;
};

//c�ļ��б����������������ⲿ���ã�

extern RobotWheelStruct rrWheel;

//c�ļ��к�������
void SetRobotVel_Vector(RobotWheelStruct *robotWheel, float dir, float vel, float rotate, float robotAngle);
void SetRobotVel_XY(RobotWheelStruct *robotWheel, float vel_x, float vel_y, float rotate, float robotAngle);
void RobotLock(RobotWheelStruct *robotWheel);

void Robot2Wheel_Vector(WheelStructTypedef* wheelStruct, float dir, float vel, float rotate, float robotAngle);
void Robot2Wheel_XY(WheelStructTypedef* wheelStruct, float vel_x, float vel_y, float rotate, float robotAngle);
void AngleOptimize(WheelStructTypedef* wheelStruct);

float Rad2Angle(float radian);
void angleLimit(float *angle);
float Angle2Rad(float angle);

void SendWheelData(CAN_HandleTypeDef *_hcan, uint32_t wheel_id, float wheel_spd, float wheel_direction);

#endif
