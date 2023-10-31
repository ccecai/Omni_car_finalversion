/**
 * @brief �������������˶�ѧ�����Լ�һЩ��صĺ���
 */
#include "steering_wheel.h"

RobotWheelStruct rrWheel = {{WHEEL_FRONT_ID, angleBase_Front, 0, 0, 0},
                            {WHEEL_LEFT_ID,  angleBase_Left, 0, 0, 0},
                            {WHEEL_RIGHT_ID, angleBase_Right, 0, 0, 0}};

/**
 * @brief �趨�����˵��ٶȣ����ѻ����˵�Ŀ���ٶ�ת����������ϵ���ٶ�ʸ�������·������̿��ư���,��������ʹ�õĿ��ƺ���
 * @param robotVel ������Ŀ���ٶ���Ϣ�ṹ��
 * @param robotAngle �����˵�ǰ�Ƕ�
 */
void SetRobotVel_Vector(RobotWheelStruct *robotWheel, float dir, float vel, float rotate, float robotAngle)
{
    if(ABS(vel) + ABS(rotate) > 0.00001f)
    {
        Robot2Wheel_Vector(&robotWheel->FrontWheel, dir, vel, rotate, robotAngle);
        AngleOptimize(&robotWheel->FrontWheel);

        Robot2Wheel_Vector(&robotWheel->LeftWheel, dir, vel, rotate, robotAngle);
        AngleOptimize(&robotWheel->LeftWheel);

        Robot2Wheel_Vector(&robotWheel->RightWheel, dir, vel, rotate, robotAngle);
        AngleOptimize(&robotWheel->RightWheel);
    }
    else
    {
        robotWheel->FrontWheel.velocity = 0;
        robotWheel->LeftWheel.velocity = 0;
        robotWheel->RightWheel.velocity = 0;
    }
    SendWheelData(&hcan2, robotWheel->FrontWheel.CAN_ID, robotWheel->FrontWheel.velocity, robotWheel->FrontWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->LeftWheel.CAN_ID, robotWheel->LeftWheel.velocity, robotWheel->LeftWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->RightWheel.CAN_ID, robotWheel->RightWheel.velocity, robotWheel->RightWheel.direction_angle);

}

void SetRobotVel_XY(RobotWheelStruct *robotWheel, float vel_x, float vel_y, float rotate, float robotAngle)
{
    if(ABS(vel_x) + ABS(vel_y) + ABS(rotate) > 0.00001f)
    {
        Robot2Wheel_XY(&robotWheel->FrontWheel, vel_x, vel_y, rotate, robotAngle);
        AngleOptimize(&robotWheel->FrontWheel);

        Robot2Wheel_XY(&robotWheel->LeftWheel, vel_x, vel_y, rotate, robotAngle);
        AngleOptimize(&robotWheel->LeftWheel);

        Robot2Wheel_XY(&robotWheel->RightWheel, vel_x, vel_y, rotate, robotAngle);
        AngleOptimize(&robotWheel->RightWheel);

    }
    else
    {
        robotWheel->FrontWheel.velocity = 0;
        robotWheel->LeftWheel.velocity = 0;
        robotWheel->RightWheel.velocity = 0;
    }
    SendWheelData(&hcan2, robotWheel->FrontWheel.CAN_ID, robotWheel->FrontWheel.velocity, robotWheel->FrontWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->LeftWheel.CAN_ID, robotWheel->LeftWheel.velocity, robotWheel->LeftWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->RightWheel.CAN_ID, robotWheel->RightWheel.velocity, robotWheel->RightWheel.direction_angle);


}

/**
 * @brief ���������ӵ���������״̬
 */
void RobotLock(RobotWheelStruct *robotWheel)
{
    robotWheel->FrontWheel.last_direction_angle = robotWheel->FrontWheel.direction_angle;
    robotWheel->FrontWheel.direction_angle = robotWheel->FrontWheel.positon_angle + 90.0f;
    robotWheel->FrontWheel.velocity = 0;
    AngleOptimize(&robotWheel->FrontWheel);

    robotWheel->LeftWheel.last_direction_angle = robotWheel->LeftWheel.direction_angle;
    robotWheel->LeftWheel.direction_angle = robotWheel->LeftWheel.positon_angle + 90.0f;
    robotWheel->LeftWheel.velocity = 0;
    AngleOptimize(&robotWheel->LeftWheel);

    robotWheel->RightWheel.last_direction_angle = robotWheel->RightWheel.direction_angle;
    robotWheel->RightWheel.direction_angle = robotWheel->RightWheel.positon_angle + 90.0f;
    robotWheel->RightWheel.velocity = 0;
    AngleOptimize(&robotWheel->RightWheel);

    SendWheelData(&hcan2, robotWheel->FrontWheel.CAN_ID, robotWheel->FrontWheel.velocity, robotWheel->FrontWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->LeftWheel.CAN_ID, robotWheel->LeftWheel.velocity, robotWheel->LeftWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->RightWheel.CAN_ID, robotWheel->RightWheel.velocity, robotWheel->RightWheel.direction_angle);
}

void Robot2Wheel_Vector(WheelStructTypedef* wheelStruct, float dir, float vel, float rotate, float robotAngle)
{
	float vel_x;
    float vel_y; //ƽ���ٶ���xy����ķ���
    float vel_rotate;               //��ת���������ٶ�
    float sumVel_x;
    float sumVel_y; //ƽ���ٶ���xy����ķ���

    /*�ȼ���ƽ���ٶȵķ���*/
    vel_x = vel * arm_cos_f32(dir);
    vel_y = vel * arm_sin_f32(dir);
    /*�ټ�����ת���������ٶ�*/
    vel_rotate = Angle2Rad(rotate) * DIS_CENTER2WHEEL;   //��������ת���ٶȴ�СΪ���ٶȳ����ֵ����ľ��루��λmm��

	/*����������������*/
	sumVel_x = vel_x + vel_rotate * arm_cos_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));  //��һ���õ��ɣ�ֱ�Ӱ�һ��ƽ���ȷֵ�xy�����ϣ�������ת���ٶȷֵ�xy�����ϣ��������xy��������ֵ�ÿ����������ȷ���ķ�����ٶȣ�
	sumVel_y = vel_y + vel_rotate * arm_sin_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));

    /*����������ӵ��ٶȴ�С����ֵ*/
	arm_sqrt_f32(sumVel_x * sumVel_x + sumVel_y * sumVel_y, &wheelStruct->velocity);

    /*����������ӵ��ٶȷ���*/
    wheelStruct->last_direction_angle = wheelStruct->direction_angle;
    wheelStruct->direction_angle = Rad2Angle(atan2f(sumVel_y, sumVel_x)) - robotAngle;
}

void Robot2Wheel_XY(WheelStructTypedef* wheelStruct, float vel_x, float vel_y, float rotate, float robotAngle)
{
    float vel_rotate;               //��ת���������ٶ�
    float sumVel_x;
    float sumVel_y; //ƽ���ٶ���xy����ķ���

    /*�ټ�����ת���������ٶ�*/
    vel_rotate = Angle2Rad(rotate) * DIS_CENTER2WHEEL;   //��������ת���ٶȴ�СΪ���ٶȳ����ֵ����ľ��루��λmm��

    /*����������������*/
    sumVel_x = vel_x + vel_rotate * arm_cos_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));  //��һ���õ��ɣ�ֱ�Ӱ�һ��ƽ���ȷֵ�xy�����ϣ�������ת���ٶȷֵ�xy�����ϣ��������xy��������ֵ�ÿ����������ȷ���ķ�����ٶȣ�
    sumVel_y = vel_y + vel_rotate * arm_sin_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));

    /*����������ӵ��ٶȴ�С����ֵ*/
    arm_sqrt_f32(sumVel_x * sumVel_x + sumVel_y * sumVel_y, &wheelStruct->velocity);

    /*����������ӵ��ٶȷ���*/
    wheelStruct->last_direction_angle = wheelStruct->direction_angle;
    wheelStruct->direction_angle = Rad2Angle(atan2f(sumVel_y, sumVel_x)) - robotAngle;
}


void AngleOptimize(WheelStructTypedef* wheelStruct)
{
    float wheel_angle_differ;

    wheel_angle_differ = wheelStruct->direction_angle - wheelStruct->last_direction_angle;
    angleLimit(&wheel_angle_differ);

    if(ABS(wheel_angle_differ) <= 90.0f)
    {
        wheelStruct->direction_angle = wheelStruct->last_direction_angle + wheel_angle_differ;
    }
    else if(wheel_angle_differ < -90.0f)
    {
        wheelStruct->direction_angle = wheelStruct->last_direction_angle + wheel_angle_differ + 180.0f;
        wheelStruct->velocity = -wheelStruct->velocity;
    }
    else if(wheel_angle_differ > 90.0f)
    {
        wheelStruct->direction_angle = wheelStruct->last_direction_angle + wheel_angle_differ - 180.0f;
        wheelStruct->velocity = -wheelStruct->velocity;
    }
}

/**
 * @brief ���Ƕ�������-180~180��
 * @param angle
 */
void angleLimit(float *angle)
{
	if(*angle>180)
	{
		*angle-=360;
        angleLimit(angle);
	}
	else if(*angle<=-180)
	{
		*angle+=360;
        angleLimit(angle);
	}
}

//�Ƕȵ�λת��1
float Angle2Rad(float angle)
{
	return angle/180.0f*3.14159265359f;
}
//�Ƕȵ�λת��2
float Rad2Angle(float radian)
{
	return radian/3.14159265359f*180;
}

/**
 * @brief ���²���ӷ�����ϵ������Ϣ
 * @param _hcan &hcan1 or 2
 * @param wheel_id ��canid
 * @param wheel_spd ��ϵ�ٶ�
 * @param wheel_direction ��ϵ����
 */
void SendWheelData(CAN_HandleTypeDef *_hcan, uint32_t wheel_id, float wheel_spd, float wheel_direction)
{
    static CAN_TxHeaderTypeDef TxHeader;    //���ͱ��Ľṹ�嶨��
    static uint8_t TxData[8];
    static uint32_t mbox;
    static union union_float union_32f;

    TxHeader.StdId = wheel_id;     // id��ֵ
    TxHeader.IDE = 0;              // ��׼֡
    TxHeader.RTR = 0;              //
    TxHeader.DLC = 8;              // 8�ֽ�����֡

    union_32f.data_32 = wheel_spd;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_32 = wheel_direction;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ��������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //����ʧ�ܾͿ�ס��
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
