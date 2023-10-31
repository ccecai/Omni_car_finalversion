//
// Created by WangCheng on 2023-10-16.
//
#include "Omni_Classis_task.h"

int count = 1;
int fu_flag = 0;

/*
 * �ú���������������ģ�ͽ��н���
 */
void Classis_Slove(FinalVelocity *target,OmniWheel *Wheel)
{
    Wheel->v_front = Speed_Increase * target->v_y + Distance * Pi * target->v_r/R_Decrease; //������������˵��Speed_Increase��R_Decrease��ӦΪһ����ֵ��������ʵ���в����ٶȽ������ƻ��߼��ٻ��С���������˻����޷��ﵽĿ�꣬����.h���޸ĺ궨��
    Wheel->v_left = -Speed_Increase * target->v_x + Distance * Pi * target->v_r/R_Decrease;
    Wheel->v_right = Speed_Increase * target->v_x + Distance * Pi * target->v_r/R_Decrease;
    Wheel->v_back = -Speed_Increase * target->v_y + Distance * Pi * target->v_r/R_Decrease;
}

/*
 * �ú����������ٶ�Ŀ��ֵ��ֵ
 */
void Disc_Get_Target(Target *target,float x,float y,float r,float angle)
{
    target->X_Speed = x + X_Start;
    target->Y_Speed = y + Y_Start;
    target->angle = angle;
    target->tan_angle = x/y;
}

void Get_Target(Target *target,float x,float y,float r,float angle)
{
    target->X_Speed = x;
    target->Y_Speed = y;
    target->R_Speed = r;
    target->angle = angle;
    target->tan_angle = x/y;
}

/*
 * �ú���Ϊ��ʹ�����ƶ�������ֱ�ߡ��Թ�����ٶȽ�������
 */
void Speed_Change(Target *target,FinalVelocity *Fv)
{

    if(fu_flag == 1)
    {
        Fourv.v_x = -Fourv.v_x;
        Fourv.v_y = -Fourv.v_y;
    }
    if(Fv->v_x > 0 && Fv->v_y < 0)
    {
        if (Fv->v_x > -Fv->v_y * target->tan_angle)
            Fv->v_x = -Fv->v_y * target->tan_angle;
        else if (Fv->v_x < -Fv->v_y * target->tan_angle)
            Fv->v_y = -Fv->v_x / target->tan_angle;
    }
    if(Fv->v_x < 0 && Fv->v_y > 0)
    {
        if (-Fv->v_x > Fv->v_y * target->tan_angle)
            Fv->v_x = -Fv->v_y * target->tan_angle;
        else if (-Fv->v_x < Fv->v_y * target->tan_angle)
            Fv->v_y = -Fv->v_x / target->tan_angle;
    }
    if(Fv->v_x > 0 && Fv->v_y > 0)
    {
        if (Fv->v_x > Fv->v_y * target->tan_angle)
            Fv->v_x = Fv->v_y * target->tan_angle;
        else if (Fv->v_x < Fv->v_y * target->tan_angle)
            Fv->v_y = Fv->v_x / target->tan_angle;
    }
    if(Fv->v_x < 0 && Fv->v_y < 0)
    {
        if (Fv->v_x < Fv->v_y * target->tan_angle)
            Fv->v_x = Fv->v_y * target->tan_angle;
        else if (Fv->v_x > Fv->v_y * target->tan_angle)
            Fv->v_y = Fv->v_x / target->tan_angle;
    }

    if(fu_flag == 1)
    {
        Fourv.v_x = -Fourv.v_x;
        Fourv.v_y = -Fourv.v_y;
        fu_flag = 0;
    }

}
/*
 * ��ͨ���ܵ㺯��
 * x_change��y_change��Ϊ�仯�ı�����ֵ��r_change��ʱ���ã�angle_changeΪ�仯���ĽǶ�
 */
float Run_Point(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r,float x_change,float y_change,float r_change,
                float angle_change,int straight_flag,int Bessel_flag)
{

    Row_Change(&FourTarget,x_change,y_change,r_change,angle_change);

    XYR_Speed_Charge(PID_x,PID_y,PID_r);

    if(straight_flag == 1)
    {
        Speed_Change(&FourTarget,&Fourv);
    }

    Classis_Slove(&Fourv,&FourWheel);

//        usart_printf("%f,%f,%f,%f,%f,%f\n",Fourv.v_x,Fourv.v_y,locater.pos_x,locater.pos_y,locater.angle,locater.continuousAngle);

    if(Bessel_flag == 1)
    {
        SendWheelData(&hcan1,M1_ID,FourWheel.v_front * 100.0f,0);
        SendWheelData(&hcan1,M2_ID,FourWheel.v_left * 100.0f,0);
        SendWheelData(&hcan1,M3_ID,FourWheel.v_right * 100.0f,0);
        SendWheelData(&hcan1,M4_ID,FourWheel.v_back * 100.0f,0);
    }
    else
    {
        if(((Fourv.v_x < 0.04f) && (Fourv.v_x > -0.04f)) && ((Fourv.v_y < 0.04f) && (Fourv.v_y > -0.04f)))
        {
            SendWheelData(&hcan1,M1_ID,0.0f,0);
            SendWheelData(&hcan1,M2_ID,0.0f,0);
            SendWheelData(&hcan1,M3_ID,0.0f,0);
            SendWheelData(&hcan1,M4_ID,0.0f,0);
            return 0;
        }
        else
        {
            SendWheelData(&hcan1,M1_ID,FourWheel.v_front * 100.0f,0);
            SendWheelData(&hcan1,M2_ID,FourWheel.v_left * 100.0f,0);
            SendWheelData(&hcan1,M3_ID,FourWheel.v_right * 100.0f,0);
            SendWheelData(&hcan1,M4_ID,FourWheel.v_back * 100.0f,0);
            return 1;
        }
    }




}


/*
 * �ú���������д�ض�����·�ߣ�Ŀǰ��дΪ���ú���һ��������
 */
void Run_RouteCube(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r,float x_change,float y_change,float r_change,float angle_change)
{
    static float last_x_change,last_y_change,last_r_change,last_angle_change;

    switch(count++)
    {
        case 1:
            Fourv.Done_flag = 1;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        case 2:
            Fourv.Done_flag = 1;
            x_change = last_x_change;
            y_change = last_y_change+500;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        case 3:
            Fourv.Done_flag = 1;
            x_change = last_x_change-500;
            y_change = last_y_change;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        case 4:
            Fourv.Done_flag = 1;
            x_change = last_x_change;
            y_change = last_y_change - 500;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        default:
            Fourv.Done_flag = 0;
            x_change = 0;
            y_change = 0;
            break;
    }

    while(Fourv.Done_flag)
    {
        Fourv.Done_flag = Run_Point(PID_x,PID_y,PID_r,x_change,y_change,r_change,0,0,0);
        usart_printf("%d,%f,%f\n",count,x_change,y_change);
    }

}
/*
 * �Ƕ�ת�����������Ƕ�ת��Ϊ����
 */
float Angle_Change(float angle)
{
    return angle*Pi/180;
}
/*
 * ������ת��Ϊ�Ƕ�
 */
float FanAngle_Change(float angle)
{
    return angle*180/Pi;
}
/*
 * �˺���Ϊ��С����������ϵת������������ϵ������������ʵ�ֶ�λ���ú���ʹ��С�����Ա��߱�ת
 * ����ԭ����Ҫ�϶���ѧ���㣬�����Լ��о�
 */
float Row_Change(Target *target,float x,float y,float r,float angle)
{
    if(((locater.angle > 0) && (locater.angle < FanAngle_Change(atan2f(y,x)))) && ((x != 0) || (y != 0)))
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(-locater.angle) + atan2f(y,x))/arm_sin_f32(Angle_Change(-locater.angle) + atan2f(y,x));
    }

    else if(((locater.angle < 90.0f) && (locater.angle > FanAngle_Change(atan2f(y,x)))) && ((x != 0) || (y != 0)) )
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = -y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(90-locater.angle) + atan2f(y,x))/arm_cos_f32(Angle_Change(90-locater.angle) + atan2f(y,x));

    }
    else if(((locater.angle > 90.0f) && (locater.angle < FanAngle_Change(atan2f(y,x)) + 90)) && ((x != 0) || (y != 0)) )
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = -y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(locater.angle) - atan2f(y,x))/arm_sin_f32(Angle_Change(locater.angle) - atan2f(y,x));

    }
    else if(((locater.angle <= 180.0f) && (locater.angle > FanAngle_Change(atan2f(y,x)) + 90)) && ((x != 0) || (y != 0)) )
    {
        fu_flag = 1;
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(locater.angle - 90) - atan2f(y,x))/arm_cos_f32(Angle_Change(locater.angle - 90) - atan2f(y,x));
//        usart_printf("%f,%f,%f,%f,%f\n",target->X_Speed,target->Y_Speed,locater.pos_x,locater.pos_y,locater.angle);
//        usart_printf("%f,%f,%f,%f,%f,%f\n",target->X_Speed,target->Y_Speed, Fourv.v_x,Fourv.v_y,locater.pos_x,locater.pos_y);
    }

    else if(((locater.angle < 0) && (locater.angle > -FanAngle_Change(atan2f(x,y))))  && (angle > 0) && ((x != 0) || (y != 0)))
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(90 + locater.angle) - atan2f(y,x))/arm_cos_f32(Angle_Change(90 + locater.angle) - atan2f(y,x));

    }
    else if(((locater.angle > -90.0f) && (locater.angle < -FanAngle_Change(atan2f(x,y)))) && ((x != 0) || (y != 0)))
    {
        target->X_Speed = -x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(-locater.angle) - atan2f(x,y))/arm_cos_f32(Angle_Change(-locater.angle) - atan2f(x,y));

    }
    else if(((locater.angle < -90.0f) && (locater.angle > -FanAngle_Change(atan2f(x,y)) - 90)) && ((x != 0) || (y != 0)) )
    {
        target->X_Speed = -x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(90 + locater.angle) + atan2f(x,y))/arm_sin_f32(Angle_Change(90 + locater.angle) + atan2f(x,y));

    }
    else if(((locater.angle >= -180.0f) && (locater.angle < -FanAngle_Change(atan2f(x,y)) - 90)) && ((x != 0) || (y != 0)) )
    {
        fu_flag = 1;
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(-locater.angle - 90) - atan2f(x,y))/arm_sin_f32(Angle_Change(-locater.angle - 90) - atan2f(x,y));

    }
    if((angle == 0) || ((x == 0) && (y == 0)))
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = x/y;

    }
}
/*
 * ԭ����ת����
 */
void Revolove(float r_change)
{
    Get_Target(&FourTarget,0,0,r_change,0);
    Classis_Slove(&FourTarget,&FourWheel);
    SendWheelData(&hcan1,M1_ID,FourWheel.v_front * 100.0f,0);
    SendWheelData(&hcan1,M2_ID,FourWheel.v_left * 100.0f,0);
    SendWheelData(&hcan1,M3_ID,FourWheel.v_right * 100.0f,0);
    SendWheelData(&hcan1,M4_ID,FourWheel.v_back * 100.0f,0);
}

/*
 * ��x,y�����ٶ�����ת���ٶȽ���pid����
 */
void XYR_Speed_Charge(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r)
{
    if (fu_flag == 1) //�������С��ת���ǶȽϴ�ʱ�ٶȽ������̷��������ߣ��ʸ��踺ֵ����ɵ�������Ŀ��ֵ��Ŀ��
    {
        Fourv.last_v_x = Fourv.v_x;
        Fourv.last_v_y = Fourv.v_y;
        Fourv.v_x = pid_calculate(PID_x, locater.pos_x * 100.0f, FourTarget.X_Speed * 100.0f);
        Fourv.v_y = pid_calculate(PID_y, locater.pos_y * 100.0f, FourTarget.Y_Speed * 100.0f);
    }
    else
    {
        Fourv.last_v_x = Fourv.v_x;
        Fourv.last_v_y = Fourv.v_y;
        Fourv.v_x = -pid_calculate(PID_x, locater.pos_x * 100.0f, FourTarget.X_Speed * 100.0f);
        Fourv.v_y = -pid_calculate(PID_y, locater.pos_y * 100.0f, FourTarget.Y_Speed * 100.0f);
    }

    Fourv.last_v_r = Fourv.v_r;
    Fourv.v_r = -pid_calculate(PID_r, locater.continuousAngle * 100.0f, FourTarget.angle * 100.0f);
    //ֱ�ӽ��н���С��ת���ӷ���תת������Ӧ�ϴ󣬹�ȡ��ֵ


}

void Remote_Control(RemoteRXSturct *RemoteRX)
{
    Fourv.v_y = RemoteRX->lx/6;
    Fourv.v_x = RemoteRX->ly/6;
    Fourv.v_r = RemoteRX->rx/25;

    Classis_Slove(&Fourv,&FourWheel);

    SendWheelData(&hcan1,M1_ID,FourWheel.v_front * 100.0f,0);
    SendWheelData(&hcan1,M2_ID,FourWheel.v_left * 100.0f,0);
    SendWheelData(&hcan1,M3_ID,FourWheel.v_right * 100.0f,0);
    SendWheelData(&hcan1,M4_ID,FourWheel.v_back * 100.0f,0);
}