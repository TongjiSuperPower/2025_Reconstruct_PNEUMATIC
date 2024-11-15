#include "power_control.h"
#include "can_task.h"
#include "struct_typedef.h"
#include "chassis_task.h"
#include "math.h"
#include "stdlib.h"
#include "remote_control.h"
#include "supercap_task.h"
#include "math_lib.h"



fp32 Pin = 0.0f;
fp32 K = 0.0f;              //����ϵ��(-1,1)
//��������ǿ�ƹ���������Сϵ��(0,1)
fp32 K_buffer = 0.0f;
//ʵ�������
fp32 infact_Pmax = 0.0f;
//���㹦�����ޣ���ʵ������-��0��5��
fp32 count_Pmax = 0.0f;
//�˲��������ޣ�Ŀ����ʹ�ù������޽�Ծ�������
fp32 filter_Pmax = 0.0f;
//��ϵļ��ٶ��ٶ�
fp32 w_match;
fp32 vx_match;
fp32 vy_match;
//�Ƿ���Ϲ��ٶȵ�flag Ϊ0����δ��Ϲ� Ϊ1����Ϲ�
uint8_t speed_match_flag = 0;
//�Ƿ���Ϲ����ٶȵ�flag Ϊ0����δ��Ϲ� Ϊ1����Ϲ�
uint8_t w_match_flag = 0;


//���̹������ƺ���
void fn_chassis_power_control(Motor3508Data_t *Data1,Motor3508Data_t *Data2,Motor3508Data_t *Data3,Motor3508Data_t *Data4,fp32 Pmax){

    fp32 t1,t2,t3,t4;
    fp32 a=0.0f,b=0.0f,c=0.0f,Ewt=0.0f,Ew2=0.0f,Et2=0.0f;

    //����ÿ�����ת��  N.m
    t1 = torque_coefficient * Data1->given_current;
    t2 = torque_coefficient * Data2->given_current;
    t3 = torque_coefficient * Data3->given_current;
    t4 = torque_coefficient * Data4->given_current;


    //��������  
    Pin = //ת���������
          t1 * Data1->relative_raw_speed + t2 * Data2->relative_raw_speed
        + t3 * Data3->relative_raw_speed + t4 * Data4->relative_raw_speed
          //ͭ��
          + K1 * (t1 * t1 + t2 * t2 + t3 * t3 + t4 * t4) 
          //����
          + K2 * (Data1->relative_raw_speed * Data1->relative_raw_speed + Data2->relative_raw_speed * Data2->relative_raw_speed
                + Data3->relative_raw_speed * Data3->relative_raw_speed + Data4->relative_raw_speed * Data4->relative_raw_speed)
          //��̬���
          + K3;
    if(Pin < 0.0f){
        Pin = 0.0f;
    }

    if(Pin <= Pmax){
        K = 1.0f;
    }

    else if(Pin > Pmax){
//        Ewt = fabs(t1 * Data1->relative_raw_speed + t2 * Data2->relative_raw_speed
//                 + t3 * Data3->relative_raw_speed + t4 * Data4->relative_raw_speed);
		Ewt = t1 * Data1->relative_raw_speed + t2 * Data2->relative_raw_speed
            + t3 * Data3->relative_raw_speed + t4 * Data4->relative_raw_speed;
        Et2 = t1 * t1 + t2 * t2 + t3 * t3 + t4 * t4;
        Ew2 = Data1->relative_raw_speed * Data1->relative_raw_speed + Data2->relative_raw_speed * Data2->relative_raw_speed
            + Data3->relative_raw_speed * Data3->relative_raw_speed + Data4->relative_raw_speed * Data4->relative_raw_speed;

        a = K1 * Et2;
        b = Ewt;
        c = K2 * Ew2 + K3 - Pmax;

        if((b*b - 4*a*c) >= 0.0f){
            K = (-b + sqrt(b*b - 4*a*c)) / (2*a);
        }
        else{
            K = 1.0f;
        }

        if(K < -1.0f || K > 1.0f){
            K = 1.0f;
        }
    }
    Data1->given_current = K_buffer * K * Data1->given_current;
    Data2->given_current = K_buffer * K * Data2->given_current;
    Data3->given_current = K_buffer * K * Data3->given_current;
    Data4->given_current = K_buffer * K * Data4->given_current;
}

//�ٶȽ��ٶ�����ϣ���֤�˶�ƽ���ԣ�ÿ200ms����һ�� mode:0������ȫ����ϣ�1����Ͻ��ٶȣ�2�����ǰ���ٶ�
void fn_chassis_speed_autoset(fp32 *w_set,fp32 *vx_set,fp32 *vy_set,uint8_t mode){
    fp32 param = 1.0f;
    switch (mode){
        case 0:
        {
            *w_set = *w_set * param;
            *vx_set = *vx_set * param;
            *vy_set = *vy_set * param;
            break;
        }
        case 1:
        {
            if(fabs(K) < 1 && K != 0.0f){
                param = (fabs(K) + W_CON) / (1 + W_CON);
            }
            else{
                param = 1.0f;
            }
            *w_set = *w_set * param;
            if(K == 1){
		        w_match_flag = 1;
                *w_set += 0.3f;
	        }
            break;
        }
        case 2:
        {
            if(fabs(K) < 1 && K != 0.0f){
                param = (fabs(K) + SPEED_CON) / (1 + SPEED_CON);
            }
            else{
                param = 1.0f;
            }
            *vx_set = *vx_set * param;
            if(K == 1){
                speed_match_flag = 1;
                *vx_set += 10.0f;
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

//�ٶ������ж�
bool_t fn_chassis_speed_reset_speed(fp32 infact_Pmax){
    static fp32 last_infact_Pmax = 0.0f;
    if(last_infact_Pmax == infact_Pmax){
        return 0;
    }
    last_infact_Pmax = infact_Pmax;
    return 1;
}
//���ٶ������ж�
bool_t fn_chassis_speed_reset_w(fp32 infact_Pmax){
    static fp32 last_infact_Pmax = 0.0f;
    if(last_infact_Pmax == infact_Pmax){
        return 0;
    }
    last_infact_Pmax = infact_Pmax;
    return 1;
}

//�ּ����ƹ���
fp32 fn_level_power_limit(fp32 infact_Pmax,uint16_t fact_buffer_energy){
	fn_Fp32Limit(&infact_Pmax,45.0f,CAP_MAX_POWER);
    K_buffer = (fact_buffer_energy / 15.0f);
    fn_Fp32Limit(&K_buffer,0.0f,1.0f);
    if(infact_Pmax <= 60.0f){
        return infact_Pmax - 3.0f;
    }
    else if(60.0f < infact_Pmax && infact_Pmax <= 80.0f){
        return infact_Pmax - 4.0f;
    }
    else if(80.0f < infact_Pmax && infact_Pmax <= 120.0f){
        return infact_Pmax - 3.0f;
    }
    else{
        return infact_Pmax;
    }
}
