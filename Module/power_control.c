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
fp32 K = 0.0f;              //缩放系数(-1,1)
//缓存能量强制功率限制缩小系数(0,1)
fp32 K_buffer = 0.0f;
//是否使用缓存能量
uint8_t buffer_energy_use_flag = 0;
//预期已经使用的缓存能量
fp32 used_buffer_energy = 0.0f;
//实际最大功率
fp32 infact_Pmax = 0.0f;
//计算功率上限，即实际上限-（0，5）
fp32 count_Pmax = 0.0f;
//滤波后功率上限，目的是使得功率上限阶跃不会过大
fp32 filter_Pmax = 0.0f;


//底盘功率限制函数
void fn_chassis_power_control(Motor3508Data_t *Data1,Motor3508Data_t *Data2,Motor3508Data_t *Data3,Motor3508Data_t *Data4,fp32 Pmax){

    fp32 t1,t2,t3,t4;
    fp32 a=0.0f,b=0.0f,c=0.0f,Ewt=0.0f,Ew2=0.0f,Et2=0.0f;

    //计算每个电机转矩  N.m
    t1 = torque_coefficient * Data1->given_current;
    t2 = torque_coefficient * Data2->given_current;
    t3 = torque_coefficient * Data3->given_current;
    t4 = torque_coefficient * Data4->given_current;


    //期望功率  
    Pin = //转轴输出功率
          t1 * Data1->relative_raw_speed + t2 * Data2->relative_raw_speed
        + t3 * Data3->relative_raw_speed + t4 * Data4->relative_raw_speed
          //铜损
          + K1 * (t1 * t1 + t2 * t2 + t3 * t3 + t4 * t4) 
          //铁损
          + K2 * (Data1->relative_raw_speed * Data1->relative_raw_speed + Data2->relative_raw_speed * Data2->relative_raw_speed
                + Data3->relative_raw_speed * Data3->relative_raw_speed + Data4->relative_raw_speed * Data4->relative_raw_speed)
          //静态损耗
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

//分级控制功率
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
        return infact_Pmax - 0.0f;
    }
    else{
        return infact_Pmax;
    }
}
