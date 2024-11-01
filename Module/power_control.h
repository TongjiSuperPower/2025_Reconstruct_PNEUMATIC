#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_

#include "struct_typedef.h"
#include "can_task.h"
#include "gimbal_task.h"

#ifdef RADUCTION_RATIO_1
#define torque_coefficient 3.6621e-04f      //0.3*20/16384=3.6621e-04f
#endif
#ifdef RADUCTION_RATIO_2
#define torque_coefficient 2.6378e-04f      //0.3*20/16384=3.6621e-04f
#endif

#define K1 2.892f
#define K2 0.00681f
#define K3 2.891f


//���̹������ƺ���
void fn_chassis_power_control(Motor3508Data_t *Data1,Motor3508Data_t *Data2,Motor3508Data_t *Data3,Motor3508Data_t *Data4,fp32 Pmax);

//�ּ����ƹ���
fp32 fn_level_power_limit(fp32 infact_Pmax,uint16_t fact_buffer_energy);

//�ж��Ƿ����û������� ���������ֱ�Ϊ ����ϵͳ�����Ļ�������ֵ ����ϵͳ�������� ����ʹ�õĻ�������
fp32 fn_buffer_energy_judgement(uint16_t fact_buffer_energy,fp32 Pmax,fp32 USEmax);

//����ʹ���˶��ٻ������� ����������Ƶ�� ����ϵͳ��������
void fn_buffer_energy_used(uint16_t fre,fp32 Pmax);


extern fp32 Pin;
extern fp32 infact_Pmax;
extern fp32 count_Pmax;
extern fp32 filter_Pmax;
extern fp32 used_buffer_energy;

#endif
