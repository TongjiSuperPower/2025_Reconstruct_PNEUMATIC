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

// #define K1 2.892f
// #define K2 0.00821f
// #define K3 2.891f
#define K1 2.866f
#define K2 0.00851f
#define K3 2.6742f

#define SPEED_CON 30
#define W_CON 30

//���̹������ƺ���
void fn_chassis_power_control(Motor3508Data_t *Data1,Motor3508Data_t *Data2,Motor3508Data_t *Data3,Motor3508Data_t *Data4,fp32 Pmax);

//�ּ����ƹ���
fp32 fn_level_power_limit(fp32 infact_Pmax,uint16_t fact_buffer_energy);

void fn_chassis_speed_autoset(fp32 *w_set,fp32 *vx_set,fp32 *vy_set,uint8_t mode);

//速度重置判断
bool_t fn_chassis_speed_reset_speed(fp32 infact_Pmax);
//角速度重置判断
bool_t fn_chassis_speed_reset_w(fp32 infact_Pmax);

extern fp32 Pin;
extern fp32 K;
extern fp32 infact_Pmax;
extern fp32 count_Pmax;
extern fp32 filter_Pmax;
extern fp32 w_match;
extern fp32 vx_match;
extern fp32 vy_match;
extern uint8_t speed_match_flag;
extern uint8_t w_match_flag;

#endif
