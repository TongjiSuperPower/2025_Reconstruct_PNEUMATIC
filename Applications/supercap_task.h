#ifndef _SUPERCAP_TASK_H_
#define _SUPERCAP_TASK_H_

#include "struct_typedef.h"
#include "can_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#define Cap_Offline_Debug 0
#define Referee_System 1
#define SUPERCAP_TASK_INIT_TIME 100
#define SUPERCAP_CONTROL_TIME 1
#define CAP_MAX_POWER 400

#define SUPER_CAP_ON 1


typedef enum{
	cap_auto_charge = 0, //flag = 0		自动模式（默认）
	cap_dis_charge  = 1, //flag = 1		只充不放
	cap_en_ucharge  = 2, //flag = 2		只放不充（现阶段不可用）
	cap_dis_ucharge = 3, //flag = 3     充放都禁止
	
	cap_just_disable = 4,
}cap_state;

typedef struct{
	fp32 Chassis_power;
	fp32 chassis_power_buffer;
	fp32 Chassis_power_limit;
	fp32 remain_power;
	fp32 remain_buffer;
	
}chassis_state_t;

typedef struct{
	pid_type_def cap_charge_pid;
	fp32 cap_charge_set;
}cap_control_t;

extern uint16_t cap_num;
extern int32_t Power_Limitation_Num;
extern int32_t Residue_Power;
extern int32_t Lost_Connection_Count;
extern int32_t Chassis_Power;
extern cap_state cap_FSM;
extern cap_state cap_FSM_ex;
extern uint8_t low_vol_flag;


#endif
