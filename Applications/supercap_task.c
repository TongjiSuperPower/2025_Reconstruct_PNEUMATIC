#include "supercap_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "can_task.h"
#include "remote_control.h"
#include "referee_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"


void fn_cap_init(cap_control_t *cap_control_init);
void fn_cap_Contorl(cap_control_t *cap_control);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t supercap_high_water;
#endif

//进入电容放电延时时间
uint16_t cap_num = 0;
//电压大于22V则允许开始放电
uint8_t cap_en_flag = 0;

int32_t Power_Limitation_Num=55000;
int32_t Residue_Power=55000/25;//55W
int32_t Last_reside_power=0;
int8_t CAN2_Cap_flag=0;
int8_t Cap_Switch_Flag=0;

int32_t Lost_Connection_Count=0;
uint8_t Cap_Switch_Count=0;

int32_t Cap_Toutuous_Uppest=25000;
int32_t Cap_Toutuous_Up=24000;
int32_t Cap_Toutuous_Down=20000;

int32_t Chassis_Power=0;

cap_state cap_FSM;
cap_state cap_FSM_ex;
cap_control_t cap_control;
chassis_state_t chassis_state;
//新电容模式flag 0为关闭 1为开启
uint8_t cap_flag;
//新电容模式切换冷却时间
uint16_t cap_mode_change_cold_time;
//低压放电保护 0：电压低不可使用电容 1：可使用电容
uint8_t low_vol_flag;



void SuperCap_Task(void const * argument){

    vTaskDelay(100);

    fn_cap_init(&cap_control);

    while(1){

        fn_cap_Contorl(&cap_control);

        cap_FSM_ex = cap_FSM;	//状态机更新

        vTaskDelay(1);
    }
}



//电容初始化
void fn_cap_init(cap_control_t *cap_control_init)
{ 
    cap_num = 800;
    cap_en_flag = 0;

	cap_flag = 0;
	cap_mode_change_cold_time = 1000;
	low_vol_flag = 0;
}


void fn_cap_Contorl(cap_control_t *cap_control)
{
	get_chassis_power_and_buffer(&chassis_state.Chassis_power, &chassis_state.chassis_power_buffer);
	chassis_state.Chassis_power_limit=ext_robot_status.chassis_power_limit;
	chassis_state.remain_power=chassis_state.Chassis_power_limit - chassis_state.Chassis_power;

    if(SUPER_CAP_ON)
	{
		if(IF_RC_SW2_MID)  //右边拨杆处于中间
		{
			if(cap_mode_change_cold_time > 0){
				cap_mode_change_cold_time--;
			}
            if(IF_KEY_PRESSED_C && cap_mode_change_cold_time == 0 && cap_flag == 0){
				cap_flag = 1;
				cap_mode_change_cold_time = 1000;
			}
			//if(IF_KEY_PRESSED_F && cap_mode_change_cold_time == 0 && cap_flag == 1){
			//	cap_flag = 0;
			//	cap_mode_change_cold_time = 1000;
			//}
			if(low_vol_flag == 0 && cap_data.Capacity >= 10){
			  low_vol_flag = 1;
			}
			if(low_vol_flag == 1 && cap_data.Capacity < 6){
			  low_vol_flag = 0;
			}

			if(cap_flag == 1){
			    cap_FSM = cap_auto_charge;
			}
			else if(cap_flag == 0){
				if(cap_data.Capacity < 29){
				    cap_FSM = cap_dis_charge;//电容电量小于25v 充
			    }
			    else{//电容电量大于25000
				    cap_FSM = cap_dis_ucharge;//不放不充电
			    }
			}
		}
		
		else if(IF_RC_SW2_DOWN)//down2
		{
            cap_mode_change_cold_time = 1000;

			if(cap_data.Capacity < 29)
			{
				cap_FSM = cap_dis_charge;//电容电量小于25v 充
			}
			else//电容电量大于25000
			{
				cap_FSM = cap_dis_ucharge;//不放不充电
				CAN2_Cap_flag = 0;
			}
		}		
		
		else//读不到遥控器值？
		{
            cap_mode_change_cold_time = 1000;

			cap_FSM = cap_dis_ucharge;//不充不放
		}
	}
}
