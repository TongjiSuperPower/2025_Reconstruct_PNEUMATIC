#include "shoot_task.h"
#include "struct_typedef.h"
#include "can_task.h"
#include "pid.h"
#include "math_lib.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "stdlib.h"
#include "math.h"
#include "referee_task.h"

//射击的射频 发/S
float shoot_speed;
//射击模块数据
shoot_data_t shoot_data;
//倒转时间
uint16_t shoot_init_time;
//倒转完成时间
uint16_t shoot_init_over_time;
//单发一次射击开始持续的时间
uint16_t shoot_continue_time;
//单发一次射击判断计数器
uint16_t shoot_single_time_count;
//单发射击判断计数器
uint16_t shooting_single_count;
//单发与自爆的切换冷却时间
uint16_t mode_time;
//自爆模式下的子模式切换冷却时间
uint16_t continue_shoot_mode_time;
//自爆模式下的子模式
uint8_t continue_shoot_mode;
//进入清弹模式的计时，摩擦轮打开左摇杆拨到下1s
uint16_t clear_time;

void fn_ShootMode(void);

//拨弹轮初始化
void fn_ShootMotorInit(void){
	fp32 af_TriggerMotor3508PosPid1[3] = {TriggerMotor3508PosPid1_ID205_kp,TriggerMotor3508PosPid1_ID205_ki,TriggerMotor3508PosPid1_ID205_kd};
	fp32 af_TriggerMotor3508PosPid2[3] = {TriggerMotor3508PosPid2_ID205_kp,TriggerMotor3508PosPid2_ID205_ki,TriggerMotor3508PosPid2_ID205_kd};
    fp32 af_TriggerMotor3508PosPid3[3] = {TriggerMotor3508PosPid3_ID205_kp,TriggerMotor3508PosPid3_ID205_ki,TriggerMotor3508PosPid3_ID205_kd};
    
    trigger_motor3508_data[0].round_num = 0;
    trigger_motor3508_data[0].relative_raw_angle = 0.0f;

	for(uint8_t m = 0;m < 6;m++){
        trigger_motor3508_data[0].raw_angle[m] = 0.0f;
		trigger_motor3508_data[0].raw_speed[m] = 0.0f;
	}
	for(uint8_t n = 0;n < 2;n++){
		trigger_motor3508_data[0].filter_angle[n] = 0.0f;
		trigger_motor3508_data[0].filter_speed[n] = 0.0f;
	}

    trigger_motor3508_data[0].relative_raw_speed = 0.0f;
    trigger_motor3508_data[0].offecd_ecd = trigger_motor3508_measure[0].ecd;
	trigger_motor3508_data[0].target_angle = 0.0f;
	trigger_motor3508_data[0].target_speed = 0.0f;
	trigger_motor3508_data[0].filter_given_current = 0.0f;
    trigger_motor3508_data[0].given_current = 0.0f;
    trigger_motor3508_data[0].double_pid_mid = 0.0f;

	fn_PidInit(&trigger_motor3508_data[0].motor_pid1,af_TriggerMotor3508PosPid1,TriggerMotor3508SpeedMinOut,TriggerMotor3508SpeedMaxOut,TriggerMotor3508SpeedMinIOut,TriggerMotor3508SpeedMaxIOut);
	fn_PidInit(&trigger_motor3508_data[0].motor_pid2,af_TriggerMotor3508PosPid2,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
    fn_PidInit(&trigger_motor3508_data[0].motor_pid3,af_TriggerMotor3508PosPid3,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
}

//摩擦轮初始化
void fn_shoot_motor3508_init(void){
	fp32 af_GimbalMotor3508PosPid3[3][3] = {{GimbalMotor3508PosPid3_ID201_kp,GimbalMotor3508PosPid3_ID201_ki,GimbalMotor3508PosPid3_ID201_kd},
	                                        {GimbalMotor3508PosPid3_ID202_kp,GimbalMotor3508PosPid3_ID202_ki,GimbalMotor3508PosPid3_ID202_kd},
											{GimbalMotor3508PosPid3_ID203_kp,GimbalMotor3508PosPid3_ID203_ki,GimbalMotor3508PosPid3_ID203_kd}};

	for(uint8_t i = 0;i < 3;i++){

        gimbal_motor3508_data[i].round_num = 0;       
        gimbal_motor3508_data[i].relative_raw_angle = 0.0f;

	    for(uint8_t m = 0;m < 5;m++){
            gimbal_motor3508_data[i].raw_angle[m] = 0.0f;
		    gimbal_motor3508_data[i].raw_speed[m] = 0.0f; 
	    }
	    for(uint8_t n = 0;n < 2;n++){
		    gimbal_motor3508_data[i].filter_angle[n] = 0.0f;
		    gimbal_motor3508_data[i].filter_speed[n] = 0.0f;
	    }

        gimbal_motor3508_data[i].relative_raw_speed = 0.0f;                 
        gimbal_motor3508_data[i].offecd_ecd = 0;   
	    gimbal_motor3508_data[i].target_angle = 0.0f;
		gimbal_motor3508_data[i].target_speed = 0.0f;    
        gimbal_motor3508_data[i].given_current = 0.0f;           
        gimbal_motor3508_data[i].double_pid_mid = 0.0f;
		
	    fn_PidInit(&gimbal_motor3508_data[i].motor_pid3,af_GimbalMotor3508PosPid3[i],GimbalMotor3508MinOut,GimbalMotor3508MaxOut,GimbalMotor3508MinIOut,GimbalMotor3508MaxIOut);

	}
}

//射击模块状态初始化
void fn_shoot_init(void){
    shoot_init_time = 0;
    shoot_init_over_time = 0;
	shoot_continue_time = 0;
	shoot_single_time_count = 0;
	shooting_single_count = 0;
	mode_time = ShootModeTime;
	
	continue_shoot_mode_time = 1000;
	continue_shoot_mode = 0;

	shoot_data.fric_state = FRIC_OFF;
	shoot_data.shoot_mode = SHOOT_DOWN;

    shoot_data.fric_speed = FricSpeed;
	shoot_data.shoot_cold_time = ShootColdTime;

	shoot_data.infact_shoot_speed = 0.0f;
	shoot_data.last_infact_shoot_speed = 0.0f;

	clear_time = 1000;

	shoot_data.shoot_over_back_flag = 0;
	shoot_data.first_shoot_flag = 1;
	shoot_data.trigger_back_flag = 0;
	shoot_data.shoot_permission_flag = 1;
	shoot_data.fric_state_change_flag = 0;
	shoot_data.shoot_over_flag = 1;
	shoot_data.shoot_single_or_countinue_flag = 1;
	shoot_data.double_shoot_flag = 0;
}




//射击模块模式选择
void fn_fric_state(void){
	if(IF_RC_SW1_MID && !IF_MOUSE_PRESSED_LEFT && !IF_RC_SW2_DOWN){
		shoot_data.fric_state_change_flag = 1;
	}
	//摩擦轮状态选择
	if(IF_RC_SW2_DOWN){
		shoot_data.fric_state = FRIC_DOWN;
	}
	if((shoot_data.fric_state == FRIC_OFF || shoot_data.fric_state == FRIC_DOWN) && (IF_RC_SW1_UP && shoot_data.fric_state_change_flag == 1) && !IF_RC_SW2_DOWN){
        shoot_data.fric_state = FRIC_ON;
		shoot_data.fric_state_change_flag = 0;
	}
	if(IF_RC_SW1_UP && shoot_data.fric_state == FRIC_ON && shoot_data.fric_state_change_flag == 1 && !IF_RC_SW2_DOWN){
        shoot_data.fric_state = FRIC_OFF;
		shoot_data.fric_state_change_flag = 0;
	}
    //射击模式选择
	fn_ShootMode();
}

//射击模式选择
void fn_ShootMode(void){
	//正在回转中不选择模式
	if(shoot_data.trigger_back_flag){
		return;
	}
	if(IF_RC_SW1_MID){
		clear_time = 1000;
	}
	if(shoot_data.fric_state == FRIC_ON && IF_RC_SW2_UP && IF_RC_SW1_DOWN && clear_time > 0){
		clear_time--;
	}
	if(IF_RC_SW2_UP && clear_time != 0){
		shoot_data.shoot_single_or_countinue_flag = 0;
	}
	if(IF_RC_SW2_UP && clear_time == 0){
		shoot_data.shoot_single_or_countinue_flag = 2;
	}
	//单发转连发冷却倒计时
	if(mode_time > 0){
		mode_time--;
	}
	if(IF_RC_SW2_MID && mode_time == 0 && IF_KEY_PRESSED_Z && shoot_data.shoot_single_or_countinue_flag == 1){
		shoot_data.shoot_single_or_countinue_flag = 0;
		mode_time = ShootModeTime;
	}
	if(IF_RC_SW2_MID && mode_time == 0 && IF_KEY_PRESSED_Z && shoot_data.shoot_single_or_countinue_flag == 0){
		shoot_data.shoot_single_or_countinue_flag = 1;
		mode_time = ShootModeTime;
	}
	//射击模式选择
	if(shoot_data.fric_state == FRIC_OFF || shoot_data.fric_state == FRIC_DOWN){
		shoot_data.shoot_mode = SHOOT_DOWN;
	}
	if(shoot_data.fric_state == FRIC_ON && shoot_data.shoot_single_or_countinue_flag == 0){
		shoot_data.shoot_mode = SHOOT_READY_SINGLE;
	}
	if(shoot_data.fric_state == FRIC_ON && shoot_data.shoot_single_or_countinue_flag == 1){
		shoot_data.shoot_mode = SHOOT_READY_COUNTINUE;
	}
	if(shoot_data.fric_state == FRIC_ON && shoot_data.shoot_single_or_countinue_flag == 2){
		shoot_data.shoot_mode = SHOOT_CLEAR;
	}
	//堵转反转模式
	if(trigger_motor3508_block_flag){
		shoot_data.shoot_mode = SHOOT_INIT;
		shoot_data.trigger_back_flag = 1;
		trigger_motor3508_data[0].target_angle = fn_RadFormat(trigger_motor3508_data[0].relative_raw_angle + TriggerBackAngle);
	}
}


//单发双环 连发单环
//射击模块电流解算
void fn_ShootMove(void){
	//拨弹轮
	//down
    if(shoot_data.shoot_mode == SHOOT_DOWN){
		shoot_continue_time = 0;
	    mode_time = ShootModeTime;
		shoot_data.shoot_cold_time = ShootColdTime;
		shoot_data.shoot_over_flag = 1;
		shoot_data.shoot_permission_flag = 0;

		trigger_motor3508_data[0].target_angle = trigger_motor3508_data[0].relative_raw_angle;
		trigger_motor3508_data[0].target_speed = 0;

		trigger_motor3508_data[0].given_current = 0;
	}

	//单发模式
	if(shoot_data.shoot_mode == SHOOT_READY_SINGLE){
		//计时上一次射击持续时间
		if(shoot_data.shoot_over_flag == 0){
			shoot_continue_time++;
		}
		//判断上一次射击是否完成
		if(shoot_continue_time > 149){
			shoot_data.shoot_over_flag = 1;
			shoot_data.shoot_over_back_flag = 1;
			shoot_continue_time = 0;
		}
		//上次射击完成则开始减少冷却
		if(shoot_data.shoot_over_flag == 1){
			if(shoot_data.shoot_cold_time > 0){
			    shoot_data.shoot_cold_time--;
			}
		}
		//单发模式下 左摇杆拨到中间且松掉鼠标左键同时冷却时间为0则给予射击权限
		if(shoot_data.shoot_cold_time == 0 && IF_RC_SW1_MID && !IF_MOUSE_PRESSED_LEFT){  // && (ext_robot_status.shooter_barrel_heat_limit - ext_power_heat_data.shooter_42mm_barrel_heat) >= 100
		    shoot_data.shoot_permission_flag = 1;
		}
		//判断是否满足射击要求
		if((IF_RC_SW1_DOWN || IF_MOUSE_PRESSED_LEFT) && shoot_data.shoot_permission_flag == 1){
			trigger_motor3508_data[0].target_angle = fn_RadFormat(trigger_motor3508_data[0].target_angle - ShootAngleAdd - ShootAngleAdd_addition);
			//shoot_continue_time = 0;
			shoot_data.shoot_permission_flag = 0;
			shoot_data.shoot_over_flag = 0;
			shoot_data.shoot_cold_time = ShootColdTime;
		}
		if(shoot_data.shoot_over_back_flag){
			trigger_motor3508_data[0].target_angle = fn_RadFormat(trigger_motor3508_data[0].target_angle + ShootAngleAdd_addition);
			shoot_data.shoot_over_back_flag = 0;
		}
		//单发模式下速度设为0
		trigger_motor3508_data[0].target_speed = 0;

		//单发模式下拨弹轮电流计算
		trigger_motor3508_data[0].double_pid_mid = fn_PidClacAngle(&trigger_motor3508_data[0].motor_pid1,
		                                                           trigger_motor3508_data[0].relative_raw_angle,trigger_motor3508_data[0].target_angle);
		trigger_motor3508_data[0].given_current = fn_PidClac(&trigger_motor3508_data[0].motor_pid2,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].double_pid_mid);
	
	    //双发检测
		if(shoot_data.double_shoot_flag == 0 && shoot_data.shoot_over_flag == 0){
			shoot_data.double_shoot_flag = 1;
		}
		if(shoot_data.double_shoot_flag == 1){
			float speed = 0;
			speed = (fabs(gimbal_motor3508_data[0].relative_raw_speed) + fabs(gimbal_motor3508_data[1].relative_raw_speed)) / 2.0f;
			//判断第一发
			if(speed < (FricSpeed - FricSpeedReduce) && shooting_single_count < 10){
				shooting_single_count++;
			}
			//第一发已经射击完成
			if(shooting_single_count == 10 && speed > (FricSpeed - FricSpeedReduce + 5.0f)){
				shooting_single_count++;
			}
			//判断第二发
			if(shooting_single_count >= 11 && speed < (FricSpeed - FricSpeedReduce) && shooting_single_count < 21){
				shooting_single_count++;
			}
		}
		//更新计数器
		if(shoot_data.double_shoot_flag == 1 && shoot_single_time_count < DetectTime){
			shoot_single_time_count++;
			
		}
		//检测到双发结束
		if(shooting_single_count == 21 && shoot_data.shoot_over_flag == 1){
			trigger_motor3508_block_flag = 1;
			shoot_single_time_count = 0;
			shoot_data.double_shoot_flag = 0;
			shooting_single_count = 0;
		}
		//检测超时结束
		if(shooting_single_count != 21 && shoot_single_time_count == DetectTime){
			shoot_data.double_shoot_flag = 0;
			shoot_single_time_count = 0;
			shooting_single_count = 0;
		}
	}

	//自爆模式
	if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
        //射击
		if(IF_MOUSE_PRESSED_LEFT){
			trigger_motor3508_data[0].target_speed = 5.0f;
	    }
		//未射击
		if(!IF_MOUSE_PRESSED_LEFT){
			trigger_motor3508_data[0].target_speed = 0.0f;
		}

        //连发模式下将角度目标值永远等于现在值，保证切换回单发模式的连续性
		trigger_motor3508_data[0].target_angle = trigger_motor3508_data[0].relative_raw_angle;

		//连发模式电流计算
		trigger_motor3508_data[0].given_current = fn_Iclear_PidClac(&trigger_motor3508_data[0].motor_pid3,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].target_speed);
	}

	if(shoot_data.shoot_mode == SHOOT_CLEAR){
		trigger_motor3508_data[0].target_speed = -10.0f;

		//清弹模式下将角度目标值永远等于现在值，保证切换回单发模式的连续性
		trigger_motor3508_data[0].target_angle = trigger_motor3508_data[0].relative_raw_angle;

		//清弹模式电流计算
		trigger_motor3508_data[0].given_current = fn_PidClac(&trigger_motor3508_data[0].motor_pid3,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].target_speed);
	}

	//拨弹轮初始化模式
	if(shoot_data.shoot_mode == SHOOT_INIT){

        if(fabs(trigger_motor3508_data[0].target_angle - trigger_motor3508_data[0].relative_raw_angle) < 0.05f){
			shoot_init_over_time++;
		}

        shoot_init_time++;

		if(shoot_init_time > 500 || shoot_init_over_time > 50){
			trigger_motor3508_data[0].target_angle = trigger_motor3508_data[0].relative_raw_angle;
            shoot_init_over_time = 0;
			shoot_init_time = 0;
			shoot_data.trigger_back_flag = 0;
			trigger_motor3508_block_flag = 0;
		}

		trigger_motor3508_data[0].target_speed = 0.0f;

		trigger_motor3508_data[0].double_pid_mid = fn_Iclear_PidClacAngle(&trigger_motor3508_data[0].motor_pid1,
		                                                           trigger_motor3508_data[0].relative_raw_angle,trigger_motor3508_data[0].target_angle);
		trigger_motor3508_data[0].given_current = fn_PidClac(&trigger_motor3508_data[0].motor_pid2,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].double_pid_mid);
	}

	//摩擦轮
	if(shoot_data.fric_state == FRIC_DOWN){
		gimbal_motor3508_data[0].target_speed = 0.0f;
		gimbal_motor3508_data[1].target_speed = 0.0f;
		gimbal_motor3508_data[2].target_speed = 0.0f;
        gimbal_motor3508_data[0].given_current = 0;
		gimbal_motor3508_data[1].given_current = 0;
		gimbal_motor3508_data[2].given_current = 0;
	}

	if(shoot_data.fric_state == FRIC_OFF){
		gimbal_motor3508_data[0].target_speed = 0.0f;
		gimbal_motor3508_data[1].target_speed = 0.0f;
		gimbal_motor3508_data[2].target_speed = 0.0f;
		//摩擦轮电流计算
        gimbal_motor3508_data[0].given_current = fn_PidClac(&gimbal_motor3508_data[0].motor_pid3,
		                                                     gimbal_motor3508_data[0].relative_raw_speed,gimbal_motor3508_data[0].target_speed);
		gimbal_motor3508_data[1].given_current = fn_PidClac(&gimbal_motor3508_data[1].motor_pid3,
		                                                     gimbal_motor3508_data[1].relative_raw_speed,gimbal_motor3508_data[1].target_speed);
		gimbal_motor3508_data[2].given_current = fn_PidClac(&gimbal_motor3508_data[2].motor_pid3,
		                                                     gimbal_motor3508_data[2].relative_raw_speed,gimbal_motor3508_data[2].target_speed);
	}
	
	if(shoot_data.fric_state == FRIC_ON){
		//赋予摩擦轮速度
		//if(shoot_data.shoot_mode == SHOOT_READY_SINGLE || shoot_data.shoot_mode == SHOOT_CLEAR){
		//    gimbal_motor3508_data[0].target_speed = shoot_data.fric_speed;
		//    gimbal_motor3508_data[1].target_speed = -shoot_data.fric_speed;
		//}
		//else if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
		//	gimbal_motor3508_data[0].target_speed = shoot_data.fric_speed - 50.0f;
		//    gimbal_motor3508_data[1].target_speed = -shoot_data.fric_speed - 50.0f;
		//}

		//超射速自动减少摩擦轮转速
		shoot_data.last_infact_shoot_speed = shoot_data.infact_shoot_speed;
		get_shoot_speed(&shoot_data.infact_shoot_speed);
		if(shoot_data.infact_shoot_speed != shoot_data.last_infact_shoot_speed && shoot_data.infact_shoot_speed > 29.4f && IF_MOUSE_PRESSED_LEFT){
            shoot_data.fric_speed -= 40.0f;
		}
		else if(shoot_data.last_infact_shoot_speed == shoot_data.infact_shoot_speed && !IF_MOUSE_PRESSED_LEFT){
			shoot_data.fric_speed = FricSpeed;
		}
		fn_Fp32Limit(&shoot_data.fric_speed,200.0f,1000.0f);

		gimbal_motor3508_data[0].target_speed = -shoot_data.fric_speed;
		gimbal_motor3508_data[1].target_speed = shoot_data.fric_speed;
		gimbal_motor3508_data[2].target_speed = -shoot_data.fric_speed;
		
		
		//摩擦轮电流计算
        gimbal_motor3508_data[0].given_current = fn_PidClac(&gimbal_motor3508_data[0].motor_pid3,
		                                                     gimbal_motor3508_data[0].relative_raw_speed,gimbal_motor3508_data[0].target_speed);
		gimbal_motor3508_data[1].given_current = fn_PidClac(&gimbal_motor3508_data[1].motor_pid3,
		                                                     gimbal_motor3508_data[1].relative_raw_speed,gimbal_motor3508_data[1].target_speed);
	    gimbal_motor3508_data[2].given_current = fn_PidClac(&gimbal_motor3508_data[2].motor_pid3,
		                                                     gimbal_motor3508_data[2].relative_raw_speed,gimbal_motor3508_data[2].target_speed);
	}
}
