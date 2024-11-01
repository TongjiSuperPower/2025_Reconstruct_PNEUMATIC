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

//�������Ƶ ��/S
float shoot_speed;
//���ģ������
shoot_data_t shoot_data;
//��תʱ��
uint16_t shoot_init_time;
//��ת���ʱ��
uint16_t shoot_init_over_time;
//����һ�������ʼ������ʱ��
uint16_t shoot_continue_time;
//����һ������жϼ�����
uint16_t shoot_single_time_count;
//��������жϼ�����
uint16_t shooting_single_count;
//�������Ա����л���ȴʱ��
uint16_t mode_time;
//�Ա�ģʽ�µ���ģʽ�л���ȴʱ��
uint16_t continue_shoot_mode_time;
//�Ա�ģʽ�µ���ģʽ
uint8_t continue_shoot_mode;
//�����嵯ģʽ�ļ�ʱ��Ħ���ִ���ҡ�˲�����1s
uint16_t clear_time;

void fn_ShootMode(void);

//�����ֳ�ʼ��
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

//Ħ���ֳ�ʼ��
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

//���ģ��״̬��ʼ��
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




//���ģ��ģʽѡ��
void fn_fric_state(void){
	if(IF_RC_SW1_MID && !IF_MOUSE_PRESSED_LEFT && !IF_RC_SW2_DOWN){
		shoot_data.fric_state_change_flag = 1;
	}
	//Ħ����״̬ѡ��
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
    //���ģʽѡ��
	fn_ShootMode();
}

//���ģʽѡ��
void fn_ShootMode(void){
	//���ڻ�ת�в�ѡ��ģʽ
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
	//����ת������ȴ����ʱ
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
	//���ģʽѡ��
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
	//��ת��תģʽ
	if(trigger_motor3508_block_flag){
		shoot_data.shoot_mode = SHOOT_INIT;
		shoot_data.trigger_back_flag = 1;
		trigger_motor3508_data[0].target_angle = fn_RadFormat(trigger_motor3508_data[0].relative_raw_angle + TriggerBackAngle);
	}
}


//����˫�� ��������
//���ģ���������
void fn_ShootMove(void){
	//������
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

	//����ģʽ
	if(shoot_data.shoot_mode == SHOOT_READY_SINGLE){
		//��ʱ��һ���������ʱ��
		if(shoot_data.shoot_over_flag == 0){
			shoot_continue_time++;
		}
		//�ж���һ������Ƿ����
		if(shoot_continue_time > 149){
			shoot_data.shoot_over_flag = 1;
			shoot_data.shoot_over_back_flag = 1;
			shoot_continue_time = 0;
		}
		//�ϴ���������ʼ������ȴ
		if(shoot_data.shoot_over_flag == 1){
			if(shoot_data.shoot_cold_time > 0){
			    shoot_data.shoot_cold_time--;
			}
		}
		//����ģʽ�� ��ҡ�˲����м����ɵ�������ͬʱ��ȴʱ��Ϊ0��������Ȩ��
		if(shoot_data.shoot_cold_time == 0 && IF_RC_SW1_MID && !IF_MOUSE_PRESSED_LEFT){  // && (ext_robot_status.shooter_barrel_heat_limit - ext_power_heat_data.shooter_42mm_barrel_heat) >= 100
		    shoot_data.shoot_permission_flag = 1;
		}
		//�ж��Ƿ��������Ҫ��
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
		//����ģʽ���ٶ���Ϊ0
		trigger_motor3508_data[0].target_speed = 0;

		//����ģʽ�²����ֵ�������
		trigger_motor3508_data[0].double_pid_mid = fn_PidClacAngle(&trigger_motor3508_data[0].motor_pid1,
		                                                           trigger_motor3508_data[0].relative_raw_angle,trigger_motor3508_data[0].target_angle);
		trigger_motor3508_data[0].given_current = fn_PidClac(&trigger_motor3508_data[0].motor_pid2,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].double_pid_mid);
	
	    //˫�����
		if(shoot_data.double_shoot_flag == 0 && shoot_data.shoot_over_flag == 0){
			shoot_data.double_shoot_flag = 1;
		}
		if(shoot_data.double_shoot_flag == 1){
			float speed = 0;
			speed = (fabs(gimbal_motor3508_data[0].relative_raw_speed) + fabs(gimbal_motor3508_data[1].relative_raw_speed)) / 2.0f;
			//�жϵ�һ��
			if(speed < (FricSpeed - FricSpeedReduce) && shooting_single_count < 10){
				shooting_single_count++;
			}
			//��һ���Ѿ�������
			if(shooting_single_count == 10 && speed > (FricSpeed - FricSpeedReduce + 5.0f)){
				shooting_single_count++;
			}
			//�жϵڶ���
			if(shooting_single_count >= 11 && speed < (FricSpeed - FricSpeedReduce) && shooting_single_count < 21){
				shooting_single_count++;
			}
		}
		//���¼�����
		if(shoot_data.double_shoot_flag == 1 && shoot_single_time_count < DetectTime){
			shoot_single_time_count++;
			
		}
		//��⵽˫������
		if(shooting_single_count == 21 && shoot_data.shoot_over_flag == 1){
			trigger_motor3508_block_flag = 1;
			shoot_single_time_count = 0;
			shoot_data.double_shoot_flag = 0;
			shooting_single_count = 0;
		}
		//��ⳬʱ����
		if(shooting_single_count != 21 && shoot_single_time_count == DetectTime){
			shoot_data.double_shoot_flag = 0;
			shoot_single_time_count = 0;
			shooting_single_count = 0;
		}
	}

	//�Ա�ģʽ
	if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
        //���
		if(IF_MOUSE_PRESSED_LEFT){
			trigger_motor3508_data[0].target_speed = 5.0f;
	    }
		//δ���
		if(!IF_MOUSE_PRESSED_LEFT){
			trigger_motor3508_data[0].target_speed = 0.0f;
		}

        //����ģʽ�½��Ƕ�Ŀ��ֵ��Զ��������ֵ����֤�л��ص���ģʽ��������
		trigger_motor3508_data[0].target_angle = trigger_motor3508_data[0].relative_raw_angle;

		//����ģʽ��������
		trigger_motor3508_data[0].given_current = fn_Iclear_PidClac(&trigger_motor3508_data[0].motor_pid3,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].target_speed);
	}

	if(shoot_data.shoot_mode == SHOOT_CLEAR){
		trigger_motor3508_data[0].target_speed = -10.0f;

		//�嵯ģʽ�½��Ƕ�Ŀ��ֵ��Զ��������ֵ����֤�л��ص���ģʽ��������
		trigger_motor3508_data[0].target_angle = trigger_motor3508_data[0].relative_raw_angle;

		//�嵯ģʽ��������
		trigger_motor3508_data[0].given_current = fn_PidClac(&trigger_motor3508_data[0].motor_pid3,
		                                                     trigger_motor3508_data[0].relative_raw_speed,trigger_motor3508_data[0].target_speed);
	}

	//�����ֳ�ʼ��ģʽ
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

	//Ħ����
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
		//Ħ���ֵ�������
        gimbal_motor3508_data[0].given_current = fn_PidClac(&gimbal_motor3508_data[0].motor_pid3,
		                                                     gimbal_motor3508_data[0].relative_raw_speed,gimbal_motor3508_data[0].target_speed);
		gimbal_motor3508_data[1].given_current = fn_PidClac(&gimbal_motor3508_data[1].motor_pid3,
		                                                     gimbal_motor3508_data[1].relative_raw_speed,gimbal_motor3508_data[1].target_speed);
		gimbal_motor3508_data[2].given_current = fn_PidClac(&gimbal_motor3508_data[2].motor_pid3,
		                                                     gimbal_motor3508_data[2].relative_raw_speed,gimbal_motor3508_data[2].target_speed);
	}
	
	if(shoot_data.fric_state == FRIC_ON){
		//����Ħ�����ٶ�
		//if(shoot_data.shoot_mode == SHOOT_READY_SINGLE || shoot_data.shoot_mode == SHOOT_CLEAR){
		//    gimbal_motor3508_data[0].target_speed = shoot_data.fric_speed;
		//    gimbal_motor3508_data[1].target_speed = -shoot_data.fric_speed;
		//}
		//else if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
		//	gimbal_motor3508_data[0].target_speed = shoot_data.fric_speed - 50.0f;
		//    gimbal_motor3508_data[1].target_speed = -shoot_data.fric_speed - 50.0f;
		//}

		//�������Զ�����Ħ����ת��
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
		
		
		//Ħ���ֵ�������
        gimbal_motor3508_data[0].given_current = fn_PidClac(&gimbal_motor3508_data[0].motor_pid3,
		                                                     gimbal_motor3508_data[0].relative_raw_speed,gimbal_motor3508_data[0].target_speed);
		gimbal_motor3508_data[1].given_current = fn_PidClac(&gimbal_motor3508_data[1].motor_pid3,
		                                                     gimbal_motor3508_data[1].relative_raw_speed,gimbal_motor3508_data[1].target_speed);
	    gimbal_motor3508_data[2].given_current = fn_PidClac(&gimbal_motor3508_data[2].motor_pid3,
		                                                     gimbal_motor3508_data[2].relative_raw_speed,gimbal_motor3508_data[2].target_speed);
	}
}
