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
//���ģ��ʵ����
shoot_control_t shoot_control;
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
//����ŵ�״̬��ʱ����,����Ϊȫ�ֱ�������debug
GPIO_PinState photogate_state;
//��ȴʱ�䣿
int32_t shoot_cooling_time = 0;
//��ǰǹ������
uint16_t cooling_heat;
//ǹ����������
uint16_t cooling_heat_limit;

//���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1��
void fn_Shoot_set_mode(void);

//������ݸ���,���²������ٶȺͼ���
void fn_Shoot_feedback_update(void);

//����״̬�����㱻����
void fn_Shoot_action(void);

//��ת��ת����
void fn_triggermotor_turn_back(void);

//������ƣ����Ʋ�������Ƕȣ����һ�η���
void fn_shoot_bullet_control(void);

//f103��������
void f103_control_loop(void);


/**
 * @brief          ���ѭ��(������������������)
 * @param[in]      void
 * @retval         ����can����ֵ
 */
void fn_shoot_control_loop(void)
{
	fn_Shoot_set_mode();	//����״̬��
	fn_Shoot_feedback_update();	//���²������ٶȺͼ���
	fn_Shoot_action();
	f103_control_loop();
	fn_cmd_F103(shoot_control.cylinder_cmd, shoot_control.quival_cmd, 0, 500);
	
}



void f103_control_loop(void)
{
	if(shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.cylinder_cmd = 0;
		shoot_control.quival_cmd = 0;
		return;
	}

	if(shoot_control.cylinder_mode == CYLIN_OPEN)
	{
		shoot_control.cylinder_cmd = 0;
	}
	else
	{
		shoot_control.cylinder_cmd = 1;
	}

	if(shoot_control.quickvalve_mode == QUIVAL_OPEN)
	{
		shoot_control.quival_cmd = 1;
	}
	else
	{
		shoot_control.quival_cmd = 0;
	}
}


//�����ֳ�ʼ��
void fn_ShootMotorInit(void)
{	
	fp32 af_TriggerMotor3508_SpeedPid[3] = {TriggerMotor3508_SpeedPid_ID207_kp ,TriggerMotor3508_SpeedPid_ID207_ki ,TriggerMotor3508_SpeedPid_ID207_kd};
	fp32 af_TriggerMotor3508_AnglePid[3] = {TriggerMotor3508_AnglePid_ID207_kp ,TriggerMotor3508_AnglePid_ID207_ki ,TriggerMotor3508_AnglePid_ID207_kd};
	fp32 af_TriggerMotor3508_ActionSpeedPid[3] = {TriggerMotor3508_ActionSpeedPid_ID207_kp ,TriggerMotor3508_ActionSpeedPid_ID207_ki ,TriggerMotor3508_ActionSpeedPid_ID207_kd};
	fp32 af_TriggerMotor3508_ActionAnglePid[3] = {TriggerMotor3508_ActionAnglePid_ID207_kp ,TriggerMotor3508_ActionAnglePid_ID207_ki ,TriggerMotor3508_ActionAnglePid_ID207_kd};
	

    trigger_motor3508_data[0].round_num = 0;
	trigger_motor3508_data[0].offecd_ecd = trigger_motor3508_measure[0].ecd;
	/*������ʱδʹ�õ�������ʼ��*/
    trigger_motor3508_data[0].relative_raw_angle = 0.0f;

	for(uint8_t m = 0;m < 6;m++){
        trigger_motor3508_data[0].raw_angle[m] = 0.0f;
		trigger_motor3508_data[0].raw_speed[m] = 0.0f;
	}
	for(uint8_t n = 0;n < 2;n++){
		trigger_motor3508_data[0].filter_angle[n] = 0.0f;
		trigger_motor3508_data[0].filter_speed[n] = 0.0f;
	}
	/*������ʱδ�õ�������ʼ��*/
    trigger_motor3508_data[0].relative_raw_speed = 0.0f;
	trigger_motor3508_data[0].target_angle = 0.0f;
	trigger_motor3508_data[0].target_speed = 0.0f;
	trigger_motor3508_data[0].filter_given_current = 0.0f;
    trigger_motor3508_data[0].given_current = 0.0f;
    trigger_motor3508_data[0].double_pid_mid = 0.0f;

	/*��ʼ��pid*/
	fn_PidInit(&shoot_control.trigger_speed_pid,af_TriggerMotor3508_SpeedPid,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	fn_PidInit(&shoot_control.trigger_angle_pid,af_TriggerMotor3508_AnglePid,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	fn_PidInit(&shoot_control.trigger_action_speed_pid,af_TriggerMotor3508_ActionSpeedPid,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	fn_PidInit(&shoot_control.trigger_action_angle_pid,af_TriggerMotor3508_ActionAnglePid,-4.0 ,4.0,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	
}


void fn_Shoot_set_mode(void)
{
	static int8_t last_s = RC_SW_UP; //��һʱ�̵��󲦸�ֵ
	shoot_control.v_key = ctl.key.v & AUTOAIM_SHOOT_KEY;
	photogate_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	shoot_control.photogate = (photogate_state == GPIO_PIN_SET);
	
	//�Ҳ��˲����µ���down
	if(IF_RC_SW2_DOWN)
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	//����������downģʽ
	else
	{
		if(IF_RC_SW2_UP)//ң����ģʽ
		{
			//�󲦸��е���ģʽ�л�
			if(IF_RC_SW1_UP && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP)
			{
				shoot_control.shoot_mode = SHOOT_READY;
			}
			else if(IF_RC_SW1_UP && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP)
			{
				shoot_control.shoot_mode = SHOOT_STOP;
			}
		}
		else if(IF_RC_SW2_MID)//����ģʽ
		{
			shoot_control.shoot_mode = SHOOT_READY;
		}
		//�����̨״̬�� ����״̬���͹ر����
		if(gimbal_data.gimbal_behaviour == GIMBAL_ZERO_FORCE || gimbal_data.gimbal_behaviour == GIMBAL_INIT)
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		if((shoot_control.press_l && shoot_control.last_press_l == 0 ) ||
		(autoaim_measure.shoot == 1 && autoaim_measure.vision_state != 0 && shoot_control.press_r && shoot_control.v_key))
		{
			shoot_control.shoot_enable_flag = 1;
		}//�����������Ҽ������ͬʱ��V����
		shoot_cooling_time++;

		// ��READY�����Ҳ���ͣת���ױպϵ����������²��󲦸ˣ��������ģʽ
		if(((IF_RC_SW1_DOWN && !switch_is_down(last_s) && IF_RC_SW2_UP) ||(shoot_control.shoot_enable_flag && IF_RC_SW2_MID))
		&& shoot_control.shoot_mode == SHOOT_READY && shoot_control.cylinder_mode == CYLIN_CLOSE && shoot_control.trigger_mode == TRIGGER_DOWN)
		{
			cooling_heat = ext_power_heat_data.shooter_42mm_barrel_heat;	//��ǰǹ������
			cooling_heat_limit = ext_robot_status.shooter_barrel_heat_limit;	//ǹ����������

			autoaim_measure.shoot = 0;
			if(cooling_heat_limit - cooling_heat < 100.0)
			{
				return;
			}
			if(shoot_cooling_time<=8000)
			{
				autoaim_measure.shoot = 0;
				return;
			}
			shoot_control.shoot_mode = SHOOT_BULLET;
		}

		// 1.�������READYģʽ
		if(shoot_control.shoot_mode == SHOOT_READY)
		{
			// ��������δ��⵽�źţ�����벦��ACTION�������̲���
			if(shoot_control.photogate != 1 && shoot_control.cylinder_mode == CYLIN_OPEN)
			{
				shoot_control.trigger_mode = TRIGGER_ACTION;

				if(fabs(shoot_control.trigger_speed) <= BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME) //�ж϶�תʱ��
				{
					shoot_control.block_time++;
					shoot_control.reverse_time = 0;	// û��ȷ���Ƕ�ת���򲻷�����ת����תʱ������
				}
				if(shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME) //ȷ���Ƕ�ת����ʼ��¼��תʱ��
				{
					shoot_control.reverse_time++;
					shoot_control.trigger_mode = TRIGGER_REVERSE;
				}
				if(shoot_control.reverse_time >= REVERSE_TIME && shoot_control.trigger_mode == TRIGGER_REVERSE)
				{
					shoot_control.block_time = 0;
					shoot_control.reverse_time = 0;
					shoot_control.trigger_mode = TRIGGER_ACTION;
				}
			}
			else if(shoot_control.photogate == 1 && shoot_control.last_photogate != 1)
			{
				shoot_control.trigger_mode =TRIGGER_ACTION_PLUS; //�����ֲ������	
			}
			
			if(shoot_control.trigger_mode == TRIGGER_ACTION_PLUS)
			{
				shoot_control.shoot_action_plus_count++;
				if(shoot_control.shoot_action_plus_count >= 250)//��⵽������źź���תһ��
				{
					shoot_control.shoot_action_plus_count = 0;
					shoot_control.trigger_mode = TRIGGER_DOWN;
				}
			}

			//�������ͣתͬʱ����ż�⵽���壬�жϸ׵�ģʽ
			if(shoot_control.trigger_mode == TRIGGER_DOWN && shoot_control.photogate == 1)
			{
				//������Ѿ��رգ���������
				if(shoot_control.cylinder_mode == CYLIN_CLOSE)
				{}
				//������Ǵ�״̬����ʱ500ms������ACTION
				if(shoot_control.cylinder_mode == CYLIN_OPEN)
				{
					shoot_control.cylin_open2action_count++;
				}
				if(shoot_control.cylin_open2action_count >= 500)
				{
					shoot_control.cylinder_mode = CYLIN_ACTION;
					shoot_control.cylin_open2action_count = 0;
				}


			}
			// ��������ǹرս����У���ʱ500ms������close
			if(shoot_control.trigger_mode == TRIGGER_DOWN)
			{
				if (shoot_control.cylinder_mode == CYLIN_ACTION)
				{
					shoot_control.cylin_action2close_count++;
				}
				if (shoot_control.cylin_action2close_count >= 500)
				{
					shoot_control.cylinder_mode = CYLIN_CLOSE;
					shoot_control.cylin_action2close_count = 0;
				}
			}
			// READY״̬�������ŷ�
			shoot_control.quickvalve_mode = QUIVAL_CLOSE;
		}

		// 2.���������BULLETģʽ����򿪿��ŷ�
		if(shoot_control.shoot_mode == SHOOT_BULLET)
		{
			shoot_control.quival_open2close_count++;
			shoot_control.quickvalve_mode = QUIVAL_OPEN;
			if(shoot_control.quival_open2close_count >= 50)
			{
				shoot_control.quickvalve_mode = QUIVAL_CLOSE;
				shoot_control.cylin_close2open_count++;
				if (shoot_control.cylin_close2open_count >= 500)
				{
					shoot_control.shoot_mode = SHOOT_STOP;
					shoot_cooling_time = 0;
					shoot_control.cylin_close2open_count = 0;
					shoot_control.quival_open2close_count = 0;
				}
				
			}
		}
	}

	// 3.���������STOPģʽ����ֹͣ����ת����ͬʱ���ŷ��رգ��״�
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.trigger_mode = TRIGGER_DOWN;
		shoot_control.cylinder_mode = CYLIN_OPEN;
		shoot_control.quickvalve_mode = QUIVAL_CLOSE;
		shoot_control.block_time = 0;
		shoot_control.reverse_time = 0;
		shoot_control.shoot_enable_flag = 0;
	}

	if (shoot_control.trigger_mode == TRIGGER_DOWN)
	{
		shoot_control.move_flag = 0;
		shoot_control.block_time = 0;
		shoot_control.reverse_time = 0;
	}	

	last_s = ctl.rc.s1;	//��һʱ�̵��󲦸�ֵ
	shoot_control.last_photogate = shoot_control.photogate;
	shoot_control.last_v_key = shoot_control.v_key;

}


void fn_Shoot_action(void)
{
	if(shoot_control.shoot_mode == SHOOT_STOP)// stopģʽ����gimbal����ĳЩ���⣬������ң����down״̬��У׼�׶εȣ���Ҫȫ��ֹͣ���ϵͳ
	{
		shoot_control.given_current = 0;
		return;
	}
	else if (shoot_control.trigger_mode == TRIGGER_DOWN)
	{
		if(shoot_control.angle_change_flag == 0)
		{
			shoot_control.trigger_angle_set = trigger_motor3508_data[0].relative_angle_19laps - PI_THREE * 0.65f;
			shoot_control.angle_change_flag = 1;
		}
		shoot_control.trigger_speed_set = fn_delta_PidClac(&shoot_control.trigger_angle_pid, fn_RadFormat(trigger_motor3508_data[0].relative_angle_19laps), fn_RadFormat(shoot_control.trigger_angle_set), shoot_control.trigger_speed);
		shoot_control.given_current = fn_PidClac(&shoot_control.trigger_speed_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
		return;
	}
	else if(shoot_control.trigger_mode == TRIGGER_REVERSE)
	{
		if(shoot_control.reverse_angle_change_flag == 0)
		{
			shoot_control.trigger_angle_set = trigger_motor3508_data[0].relative_angle_19laps - PI_THREE * 0.35f;
			shoot_control.reverse_angle_change_flag = 1;
		}
		shoot_control.trigger_speed_set = fn_delta_PidClac(&shoot_control.trigger_angle_pid, fn_RadFormat(trigger_motor3508_data[0].relative_angle_19laps), fn_RadFormat(shoot_control.trigger_angle_set), shoot_control.trigger_speed);
		shoot_control.given_current = fn_PidClac(&shoot_control.trigger_speed_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
		shoot_control.move_flag = 0;
		return;
	}
	else if(shoot_control.trigger_mode == TRIGGER_ACTION || shoot_control.trigger_mode == TRIGGER_ACTION_PLUS)
	{
		shoot_control.trigger_angle_pid.f_MaxOut = 4.0f;	//������
		shoot_control.trigger_angle_pid.f_MaxIout = 0.5;
		fn_shoot_bullet_control();
		shoot_control.trigger_speed_set = fn_delta_PidClac(&shoot_control.trigger_action_angle_pid, fn_RadFormat(trigger_motor3508_data[0].relative_angle_19laps), fn_RadFormat(shoot_control.trigger_angle_set), shoot_control.trigger_speed);
		shoot_control.given_current = fn_PidClac(&shoot_control.trigger_action_speed_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
		shoot_control.angle_change_flag = 0;
		shoot_control.reverse_angle_change_flag = 0;
	}

}


void fn_shoot_bullet_control(void)
{
	// ÿ�β������� 1/3PI �ĽǶȣ�ʹ����һ���ܵ���Ŀ��ֵ
	if(shoot_control.move_flag == 0)	// �����ʾ���Ըı�Ŀ��Ƕȣ�˵���ѽ����downģʽ������flag���㣬���Կ�ʼ��һ�β���
	{
		shoot_control.trigger_angle_set = fn_RadFormat(trigger_motor3508_data[0].relative_angle_19laps) + PI_THREE * 8.0f / 3.0f;
		shoot_control.move_flag = 1;	// �ı�Ŀ��ǶȺ󼴸ı�flag����ֹ�ڵ�ǰ�ֲ���������Ŀ��ֵ�����仯
	}
}


void fn_shoot_init(void)
{

	shoot_control.f103_data = fn_get_f103_data_point();//��ȡf103����ָ��
	fn_ShootMotorInit();//��ʼ��������3508���
	/*״̬����ʼ��*/
	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.trigger_mode = TRIGGER_DOWN;
	shoot_control.cylinder_mode = CYLIN_OPEN;
	shoot_control.quickvalve_mode = QUIVAL_CLOSE;

	fn_Shoot_feedback_update();//�������ٶȸ��£���갴������
	shoot_control.ecd_count = 0;
	shoot_control.trigger_angle = trigger_motor3508_measure[0].ecd * MOTOR_ECD_TO_ANGLE19;
	shoot_control.given_current = 0;
	shoot_control.move_flag = 0;
	shoot_control.move_flag2 = 0;
	shoot_control.trigger_angle_set = trigger_motor3508_data[0].relative_angle_19laps;
	shoot_control.trigger_speed = 0.0f;
	shoot_control.trigger_speed_set = 0.0f;
	shoot_control.key_time = 0;
	shoot_control.v_key = 0;
	shoot_control.last_v_key = 0;
	shoot_control.last_shoot_flag = 0;

	shoot_control.cylin_open2action_count = 0;
	shoot_control.cylin_action2close_count = 0;
	shoot_control.cylin_close2open_count = 0;
	shoot_control.quival_open2close_count = 0;
	shoot_control.shoot_action_plus_count = 0;
	shoot_control.angle_change_flag = 0;
	shoot_control.reverse_angle_change_flag = 0;
	shoot_control.cylinder_cmd = 0;
	shoot_control.quival_cmd = 0;
	shoot_control.last_photogate = 0;
	shoot_control.photogate = 0;

	shoot_control.block_time = 0;
	shoot_control.reverse_time = 0;

	shoot_control.shoot_enable_flag = 0;
}

void fn_Shoot_feedback_update(void)
{
	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	// �����ֵ���ٶ��˲�һ��
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	// ���׵�ͨ�˲�
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor3508_measure[0].speed_rpm * TriggerMotor3508_RPM_TO_SPEED) * fliter_num[2];
	shoot_control.trigger_speed = speed_fliter_3;

	//��갴��
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = ctl.mouse.press_l;
	shoot_control.press_r = ctl.mouse.press_r; 

}


