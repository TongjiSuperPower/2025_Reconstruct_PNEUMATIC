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
//射击模块实例化
shoot_control_t shoot_control;
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
//光电门的状态临时变量,定义为全局变量方便debug
GPIO_PinState photogate_state;
//冷却时间？
int32_t shoot_cooling_time = 0;
//当前枪口热量
uint16_t cooling_heat;
//枪口热量上限
uint16_t cooling_heat_limit;

//射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗
void fn_Shoot_set_mode(void);

//射击数据更新,更新拨弹轮速度和键鼠
void fn_Shoot_feedback_update(void);

//根据状态机计算被控量
void fn_Shoot_action(void);

//堵转倒转处理
void fn_triggermotor_turn_back(void);

//射击控制，控制拨弹电机角度，完成一次发射
void fn_shoot_bullet_control(void);

//f103控制任务
void f103_control_loop(void);


/**
 * @brief          射击循环(！！！主函数！！！)
 * @param[in]      void
 * @retval         返回can控制值
 */
void fn_shoot_control_loop(void)
{
	fn_Shoot_set_mode();	//设置状态机
	fn_Shoot_feedback_update();	//更新拨弹轮速度和键鼠
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


//拨弹轮初始化
void fn_ShootMotorInit(void)
{	
	fp32 af_TriggerMotor3508_SpeedPid[3] = {TriggerMotor3508_SpeedPid_ID207_kp ,TriggerMotor3508_SpeedPid_ID207_ki ,TriggerMotor3508_SpeedPid_ID207_kd};
	fp32 af_TriggerMotor3508_AnglePid[3] = {TriggerMotor3508_AnglePid_ID207_kp ,TriggerMotor3508_AnglePid_ID207_ki ,TriggerMotor3508_AnglePid_ID207_kd};
	fp32 af_TriggerMotor3508_ActionSpeedPid[3] = {TriggerMotor3508_ActionSpeedPid_ID207_kp ,TriggerMotor3508_ActionSpeedPid_ID207_ki ,TriggerMotor3508_ActionSpeedPid_ID207_kd};
	fp32 af_TriggerMotor3508_ActionAnglePid[3] = {TriggerMotor3508_ActionAnglePid_ID207_kp ,TriggerMotor3508_ActionAnglePid_ID207_ki ,TriggerMotor3508_ActionAnglePid_ID207_kd};
	

    trigger_motor3508_data[0].round_num = 0;
	trigger_motor3508_data[0].offecd_ecd = trigger_motor3508_measure[0].ecd;
	/*以下暂时未使用到，仅初始化*/
    trigger_motor3508_data[0].relative_raw_angle = 0.0f;

	for(uint8_t m = 0;m < 6;m++){
        trigger_motor3508_data[0].raw_angle[m] = 0.0f;
		trigger_motor3508_data[0].raw_speed[m] = 0.0f;
	}
	for(uint8_t n = 0;n < 2;n++){
		trigger_motor3508_data[0].filter_angle[n] = 0.0f;
		trigger_motor3508_data[0].filter_speed[n] = 0.0f;
	}
	/*以上暂时未用到，仅初始化*/
    trigger_motor3508_data[0].relative_raw_speed = 0.0f;
	trigger_motor3508_data[0].target_angle = 0.0f;
	trigger_motor3508_data[0].target_speed = 0.0f;
	trigger_motor3508_data[0].filter_given_current = 0.0f;
    trigger_motor3508_data[0].given_current = 0.0f;
    trigger_motor3508_data[0].double_pid_mid = 0.0f;

	/*初始化pid*/
	fn_PidInit(&shoot_control.trigger_speed_pid,af_TriggerMotor3508_SpeedPid,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	fn_PidInit(&shoot_control.trigger_angle_pid,af_TriggerMotor3508_AnglePid,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	fn_PidInit(&shoot_control.trigger_action_speed_pid,af_TriggerMotor3508_ActionSpeedPid,TriggerMotor3508MinOut,TriggerMotor3508MaxOut,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	fn_PidInit(&shoot_control.trigger_action_angle_pid,af_TriggerMotor3508_ActionAnglePid,-4.0 ,4.0,TriggerMotor3508MinIOut,TriggerMotor3508MaxIOut);
	
}


void fn_Shoot_set_mode(void)
{
	static int8_t last_s = RC_SW_UP; //上一时刻的左拨杆值
	shoot_control.v_key = ctl.key.v & AUTOAIM_SHOOT_KEY;
	photogate_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	shoot_control.photogate = (photogate_state == GPIO_PIN_SET);
	
	//右拨杆拨到下档则down
	if(IF_RC_SW2_DOWN)
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	//整车不处在down模式
	else
	{
		if(IF_RC_SW2_UP)//遥控器模式
		{
			//左拨杆中到上模式切换
			if(IF_RC_SW1_UP && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP)
			{
				shoot_control.shoot_mode = SHOOT_READY;
			}
			else if(IF_RC_SW1_UP && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP)
			{
				shoot_control.shoot_mode = SHOOT_STOP;
			}
		}
		else if(IF_RC_SW2_MID)//键鼠模式
		{
			shoot_control.shoot_mode = SHOOT_READY;
		}
		//如果云台状态是 无力状态，就关闭射击
		if(gimbal_data.gimbal_behaviour == GIMBAL_ZERO_FORCE || gimbal_data.gimbal_behaviour == GIMBAL_INIT)
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		if((shoot_control.press_l && shoot_control.last_press_l == 0 ) ||
		(autoaim_measure.shoot == 1 && autoaim_measure.vision_state != 0 && shoot_control.press_r && shoot_control.v_key))
		{
			shoot_control.shoot_enable_flag = 1;
		}//左键发射或者右键自瞄的同时按V发射
		shoot_cooling_time++;

		// 在READY，并且拨盘停转、缸闭合的情况下如果下拨左拨杆，进入射击模式
		if(((IF_RC_SW1_DOWN && !switch_is_down(last_s) && IF_RC_SW2_UP) ||(shoot_control.shoot_enable_flag && IF_RC_SW2_MID))
		&& shoot_control.shoot_mode == SHOOT_READY && shoot_control.cylinder_mode == CYLIN_CLOSE && shoot_control.trigger_mode == TRIGGER_DOWN)
		{
			cooling_heat = ext_power_heat_data.shooter_42mm_barrel_heat;	//当前枪口热量
			cooling_heat_limit = ext_robot_status.shooter_barrel_heat_limit;	//枪口热量上限

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

		// 1.如果进入READY模式
		if(shoot_control.shoot_mode == SHOOT_READY)
		{
			// 如果光电门未检测到信号，则进入拨盘ACTION，否则拨盘不动
			if(shoot_control.photogate != 1 && shoot_control.cylinder_mode == CYLIN_OPEN)
			{
				shoot_control.trigger_mode = TRIGGER_ACTION;

				if(fabs(shoot_control.trigger_speed) <= BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME) //判断堵转时间
				{
					shoot_control.block_time++;
					shoot_control.reverse_time = 0;	// 没有确定是堵转，则不发生反转，反转时间清零
				}
				if(shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME) //确定是堵转，开始记录反转时间
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
				shoot_control.trigger_mode =TRIGGER_ACTION_PLUS; //拨弹轮拨弹完毕	
			}
			
			if(shoot_control.trigger_mode == TRIGGER_ACTION_PLUS)
			{
				shoot_control.shoot_action_plus_count++;
				if(shoot_control.shoot_action_plus_count >= 250)//检测到光电门信号后再转一下
				{
					shoot_control.shoot_action_plus_count = 0;
					shoot_control.trigger_mode = TRIGGER_DOWN;
				}
			}

			//如果拨盘停转同时光电门检测到物体，判断缸的模式
			if(shoot_control.trigger_mode == TRIGGER_DOWN && shoot_control.photogate == 1)
			{
				//如果缸已经关闭，满足条件
				if(shoot_control.cylinder_mode == CYLIN_CLOSE)
				{}
				//如果缸是打开状态，延时500ms，进入ACTION
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
			// 如果现在是关闭进行中，延时500ms，进入close
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
			// READY状态不开快排阀
			shoot_control.quickvalve_mode = QUIVAL_CLOSE;
		}

		// 2.如果现在是BULLET模式，则打开快排阀
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

	// 3.如果现在是STOP模式，则停止拨盘转动，同时快排阀关闭，缸打开
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

	last_s = ctl.rc.s1;	//上一时刻的左拨杆值
	shoot_control.last_photogate = shoot_control.photogate;
	shoot_control.last_v_key = shoot_control.v_key;

}


void fn_Shoot_action(void)
{
	if(shoot_control.shoot_mode == SHOOT_STOP)// stop模式代表gimbal出现某些问题，例如在遥控器down状态、校准阶段等，需要全面停止射击系统
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
		shoot_control.trigger_angle_pid.f_MaxOut = 4.0f;	//？疑问
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
	// 每次拨动大于 1/3PI 的角度，使弹丸一定能到达目标值
	if(shoot_control.move_flag == 0)	// 置零表示可以改变目标角度，说明已进入过down模式，并且flag清零，可以开始下一次拨弹
	{
		shoot_control.trigger_angle_set = fn_RadFormat(trigger_motor3508_data[0].relative_angle_19laps) + PI_THREE * 8.0f / 3.0f;
		shoot_control.move_flag = 1;	// 改变目标角度后即改变flag，防止在当前轮拨弹过程中目标值发生变化
	}
}


void fn_shoot_init(void)
{

	shoot_control.f103_data = fn_get_f103_data_point();//获取f103数据指针
	fn_ShootMotorInit();//初始化拨弹轮3508电机
	/*状态机初始化*/
	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.trigger_mode = TRIGGER_DOWN;
	shoot_control.cylinder_mode = CYLIN_OPEN;
	shoot_control.quickvalve_mode = QUIVAL_CLOSE;

	fn_Shoot_feedback_update();//拨弹轮速度更新，鼠标按键更新
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

	// 拨弹轮电机速度滤波一下
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	// 二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor3508_measure[0].speed_rpm * TriggerMotor3508_RPM_TO_SPEED) * fliter_num[2];
	shoot_control.trigger_speed = speed_fliter_3;

	//鼠标按键
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = ctl.mouse.press_l;
	shoot_control.press_r = ctl.mouse.press_r; 

}


