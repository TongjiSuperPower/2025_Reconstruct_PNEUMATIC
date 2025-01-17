#include "gimbal_task.h"
#include "struct_typedef.h"
#include "pid.h"
#include "can_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "math_lib.h"
#include "stdlib.h"
#include "math.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "referee_task.h"
#include "plot.h"
#include "chassis_task.h"
#include "stdbool.h"

//射击数据发送频率控制符
uint16_t shoot_mode_sned_num;
uint16_t q_send_num;


//云台数据结构定义
gimbal_data_t gimbal_data;

//云台回中模式下回中后的时间
uint16_t gimbal_init_over_time;
//云台进入回中模式的时间
uint16_t gimbal_init_time;
//云台是否在回中模式
uint8_t gimbal_init_flag;
//掉头冷却时间1s
uint16_t turnover_cold_time;
//发送给上位机的射击状态标识符
uint8_t shoot_mode_flag;
//云台进入zero_force后低重力补偿持续时间
uint16_t gimbal_into_zero_force_time;


//云台数据初始化
void fn_GimbalInit(void);
//云台电机初始化函数
void fn_GimbalMotorInit(void);
//云台状态选择
void fn_GimbalMode(void);
//电机模式选择
void fn_MotorMode(void);
//云台电流解算
void fn_GimbalMove(void);
//读取零点
void fn_get_pitch_zero_encode(void);



//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
void Gimbal_Task(void const * argument){
    
    //等待一段时间
    vTaskDelay(500);

    //云台电机初始化
    fn_GimbalMotorInit();

    //云台初始化
    fn_GimbalInit();


    //射击模块初始化
    fn_shoot_init();

    //读取pitch零点
    fn_get_pitch_zero_encode();

   

    while(1){

        //遥控器数据出错或翻车则停掉电机
        while(remote_control_data_error_flag || rollover_flag){
            //fn_cmd_CAN2GimbalMotor(0,0,0,0);
            fn_cmd_CAN1GimbalMotor1(0,0,0,0);
            //fn_cmd_CAN1GimbalMotor2(0,0,0,0);
            motor_mi_controlmode(0,0,0,0,0);
			vTaskDelay(100);
        }

        //云台电机选择模式
        fn_GimbalMode();

        //计算射击拨弹轮电流
        fn_shoot_control_loop();

        //计算云台电机电流
        fn_GimbalMove();

        
        
        //给上位机发送射击数据 50Hz
        if(shoot_mode_sned_num > 19){
            fn_cmd_shoot_data_to_computer(ext_robot_shoot_data.initial_speed,shoot_mode_flag);
            shoot_mode_sned_num = 0;
        }
        shoot_mode_sned_num++;

        //给上位机发送四元数 500Hz
        if(q_send_num >= 3){
            fn_cmd_quat_to_computer(INS_quat[1],INS_quat[2],INS_quat[3],INS_quat[0]);
			q_send_num=1;
		}
		q_send_num++;
				
				
        
		motor_mi_controlmode(gimbal_motormi_data[0].given_current,0,0,0,0);
        
		fn_cmd_CAN2TriggerMotor(0,0,shoot_control.given_current,0);		



        //频率1000Hz
        vTaskDelay(1);

    }
}



//云台数据初始化
void fn_GimbalInit(void){
    fp32 af_GimbalIMUPosPid1[2][3] = {{GimbalPid1Yaw_kp,GimbalPid1Yaw_ki,GimbalPid1Yaw_kd},
                                      {GimbalPid1Pitch_kp,GimbalPid1Pitch_ki,GimbalPid1Pitch_kd}};
    fp32 af_GimbalIMUPosPid2[2][3] = {{GimbalPid2Yaw_kp,GimbalPid2Yaw_ki,GimbalPid2Yaw_kd},
                                      {GimbalPid2Pitch_kp,GimbalPid2Pitch_ki,GimbalPid2Pitch_kd}};

    fp32 af_GimbalIMUPosAutoaimPid1[2][3] = {{GimbalPid1YawAutoaim_kp,GimbalPid1YawAutoaim_ki,GimbalPid1YawAutoaim_kd},
                                             {GimbalPid1PitchAutoaim_kp,GimbalPid1PitchAutoaim_ki,GimbalPid1PitchAutoaim_kd}};
    fp32 af_GimbalIMUPosAutoaimPid2[2][3] = {{GimbalPid2YawAutoaim_kp,GimbalPid2YawAutoaim_ki,GimbalPid2YawAutoaim_kd},
                                             {GimbalPid2PitchAutoaim_kp,GimbalPid2PitchAutoaim_ki,GimbalPid2PitchAutoaim_kd}};

    shoot_mode_sned_num = 0;
    
    gimbal_init_over_time = 0;
    gimbal_init_time = 0;
    gimbal_init_flag = 0;
    shoot_mode_flag = 0;
    gimbal_into_zero_force_time = 2000;
    turnover_cold_time = TurnOverColdTime;

    gimbal_data.gyro_yaw_target_angle = INS_eulers[0];
    gimbal_data.gyro_pit_target_angle = INS_eulers[1];

    gimbal_data.f_GimbalYawPidMid = 0.0f;
    gimbal_data.f_GimbalPitPidMid = 0.0f;

    gimbal_data.gimbal_behaviour = GIMBAL_ZERO_FORCE;
    gimbal_data.last_gimbal_behaviour = GIMBAL_ZERO_FORCE;

    gimbal_data.gimbal_motor_yaw_mode = GIMBAL_Motor_DOWN;
    gimbal_data.gimbal_motor_pit_mode = GIMBAL_Motor_DOWN;


    fn_PidInit(&gimbal_data.GimbalIMUYawPid1,af_GimbalIMUPosPid1[0],-20,20,-2,2);
    fn_PidInit(&gimbal_data.GimbalIMUYawPid2,af_GimbalIMUPosPid2[0],GimbalPidYawMinOut,GimbalPidYawMaxOut,GimbalPidYawMinIOut,GimbalPidYawMaxIOut);

    fn_PidInit(&gimbal_data.GimbalIMUPitPid1,af_GimbalIMUPosPid1[1],GimbalPid1PitchMinOut,GimbalPid1PitchMaxOut,GimbalPid1PitchMinIOut,GimbalPid1PitchMaxIout);
    fn_PidInit(&gimbal_data.GimbalIMUPitPid2,af_GimbalIMUPosPid2[1],GimbalPidPitMinOut,GimbalPidPitMaxOut,GimbalPidPitMinIOut,GimbalPidPitMaxIOut);

    fn_PidInit(&gimbal_data.GimbalIMUYawAutoaimPid1,af_GimbalIMUPosAutoaimPid1[0],-20,20,-2,2);
    fn_PidInit(&gimbal_data.GimbalIMUYawAutoaimPid2,af_GimbalIMUPosAutoaimPid2[0],GimbalPidYawMinOut,GimbalPidYawMaxOut,GimbalPidYawMinIOut,GimbalPidYawMaxIOut);

    fn_PidInit(&gimbal_data.GimbalIMUPitAutoaimPid1,af_GimbalIMUPosAutoaimPid1[1],-10,10,-1,1);
    fn_PidInit(&gimbal_data.GimbalIMUPitAutoaimPid2,af_GimbalIMUPosAutoaimPid2[1],GimbalPidPitMinOut,GimbalPidPitMaxOut,GimbalPidPitMinIOut,GimbalPidPitMaxIOut);
}

//云台电机初始化函数
void fn_GimbalMotorInit(void){
    
    //云台电机初始化
    fp32 af_GimbalMotorPosPid1[2][3] = {{GimbalMotor4310PosPid1_Yaw_kp,GimbalMotor4310PosPid1_Yaw_ki,GimbalMotor4310PosPid1_Yaw_kd},
	                                    {GimbalMotormiPosPid1_Pit_kp,GimbalMotormiPosPid1_Pit_ki,GimbalMotormiPosPid1_Pit_kd}};
	fp32 af_GimbalMotorPosPid2[2][3] = {{GimbalMotor4310PosPid2_Yaw_kp,GimbalMotor4310PosPid2_Yaw_ki,GimbalMotor4310PosPid2_Yaw_kd},
	                                    {GimbalMotormiPosPid2_Pit_kp,GimbalMotormiPosPid2_Pit_ki,GimbalMotormiPosPid2_Pit_kd}};


    //YAW轴达妙电机初始化
	fn_DM_start_motor();

    gimbal_motor4310_data[0].round_num = 0;
    gimbal_motor4310_data[0].target_angle = 0.0f;
    gimbal_motor4310_data[0].target_torque = 0.0f;
    gimbal_motor4310_data[0].offecd_angle = 0.335507f;
																			

    gimbal_motor4310_data[0].double_pid_mid = 0.0f;
    fn_PidInit(&gimbal_motor4310_data[0].motor_pid1,af_GimbalMotorPosPid1[0],-30,30,-0.2,0.2);
	fn_PidInit(&gimbal_motor4310_data[0].motor_pid2,af_GimbalMotorPosPid2[0],GimbalPidYawMinOut,GimbalPidYawMaxOut,GimbalPidYawMinIOut,GimbalPidYawMaxIOut);

    //PIT小米电机
    init_cybergear(0);
    

    gimbal_motormi_data[0].relative_raw_angle = 0.0f;
    //gimbal_motor6020_data[i].offecd_ecd = gimbal_motor3508_measure[i].ecd;
    //gimbal_motormi_data[0].offecd_ecd = 33910;
	
    //gimbal_motormi_data[0].offecd_ecd = Pitch_offset_ecd;
		
    
	for(uint8_t m = 0;m < 6;m++){
        gimbal_motormi_data[0].raw_angle[m] = 0.0f; 
	}
	for(uint8_t n = 0;n < 2;n++){
		gimbal_motormi_data[0].filter_angle[n] = 0.0f;
	}

    gimbal_motormi_measure[0].ecd = 0;

    gimbal_motormi_data[0].relative_raw_speed = 0.0f;                    
	gimbal_motormi_data[0].target_angle = 0.0f;         
    gimbal_motormi_data[0].given_current = 0.0f;           
    gimbal_motormi_data[0].double_pid_mid = 0.0f;
		
    fn_PidInit(&gimbal_motormi_data[0].motor_pid1,af_GimbalMotorPosPid1[1],-30,30,-3,3);
	fn_PidInit(&gimbal_motormi_data[0].motor_pid2,af_GimbalMotorPosPid2[1],GimbalPidPitMinOut,GimbalPidPitMaxOut,GimbalPidPitMinIOut,GimbalPidPitMaxIOut);

}


void fn_get_pitch_zero_encode(void){
    // gimbal_motormi_data[0].offecd_ecd = 34130;
	  uint16_t delay = 0;
    while(delay < 300){
        motor_mi_controlmode(-0.5f,0,0,0,0);
	    if(fabs(gimbal_motormi_data[0].relative_raw_speed) < 0.5f && gimbal_motormi_measure[0].ecd != 0){
		    delay++;
	    }
        else{
            delay = 0;
        }
        if(delay == 200){
            gimbal_motormi_data[0].offecd_ecd = gimbal_motormi_measure[0].ecd - 1650;
        }
        vTaskDelay(1);
    }
}

//云台状态选择
void fn_GimbalMode(void){
    //正在回中过程中无法调整模式
    if(gimbal_init_flag == 1){
        return;
    }
    
    //遥控器
    if(!IF_RC_SW2_MID){
        if(IF_RC_SW2_DOWN){
            gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
            gimbal_data.gimbal_behaviour = GIMBAL_ZERO_FORCE;
            shoot_mode_flag = 0;
        }

        if(gimbal_data.last_gimbal_behaviour != GIMBAL_ZERO_FORCE && gimbal_data.gimbal_behaviour == GIMBAL_ZERO_FORCE){
            gimbal_into_zero_force_time = 0;
        }

        if(IF_RC_SW2_UP){
            gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
            gimbal_data.gimbal_behaviour = GIMBAL_GYRO;
            shoot_mode_flag = 0;
        }
    }

    //键鼠 长按鼠标右键自瞄 只长按B打符
    if(IF_RC_SW2_MID){
        gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
        
        if(!IF_MOUSE_PRESSED_RIGHT){
           gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
           gimbal_data.gimbal_behaviour = GIMBAL_GYRO;
           shoot_mode_flag = 0;
        }
        // 按下右键时判定为自瞄模式
        else{
           gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
           gimbal_data.gimbal_behaviour = GIMBAL_AUTO;
        }
    }
    
    //判断是否进入回中模式
    if(gimbal_data.last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_data.gimbal_behaviour != GIMBAL_ZERO_FORCE){
        gimbal_data.gimbal_behaviour = GIMBAL_INIT;
        gimbal_init_flag = 1;
        shoot_mode_flag = 0;
    }

    //根据云台状态选择电机控制模式
    fn_MotorMode();

    //覆盖自瞄数据，保证模式切换的连续性
    if(gimbal_data.gimbal_behaviour != GIMBAL_AUTO){
        autoaim_measure.vision_state = 0;
    }
}

//电机模式选择
void fn_MotorMode(void){
    if(gimbal_data.gimbal_behaviour == GIMBAL_ZERO_FORCE){
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_Motor_DOWN;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_Motor_DOWN;
    }
    if(gimbal_data.gimbal_behaviour == GIMBAL_INIT){
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_ENCONDE;
    }
    if(gimbal_data.gimbal_behaviour == GIMBAL_GYRO){
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_GYRO;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_GYRO;
    }
    if(gimbal_data.gimbal_behaviour == GIMBAL_ENCONDE){
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_ENCONDE;
    }
    if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO){
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_GYRO;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_GYRO;
    }
}

//云台电流解算
void fn_GimbalMove(void){

    //电机为模式DOWN
    if(gimbal_data.gimbal_motor_yaw_mode == GIMBAL_Motor_DOWN && gimbal_data.gimbal_motor_pit_mode == GIMBAL_Motor_DOWN){
        if(gimbal_into_zero_force_time < 2000){
            gimbal_into_zero_force_time ++;
        }

        gimbal_motor4310_data[0].target_torque = 0.0f;
        if(gimbal_into_zero_force_time < 2000){
            gimbal_motormi_data[0].given_current = sin(PI / 2.0f - OFFSET_ANGLE + gimbal_motormi_data[0].relative_raw_angle) * Tor_param * 0.7;
		}
        else{
            gimbal_motormi_data[0].given_current = 0.0f;
        }
    }
        
    //电机模式为GYRO
    if(gimbal_data.gimbal_motor_yaw_mode == GIMBAL_MOTOR_GYRO && gimbal_data.gimbal_motor_pit_mode == GIMBAL_MOTOR_GYRO){
        //遥控器
        if(!IF_RC_SW2_MID){
            //陀螺仪控云台
            if(gimbal_data.gimbal_behaviour == GIMBAL_GYRO){
                //获取角速度
                gimbal_data.gyro_yaw_angle_add = -(float)(ctl.rc.ch2 - 1024) / 660.0f * WMax;
                gimbal_data.gyro_pit_angle_add = (float)(ctl.rc.ch3 - 1024) / 660.0f * WMax;
                //pitch轴限角 暂时未考虑底盘斜置于斜坡上的情况即yaw轴对pitch角度的影响与pitch轴角度自增带来的超出限位情况以及自增超范围的情况
                if(fn_scope_judgment(gimbal_motormi_data[0].relative_raw_angle,PitAngleMin,PitAngleMax)){
                    if(fabs(gimbal_motormi_data[0].relative_raw_angle + gimbal_data.gyro_pit_angle_add) > fabs(gimbal_motormi_data[0].relative_raw_angle)){
                        gimbal_data.gyro_pit_angle_add = 0.0f;
                    }
                }
                //赋值目标角度
                gimbal_data.gyro_yaw_target_angle = fn_RadFormat(gimbal_data.gyro_yaw_target_angle + gimbal_data.gyro_yaw_angle_add);
                gimbal_data.gyro_pit_target_angle = fn_RadFormat(gimbal_data.gyro_pit_target_angle + gimbal_data.gyro_pit_angle_add);

                //解算GYRO模式下两轴电流
                gimbal_data.f_GimbalYawPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUYawPid1,INS_eulers[0], gimbal_data.gyro_yaw_target_angle);
                gimbal_motor4310_data[0].target_torque = fn_PidClac(&gimbal_data.GimbalIMUYawPid2,INS_gyro[2],gimbal_data.f_GimbalYawPidMid);

                gimbal_data.f_GimbalPitPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUPitPid1,INS_eulers[1], gimbal_data.gyro_pit_target_angle);
                gimbal_motormi_data[0].given_current = fn_PidClac(&gimbal_data.GimbalIMUPitPid2,INS_gyro[0],gimbal_data.f_GimbalPitPidMid)
                                                        + sin(PI / 2.0f - OFFSET_ANGLE + gimbal_motormi_data[0].relative_raw_angle) * Tor_param;
            }
        }

        //键鼠
        if(IF_RC_SW2_MID){
            // 记录按键的状态
            static bool KEY_W_PRESSED = false;
            static bool KEY_S_PRESSED = false;
            static bool KEY_A_PRESSED = false;
            static bool KEY_D_PRESSED = false;
            static bool KEY_Q_PRESSED = false;
            static bool KEY_E_PRESSED = false;
            //陀螺仪控云台
            if(gimbal_data.gimbal_behaviour == GIMBAL_GYRO){
                if(chassis_move_data.chassis_mode == chassis_not_follow){
                    
                    if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S){
                        if(IF_KEY_PRESSED_W && !KEY_W_PRESSED){
                            gimbal_data.gyro_pit_angle_add = WCoef * 0.5f;
                            KEY_W_PRESSED = true;
                        }
                        else if(IF_KEY_PRESSED_S && !KEY_S_PRESSED){
                            gimbal_data.gyro_pit_angle_add = -WCoef * 0.5f;
                            KEY_S_PRESSED = true;
                        }
                    }
                    else{
                        KEY_W_PRESSED = false;
                        KEY_S_PRESSED = false;
                        gimbal_data.gyro_pit_angle_add = 0.0f;
                    }

                    if(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D){
                        if(IF_KEY_PRESSED_A && !KEY_A_PRESSED){
                            gimbal_data.gyro_yaw_angle_add = WCoef * 0.5f;
                            KEY_A_PRESSED = true;
                        }
                        else if(IF_KEY_PRESSED_D && !KEY_D_PRESSED){
                            gimbal_data.gyro_yaw_angle_add = -WCoef * 0.5f;
                            KEY_D_PRESSED = true;
                        }
                    }
                    else{
                        KEY_A_PRESSED = false;
                        KEY_D_PRESSED = false;
                        gimbal_data.gyro_yaw_angle_add = 0.0f;
                    }
                }
                else{
                    gimbal_data.gyro_yaw_angle_add = -MOUSE_X_MOVE_SPEED * WCoef;
                    gimbal_data.gyro_pit_angle_add = -MOUSE_Y_MOVE_SPEED * WCoef;
                }
                //掉头冷却减少
                if(turnover_cold_time > 0){
                    turnover_cold_time--;
                }
                if(IF_KEY_PRESSED_Q && !KEY_Q_PRESSED){
                    gimbal_data.gyro_yaw_angle_add = WCoef * 70;
                }
                if(IF_KEY_PRESSED_E && !KEY_E_PRESSED){
                    gimbal_data.gyro_yaw_angle_add = -WCoef * 70;
                }
                //按下X键回头
                if(IF_KEY_PRESSED_X && turnover_cold_time == 0){
                    gimbal_data.gyro_yaw_angle_add = PI;
                    turnover_cold_time = TurnOverColdTime;
                }
                //pitch轴限角 暂时未考虑底盘斜置于斜坡上的情况即yaw轴对pitch角度的影响与pitch轴角度自增带来的超出限位情况以及自增超范围的情况
                if(fn_scope_judgment(gimbal_motormi_data[0].relative_raw_angle,PitAngleMin,PitAngleMax)){
                    if(fabs(gimbal_motormi_data[0].relative_raw_angle + gimbal_data.gyro_pit_angle_add) > fabs(gimbal_motormi_data[0].relative_raw_angle)){
                        gimbal_data.gyro_pit_angle_add = 0.0f;
                    }
                }

                //限制yaw轴角度增量
                if (gimbal_data.gyro_yaw_angle_add >= 3.2f) {
                    gimbal_data.gyro_yaw_angle_add = 3.2f;
                }
                else if(gimbal_data.gyro_yaw_angle_add <= -3.2f){
                    gimbal_data.gyro_yaw_angle_add = -3.2f;
                }

                //赋值目标角度
                gimbal_data.gyro_yaw_target_angle = fn_RadFormat(gimbal_data.gyro_yaw_target_angle + gimbal_data.gyro_yaw_angle_add);
                gimbal_data.gyro_pit_target_angle = fn_RadFormat(gimbal_data.gyro_pit_target_angle + gimbal_data.gyro_pit_angle_add);

                //解算GYRO模式下两轴电流
                gimbal_data.f_GimbalYawPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUYawPid1,INS_eulers[0], gimbal_data.gyro_yaw_target_angle);
                gimbal_motor4310_data[0].target_torque = fn_PidClac(&gimbal_data.GimbalIMUYawPid2,INS_gyro[2],gimbal_data.f_GimbalYawPidMid);

                gimbal_data.f_GimbalPitPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUPitPid1,INS_eulers[1], gimbal_data.gyro_pit_target_angle);
                gimbal_motormi_data[0].given_current = fn_PidClac(&gimbal_data.GimbalIMUPitPid2,INS_gyro[0],gimbal_data.f_GimbalPitPidMid)
                                                        + sin(PI / 2.0f - OFFSET_ANGLE + gimbal_motormi_data[0].relative_raw_angle) * Tor_param;
            }

        //自瞄控云台
        if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO){
            shoot_mode_flag = 1;
            

            //赋予自瞄坐标
            if(autoaim_measure.vision_state == 1){
                gimbal_data.gyro_yaw_target_angle = autoaim_measure.yaw;
                gimbal_data.gyro_pit_target_angle = autoaim_measure.pitch;
            }

            /*加一个电控限位*/
            if(fn_scope_judgment(gimbal_data.gyro_pit_target_angle,PitAngleMin,PitAngleMax)){
                if(gimbal_data.gyro_pit_target_angle >PitAngleMax){
                    gimbal_data.gyro_pit_target_angle = PitAngleMax;
                }
                else if(gimbal_data.gyro_pit_target_angle < PitAngleMin){
                    gimbal_data.gyro_pit_target_angle = PitAngleMin;
                }
            }
            //解算GYRO模式下两轴电流
            gimbal_data.f_GimbalYawPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUYawPid1,INS_eulers[0], gimbal_data.gyro_yaw_target_angle);
            gimbal_motor4310_data[0].target_torque = fn_PidClac(&gimbal_data.GimbalIMUYawPid2,INS_gyro[2],gimbal_data.f_GimbalYawPidMid);

            gimbal_data.f_GimbalPitPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUPitPid1,INS_eulers[1], gimbal_data.gyro_pit_target_angle);
            gimbal_motormi_data[0].given_current = fn_PidClac(&gimbal_data.GimbalIMUPitPid2,INS_gyro[0],gimbal_data.f_GimbalPitPidMid)
                                                    + sin(PI / 2.0f - OFFSET_ANGLE + gimbal_motormi_data[0].relative_raw_angle) * Tor_param;
            //解算自瞄模式下两轴电流
            // gimbal_data.f_GimbalYawAutoaimPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUYawAutoaimPid1,INS_eulers[0], gimbal_data.gyro_yaw_target_angle);
            // gimbal_motor4310_data[0].target_torque = fn_PidClac(&gimbal_data.GimbalIMUYawAutoaimPid2,INS_gyro[2],gimbal_data.f_GimbalYawAutoaimPidMid);

            // gimbal_data.f_GimbalPitAutoaimPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUPitAutoaimPid1,INS_eulers[1], gimbal_data.gyro_pit_target_angle);
            // gimbal_motormi_data[0].given_current = -fn_PidClac(&gimbal_data.GimbalIMUPitAutoaimPid2,INS_gyro[0],gimbal_data.f_GimbalPitAutoaimPidMid)
            //                                         + sin(PI / 2.0f - OFFSET_ANGLE + gimbal_motormi_data[0].relative_raw_angle) * Tor_param;
        }

            //不在自瞄则将自瞄PID的Iout清零，减少对后续的影响
//            if(gimbal_data.gimbal_behaviour != GIMBAL_AUTO){
//                gimbal_data.GimbalIMUYawAutoaimPid1.f_Iout *= 1.0f;
//                gimbal_data.GimbalIMUYawAutoaimPid2.f_Iout *= 1.0f;

//                gimbal_data.GimbalIMUPitAutoaimPid1.f_Iout *= 1.0f;
//                gimbal_data.GimbalIMUPitAutoaimPid2.f_Iout *= 1.0f;
//            }
        }
    }

    //电机模式为ENCODE
    if(gimbal_data.gimbal_motor_yaw_mode == GIMBAL_MOTOR_ENCONDE && gimbal_data.gimbal_motor_pit_mode == GIMBAL_MOTOR_ENCONDE){
        //云台初始化
        if(gimbal_data.gimbal_behaviour == GIMBAL_INIT){

            //赋予目标角度
            gimbal_motor4310_data[0].target_angle = chassis_move_data.chassis_relative_angle_set;
            gimbal_motormi_data[0].target_angle = 0.0f;
                    
            //判断是否已经回中
            if((fabs(fn_RadFormat(gimbal_motor4310_data[0].position - chassis_move_data.chassis_relative_angle_set))) < 0.05f && (fabs(gimbal_motormi_data[0].relative_raw_angle - 0.0f) < 0.05f)){
                gimbal_init_over_time++;
            }

            gimbal_init_time++;
                    
            //判断初始化完成 进入init模式3s或者回中后持续0.5s则认为init模式完成
            if(gimbal_init_time == 3000 || gimbal_init_over_time == 500){
                gimbal_data.gyro_yaw_target_angle = INS_eulers[0];
                gimbal_data.gyro_pit_target_angle = INS_eulers[1];
                gimbal_init_over_time = 0;
                gimbal_init_time = 0;
                gimbal_init_flag = 0;
            }
        }

        //解算ENCODE模式下两轴电流
        gimbal_motor4310_data[0].double_pid_mid = fn_PidClacAngle(&gimbal_motor4310_data[0].motor_pid1,
                                                gimbal_motor4310_data[0].position,gimbal_motor4310_data[0].target_angle);
        gimbal_motor4310_data[0].target_torque = fn_PidClac(&gimbal_motor4310_data[0].motor_pid2,
                                                gimbal_motor4310_data[0].velocity,gimbal_motor4310_data[0].double_pid_mid);
        gimbal_motormi_data[0].double_pid_mid = fn_PidClacAngle(&gimbal_motormi_data[0].motor_pid1,
                                                gimbal_motormi_data[0].relative_raw_angle,gimbal_motormi_data[0].target_angle);
        gimbal_motormi_data[0].given_current = fn_PidClac(&gimbal_motormi_data[0].motor_pid2,
                                                gimbal_motormi_data[0].relative_raw_speed,gimbal_motormi_data[0].double_pid_mid)
                                                + sin(PI / 2.0f - OFFSET_ANGLE + gimbal_motormi_data[0].relative_raw_angle) * Tor_param;
    }
}
