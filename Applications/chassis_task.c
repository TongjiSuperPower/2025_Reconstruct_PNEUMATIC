#include "chassis_task.h"
#include "struct_typedef.h"
#include "pid.h"
#include "math_lib.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "can_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "filter.h"
#include "power_control.h"
#include "math.h"
#include "referee_task.h"
#include "supercap_task.h"
#include "ins_task.h"
#include "shoot_task.h"
#include "plot.h"

uint16_t send_num = 0;

uint8_t cap_send_num = 0;

uint8_t chassis_send_num = 0;

uint16_t chassis_mode_time;

//定义底盘数据
chassis_move_t chassis_move_data;
//底盘小陀螺观测时间计数
uint16_t spin_w_count;
//小陀螺速度观测flag 为0则此功率下没有在观测 为1则是观测中
uint8_t spin_w_flag;
//当前底盘最大功率观测值
fp32 relative_Pmax;


//底盘初始化函数
void fn_ChassisInit(void);
//底盘电机初始化函数
void fn_ChassisMotorInit(void);
//底盘模式选择
void fn_ChassisMode(void);
//底盘电流解算
void fn_ChassisMove(void);




void Chassis_Task(void const * argument){

    vTaskDelay(500);

    //电机底盘初始化
    fn_ChassisInit();
    fn_ChassisMotorInit();

    while(1){

        //遥控器数据出错或翻车则停掉电机
        while(remote_control_data_error_flag || rollover_flag){
            fn_cmd_CAN2ChassisMotor(0,0,0,0);
            fn_cmd_CAN2TriggerMotor(0,0,0,0);
					fn_ctrl_DM_motor(0,0,0,0,0);
			vTaskDelay(100);
        }

        //底盘模式选择
        fn_ChassisMode();

        //计算底盘电流
        fn_ChassisMove();
        
        //发送电容数据
        if(cap_send_num > 9){
            fn_cmd_supercap(cap_FSM, ext_robot_status.chassis_power_limit, ext_robot_status.power_management_chassis_output, 
            ext_power_heat_data.chassis_power, ext_power_heat_data.buffer_energy);
            cap_send_num = 0;
        }
        cap_send_num++;
        //发送电流
        if(chassis_send_num > 1){
            //fn_cmd_CAN2ChassisMotor(chassis_motor3508_data[0].given_current,chassis_motor3508_data[1].given_current,chassis_motor3508_data[2].given_current,chassis_motor3508_data[3].given_current);
            //fn_cmd_CAN2ChassisMotor(0,0,0,0);
            chassis_send_num = 0;
        }
        chassis_send_num ++;
        fn_ctrl_DM_motor(0,0,0,0,gimbal_motor4310_data[0].target_torque);
        //fn_ctrl_DM_motor(0,0,0,0,0);
        
        //fn_cmd_CAN2TriggerMotor(0,0,0,0);

        if(send_num > 49){
			//plot4(gimbal_data.gyro_yaw_target_angle*57.3f,gimbal_data.gyro_pit_target_angle*57.3f,INS_eulers[0]*57.3f,INS_eulers[1]*57.3f);
					plot6(ext_power_heat_data.chassis_power,infact_Pmax,ext_power_heat_data.buffer_energy,Pin,w_match,K);
					//plot3(INS_eulers[0],gimbal_motor4310_data[0].position,fn_RadFormat(INS_eulers[0]-gimbal_motor4310_data[0].position));
					//plot6(ext_power_heat_data.chassis_power,infact_Pmax,ext_power_heat_data.buffer_energy,cap_data.Capacity,cap_data.Cell_Power,cap_data.Cap_Power);
					//plot5(ext_power_heat_data.chassis_power,infact_Pmax,ext_power_heat_data.buffer_energy,cap_data.Capacity / 100.0f,cap_num);
			//plot5(ext_power_heat_data.shooter_17mm_1_barrel_heat,ext_robot_status.shooter_barrel_heat_limit,ext_robot_shoot_data.initial_speed
			      //,trigger_motor2006_data[0].relative_raw_speed,trigger_motor2006_data[0].target_speed);
					//plot3(ext_power_heat_data.shooter_17mm_1_barrel_heat,ext_robot_status.shooter_barrel_heat_limit,ext_robot_status.shooter_barrel_cooling_value);
			//plot6(gimbal_motor3508_data[0].relative_raw_speed,-gimbal_motor3508_data[1].relative_raw_speed,shoot_single_time_count,shooting_single_count,speed,trigger_motor2006_block_flag);
			//plot4(-chassis_motor3508_data[0].relative_raw_speed,chassis_motor3508_data[1].relative_raw_speed,
			    //-chassis_motor3508_data[2].relative_raw_speed,chassis_motor3508_data[3].relative_raw_speed);
			//plot1(gimbal_motor6020_data[0].relative_raw_speed);
					//plot5(-gimbal_motor3508_data[0].relative_raw_speed,gimbal_motor3508_data[1].relative_raw_speed,-gimbal_motor3508_data[2].relative_raw_speed,FricSpeed,shoot_data.infact_shoot_speed);
					//plot3(trigger_motor3508_data[0].target_angle,trigger_motor3508_data[0].relative_raw_angle,shoot_data.infact_shoot_speed);
          //plot2(chassis_motor3508_data[0].relative_raw_speed, 0);  
					//plot4(INS_eulers[0],gimbal_data.gyro_pit_target_angle,INS_eulers[1],gimbal_data.gyro_yaw_target_angle);
        send_num = 0;
				}
        send_num++;

        vTaskDelay(1);

    }

}


//底盘初始化函数
void fn_ChassisInit(void){
    chassis_mode_time = 500;

    spin_w_count = 0;
    spin_w_flag = 0;
    relative_Pmax = 0.0f;

    chassis_move_data.chassis_mode = chassis_down;
    chassis_move_data.vx_set = 0.0f;
	chassis_move_data.vy_set = 0.0f;
	chassis_move_data.w_set = 0.0f;
    chassis_move_data.chassis_relative_angle_set = 0.0f;

    w_match = SpinW;
    vx_match = KeycontrolV;
    vy_match = KeycontrolV;

    chassis_move_data.vx_max_speed = VxMaxSpeed;
	chassis_move_data.vx_min_speed = VxMinSpeed;
	chassis_move_data.vy_max_speed = VyMaxSpeed;
	chassis_move_data.vy_min_speed = VyMinSpeed;
    
    //底盘跟随PID参数
    fp32 af_chassis_follow_pid[3] = {chassis_follow_pid_kp,chassis_follow_pid_ki,chassis_follow_pid_kd};
    fn_PidInit(&chassis_move_data.motor_pid1,af_chassis_follow_pid,chassis_follow_min_out,chassis_follow_max_out,chassis_follow_min_iout,chassis_follow_max_iout);
}


//底盘电机初始化函数
void fn_ChassisMotorInit(void){
    
	//底盘3508电机初始化
    //fp32 af_ChassisMotor3508PosPid1[4][3] = {{0.0f,0.0f,0.0f},
	//                                         {0.0f,0.0f,0.0f},
	//									       {0.0f,0.0f,0.0f},
	//									       {0.0f,0.0f,0.0f}};

    //fp32 af_ChassisMotor3508PosPid2[4][3] = {{0.0f,0.0f,0.0f},
	//                                         {0.0f,0.0f,0.0f},
	//								     	   {0.0f,0.0f,0.0f},
	//								       	   {0.0f,0.0f,0.0f}};
    
    //底盘电机PID参数
    fp32 af_ChassisMotor3508PosPid3[4][3] = {{ChassisMotor3508PID3_ID201_kp,ChassisMotor3508PID3_ID201_ki,ChassisMotor3508PID3_ID201_kd},
	                                         {ChassisMotor3508PID3_ID202_kp,ChassisMotor3508PID3_ID202_ki,ChassisMotor3508PID3_ID202_kd},
										     {ChassisMotor3508PID3_ID203_kp,ChassisMotor3508PID3_ID203_ki,ChassisMotor3508PID3_ID203_kd},
										     {ChassisMotor3508PID3_ID204_kp,ChassisMotor3508PID3_ID204_ki,ChassisMotor3508PID3_ID204_kd}};
    for(uint8_t i = 0;i < 4;i++){

        chassis_motor3508_data[i].round_num = 0;       
        chassis_motor3508_data[i].relative_raw_angle = 0.0f;

		for(uint8_t m = 0;m < 6;m++){
            chassis_motor3508_data[i].raw_angle[m] = 0.0f;
			chassis_motor3508_data[i].raw_speed[m] = 0.0f; 
		}
		for(uint8_t n = 0;n < 2;n++){
			chassis_motor3508_data[i].filter_angle[n] = 0.0f;
			chassis_motor3508_data[i].filter_speed[n] = 0.0f;
		}

        chassis_motor3508_data[i].relative_raw_speed = 0.0f;                 
        chassis_motor3508_data[i].offecd_ecd = 0;   
	    chassis_motor3508_data[i].target_angle = 0.0f;
		chassis_motor3508_data[i].target_speed = 0.0f;          
        chassis_motor3508_data[i].given_current = 0.0f;           
        chassis_motor3508_data[i].double_pid_mid = 0.0f;

		//fn_PidInit(&chassis_motor3508_data[i].motor_pid1,af_ChassisMotor3508PosPid1[i],-4000.0f,4000.0f,-500.0f,500.0f);
		//fn_PidInit(&chassis_motor3508_data[i].motor_pid2,af_ChassisMotor3508PosPid2[i],-4000.0f,4000.0f,-500.0f,500.0f);
        fn_PidInit(&chassis_motor3508_data[i].motor_pid3,af_ChassisMotor3508PosPid3[i], chassis_motor3508_min_out,chassis_motor3508_max_out,chassis_motor3508_min_iout,chassis_motor3508_max_iout);

	}

}



//底盘模式选择
void fn_ChassisMode(void){
    
    //遥控器模式
    if(!IF_RC_SW2_MID){
        if(IF_RC_SW2_UP && gimbal_data.gimbal_behaviour != GIMBAL_INIT){
            chassis_move_data.chassis_mode = chassis_follow;
        }
        if(IF_RC_SW2_UP && ctl.rc.k0 >= 1500 && gimbal_data.gimbal_behaviour != GIMBAL_INIT){
            chassis_move_data.chassis_mode = chassis_spin;
        }
    }


    //键鼠模式
    if(IF_RC_SW2_MID){
        if(chassis_mode_time > 0){
            chassis_mode_time --;
        }
        //不在小陀螺和打符模式时底盘进入跟随模式
        if(!IF_KEY_PRESSED_SHIFT && chassis_move_data.chassis_mode != chassis_not_follow && gimbal_data.gimbal_behaviour != GIMBAL_INIT){
            chassis_move_data.chassis_mode = chassis_follow;
        }
        //按下B进入吊射模式则固定底盘掉底盘
        if(IF_KEY_PRESSED_CTRL && gimbal_data.gimbal_behaviour != GIMBAL_INIT && chassis_move_data.chassis_mode != chassis_not_follow && chassis_mode_time == 0){
            chassis_move_data.chassis_mode = chassis_not_follow;
            chassis_mode_time = 500;
        }
        if(IF_KEY_PRESSED_CTRL && gimbal_data.gimbal_behaviour != GIMBAL_INIT && chassis_move_data.chassis_mode == chassis_not_follow && chassis_mode_time == 0){
            chassis_move_data.chassis_mode = chassis_follow;
            chassis_mode_time = 500;
        }
        //SHIFT放在后面保证在任何模式下按下SHIFT后都是小陀螺模式
        if(IF_KEY_PRESSED_SHIFT && gimbal_data.gimbal_behaviour != GIMBAL_INIT){
            chassis_move_data.chassis_mode = chassis_spin;
        }
        //如果在观测中退出了小陀螺模式，则清除观测数据，下次重新开始观测
        if(chassis_move_data.chassis_mode != chassis_spin && spin_w_flag == 1){
            spin_w_flag = 0;
            relative_Pmax = 0.0f;
        }
    }

    if(IF_RC_SW2_DOWN || gimbal_data.gimbal_behaviour == GIMBAL_INIT){
        chassis_move_data.chassis_mode = chassis_down;
    }
}

//底盘电流解算
void fn_ChassisMove(void){
    
    //down
    if(chassis_move_data.chassis_mode == chassis_down){
        for(uint8_t i = 0;i < 4;i++){
            chassis_motor3508_data[i].given_current = 0.0f;
        }
				
				//确定当前的正前方为0°或者180°
        if(-PI/2 < gimbal_motor4310_data[0].position && gimbal_motor4310_data[0].position < PI/2){
            chassis_move_data.chassis_relative_angle_set = 0.0f;
        }
        else{
            chassis_move_data.chassis_relative_angle_set = PI;
        }
    }

    //底盘跟随模式
    if(chassis_move_data.chassis_mode == chassis_follow){
        //遥控器模式
        if(!IF_RC_SW2_MID){
            chassis_move_data.vx_set = (float)(ctl.rc.ch1 - 1024) / 660.0f * RCcontrolMaxV;
            chassis_move_data.vy_set = (float)(ctl.rc.ch0 - 1024) / 660.0f * RCcontrolMaxV;
        }
        //键鼠模式
        if(IF_RC_SW2_MID){
            if(!IF_KEY_PRESSED_C){
                static uint16_t match_T = 0;
                static uint16_t speed_stable_t = 0;
                //拟合角速度速度
                if(fn_chassis_speed_reset_speed(infact_Pmax) || vx_match < 125.0f){
                    vx_match = KeycontrolV;
                    vy_match = KeycontrolV;
                    speed_match_flag = 0;
                    match_T = 0;
                }
                //速度稳定后再拟合
                if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S){
                    if(speed_stable_t < 2500){
                        speed_stable_t ++;
                    }
                }
                else{
                    speed_stable_t = 0;
                }
                if(!speed_match_flag && match_T == 200 && !(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D) && speed_stable_t == 2500){
                    fn_chassis_speed_autoset(&w_match,&vx_match,&vy_match,2);
		        	match_T = 0;
                }
		        if(match_T < 200 && !(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D)){
		        	match_T++;
		        }
                if(!(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S)){
                    chassis_move_data.vx_set = 0.0f;
                }
                if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S){
                    if(IF_KEY_PRESSED_W){
                        chassis_move_data.vx_set = vx_match;
                    }
                    if(IF_KEY_PRESSED_S){
                        chassis_move_data.vx_set = -vx_match;
                    }
                }
                if(!(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D)){
                    chassis_move_data.vy_set = 0.0f;
                }
                if(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D){
                    if(IF_KEY_PRESSED_A){
                        chassis_move_data.vy_set = -vx_match;
                    }
                    if(IF_KEY_PRESSED_D){
                        chassis_move_data.vy_set = vx_match;
                    }
                }
            }
            else{
                if(!(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S)){
                    chassis_move_data.vx_set = 0.0f;
                }
                if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S){
                    if(IF_KEY_PRESSED_W){
                        chassis_move_data.vx_set = KeycontrolCapV;
                    }
                    if(IF_KEY_PRESSED_S){
                        chassis_move_data.vx_set = -KeycontrolCapV;
                    }
                }
                if(!(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D)){
                    chassis_move_data.vy_set = 0.0f;
                }
                if(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D){
                    if(IF_KEY_PRESSED_A){
                        chassis_move_data.vy_set = -KeycontrolCapV;
                    }
                    if(IF_KEY_PRESSED_D){
                        chassis_move_data.vy_set = KeycontrolCapV;
                    }
                }
            }
        }
        //底盘速度一阶低通滤波 目的是平滑速度
        if(!IF_KEY_PRESSED_C){
            fn_low_filter(&chassis_move_data.vx_filter_set,chassis_move_data.vx_set,1.0f);
            fn_low_filter(&chassis_move_data.vy_filter_set,chassis_move_data.vy_set,1.0f);
        }
        else{
			fn_low_filter(&chassis_move_data.vx_filter_set,chassis_move_data.vx_set,0.006f);
            fn_low_filter(&chassis_move_data.vy_filter_set,chassis_move_data.vy_set,0.006f);
        }

        //底盘速度限制
        fn_Fp32Limit(&chassis_move_data.vx_filter_set,chassis_move_data.vx_min_speed,chassis_move_data.vx_max_speed);
        fn_Fp32Limit(&chassis_move_data.vy_filter_set,chassis_move_data.vy_min_speed,chassis_move_data.vy_max_speed);
        fn_Fp32Limit(&chassis_move_data.vx_filter_set,chassis_move_data.vx_min_speed,chassis_move_data.vx_max_speed);
        fn_Fp32Limit(&chassis_move_data.vy_filter_set,chassis_move_data.vy_min_speed,chassis_move_data.vy_max_speed);

        if(fabs(chassis_move_data.vx_filter_set) < 1.0e-6f || fabs(chassis_move_data.vy_filter_set) < 1.0e-6f){
            if(fabs(chassis_move_data.vx_filter_set) < 1.0e-6f){
                chassis_move_data.vx_filter_set = 0.0f;
            }
            if(fabs(chassis_move_data.vy_filter_set) < 1.0e-6f){
                chassis_move_data.vy_filter_set = 0.0f;
            }
        }
        
        //底盘跟随角速度
        chassis_move_data.w_set = -fn_PidClacAngle(&chassis_move_data.motor_pid1,gimbal_motor4310_data[0].position,chassis_move_data.chassis_relative_angle_set);
        //中心死区
        if(-ZeroW < chassis_move_data.w_set && chassis_move_data.w_set < ZeroW){
            chassis_move_data.w_set = 0.0f;
        }

        //底盘电机速度解算
        chassis_motor3508_data[0].target_speed = fn_WheelSpeedW1(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[1].target_speed = fn_WheelSpeedW2(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[2].target_speed = fn_WheelSpeedW3(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[3].target_speed = fn_WheelSpeedW4(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);

        //电机速度限制
        if(fn_scope_judgment(chassis_motor3508_data[0].target_speed,-AxleWMax,AxleWMax) || fn_scope_judgment(chassis_motor3508_data[1].target_speed,-AxleWMax,AxleWMax)
        || fn_scope_judgment(chassis_motor3508_data[2].target_speed,-AxleWMax,AxleWMax) || fn_scope_judgment(chassis_motor3508_data[3].target_speed,-AxleWMax,AxleWMax)){
            fp32 k = 1.0f;
            fp32 max_speed = 0.0f;
            for(uint8_t i = 0;i < 4;i++){
                if(max_speed < fabs(chassis_motor3508_data[i].target_speed)){
                    max_speed = fabs(chassis_motor3508_data[i].target_speed);
                }
            }
            if(max_speed >= AxleWMax){
                k = AxleWMax / max_speed;
            }
            for(uint8_t i = 0;i < 4;i++){
                chassis_motor3508_data[i].target_speed = k * chassis_motor3508_data[i].target_speed;
            }
        }

        //底盘电机电流计算
        for(uint8_t i = 0;i < 4;i++){
            chassis_motor3508_data[i].given_current = fn_Iclear_PidClac(&chassis_motor3508_data[i].motor_pid3,chassis_motor3508_data[i].relative_raw_speed,chassis_motor3508_data[i].target_speed);
        }

        //功率限制
        //电容开启功率控制策略
        if(cap_FSM == cap_auto_charge){
            if(!IF_KEY_PRESSED_C){
                infact_Pmax = ext_robot_status.chassis_power_limit;
            }
            else{
                if(cap_data.Capacity >= 18 && low_vol_flag == 1){
                    infact_Pmax = CAP_MAX_POWER;
                }
                else if(cap_data.Capacity >= 6 && low_vol_flag == 1){
                    infact_Pmax = cap_data.Capacity / 18 * CAP_MAX_POWER;
                }
                else{
                    infact_Pmax = ext_robot_status.chassis_power_limit;
                }
            }
        }
        //电容未开
        else{
            infact_Pmax = ext_robot_status.chassis_power_limit;
        }
        count_Pmax = fn_level_power_limit(infact_Pmax,ext_power_heat_data.buffer_energy);
        fn_low_filter(&filter_Pmax,count_Pmax,0.15f);
        fn_chassis_power_control(&chassis_motor3508_data[0],&chassis_motor3508_data[1],&chassis_motor3508_data[2],&chassis_motor3508_data[3],filter_Pmax);
        
        //确定当前的正前方为0°或者180°
        if(-PI/2 < gimbal_motor4310_data[0].position && gimbal_motor4310_data[0].position < PI/2){
            chassis_move_data.chassis_relative_angle_set = 0.0f;
        }
        else{
            chassis_move_data.chassis_relative_angle_set = PI;
        }
    }

    //底盘小陀螺模式
    if(chassis_move_data.chassis_mode == chassis_spin){
        //小陀螺角速度确定
//        static fp32 sum_w = 0;
//		if((chassis_move_data.vx_filter_set != 0.0f || chassis_move_data.vy_filter_set != 0.0f) && spin_w_flag){
//			spin_w_flag = 0;
//            relative_Pmax = 0.0f;
//		}
//        if(spin_w_flag == 0 && (relative_Pmax + infact_Pmax) / 2 != relative_Pmax 
//					 && chassis_move_data.vx_filter_set == 0.0f && chassis_move_data.vy_filter_set == 0.0f){
//            spin_w_flag = 1;
//            relative_Pmax = infact_Pmax;
//            sum_w = 0;
//            spin_w_count = 0;
//        }
//        if(spin_w_flag == 1 && spin_w_count < 3000){
//            spin_w_count++;
//        }

//        //2s加速等待
//        if(spin_w_count < 2000){
//            chassis_move_data.w_set = SpinW;
//        }
//        //加速2s后记录1s观测数据
//        else if(spin_w_count < 3000){
//            sum_w += gimbal_motor4310_data[0].velocity - INS_gyro[2];
//					chassis_move_data.w_set = SpinW;
//        }
//        //观测完毕
//        else{
//            if(-sum_w / 1000 < SpinW){
//				if(1){
//                    chassis_move_data.w_set = -sum_w / 1000 - 0.15f;
//				}
//				else{
//					chassis_move_data.w_set = -(-sum_w / 1000 - 0.15f);
//				}
//            }
//            else{
//                if(1){
//                    chassis_move_data.w_set = SpinW;
//				}
//				else{
//					chassis_move_data.w_set = -SpinW;
//				}
//            }
//            spin_w_flag = 0;
//        }
        static uint16_t match_T = 0;
        //拟合角速度速度
        if(fn_chassis_speed_reset_w(infact_Pmax)){
            w_match = SpinW;
            w_match_flag = 0;
            match_T = 0;
        }
        if(!w_match_flag && match_T == ((uint16_t)(40 + infact_Pmax * 2)) && !(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D || IF_KEY_PRESSED_W || IF_KEY_PRESSED_S)){
            fn_chassis_speed_autoset(&w_match,&vx_match,&vy_match,1);
			match_T = 0;
        }
		if(match_T < ((uint16_t)(40 + infact_Pmax * 2)) && !(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D || IF_KEY_PRESSED_W || IF_KEY_PRESSED_S)){
			match_T++;
		}
        
        //赋值角速度
        chassis_move_data.w_set = w_match;

        //平移速度
        if(IF_RC_SW2_UP){
            chassis_move_data.vx_set = (float)(ctl.rc.ch1 - 1024) / 660.0f * SpinV;
            chassis_move_data.vy_set = (float)(ctl.rc.ch0 - 1024) / 660.0f * SpinV;
        }
        if(IF_RC_SW2_MID){
            if(infact_Pmax <= 150.0f){
                if(!(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D)){
                    chassis_move_data.vx_set = 0.0f;
                    chassis_move_data.vy_set = 0.0f;
                }
                if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D){
                    fp32 vx_w,vx_s,vx_a,vx_d;
                    fp32 vy_w,vy_s,vy_a,vy_d;
                    if(IF_KEY_PRESSED_W){
                        vx_w = SpinV * chassis_move_data.w_set / SpinW;
                        //vy_w = 0.8f * SpinV * chassis_move_data.w_set / SpinW;
                        vy_w = 0.0f;
                    }
                    else{
                        vx_w = 0.0f;
                        vy_w = 0.0f;
                    }
                    if(IF_KEY_PRESSED_S){
                        vx_s = -SpinV * chassis_move_data.w_set / SpinW;
                        //vy_s = -0.8f *SpinV * chassis_move_data.w_set / SpinW;
                        vy_s = 0.0f;
                    }
                    else{
                        vx_s = 0.0f;
                        vy_s = 0.0f;
                    }
                    if(IF_KEY_PRESSED_A){
                        //vx_a = 0.8f * SpinV * chassis_move_data.w_set / SpinW;
                        vx_a = 0.0f;
                        vy_a = -SpinV * chassis_move_data.w_set / SpinW;
                    }
                    else{
                        vx_a = 0.0f;
                        vy_a = 0.0f;
                    }
                    if(IF_KEY_PRESSED_D){
                        //vx_d = -0.8f * SpinV * chassis_move_data.w_set / SpinW;
                        vx_d = 0.0f;
                        vy_d = SpinV * chassis_move_data.w_set / SpinW;
                    }
                    else{
                        vx_d = 0.0f;
                        vy_d = 0.0f;
                    }
                    chassis_move_data.vx_set = vx_w + vx_s + vx_a + vx_d;
                    fn_Fp32Limit(&chassis_move_data.vx_set,-SpinV,SpinV);
                    chassis_move_data.vy_set = vy_w + vy_s + vy_a + vy_d;
                    fn_Fp32Limit(&chassis_move_data.vy_set,-SpinV,SpinV);
                }
            }
            else{
                if(!(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S)){
                    chassis_move_data.vx_set = 0.0f;
                }
                if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S){
                    if(IF_KEY_PRESSED_W){
                        chassis_move_data.vx_set = SpinV;
                    }
                    if(IF_KEY_PRESSED_S){
                        chassis_move_data.vx_set = -SpinV;
                    }
                }
                if(!(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D)){
                    chassis_move_data.vy_set = 0.0f;
                }
                if(IF_KEY_PRESSED_A || IF_KEY_PRESSED_D){
                    if(IF_KEY_PRESSED_A){
                        chassis_move_data.vy_set = -SpinV;
                    }
                    if(IF_KEY_PRESSED_D){
                        chassis_move_data.vy_set = SpinV;
                    }
                }
            }
        }

        //底盘速度一阶低通滤波 目的是平滑速度
        fn_low_filter(&chassis_move_data.vx_filter_set,chassis_move_data.vx_set,0.2f);
        fn_low_filter(&chassis_move_data.vy_filter_set,chassis_move_data.vy_set,0.2f);

        //底盘速度限制
        fn_Fp32Limit(&chassis_move_data.vx_set,chassis_move_data.vx_min_speed,chassis_move_data.vx_max_speed);
        fn_Fp32Limit(&chassis_move_data.vy_set,chassis_move_data.vy_min_speed,chassis_move_data.vy_max_speed);
        fn_Fp32Limit(&chassis_move_data.vx_filter_set,chassis_move_data.vx_min_speed,chassis_move_data.vx_max_speed);
        fn_Fp32Limit(&chassis_move_data.vy_filter_set,chassis_move_data.vy_min_speed,chassis_move_data.vy_max_speed);

        if(fabs(chassis_move_data.vx_filter_set) < 1.0e-6f || fabs(chassis_move_data.vy_filter_set) < 1.0e-6f){
            if(fabs(chassis_move_data.vx_filter_set) < 1.0e-6f){
                chassis_move_data.vx_filter_set = 0.0f;
            }
            if(fabs(chassis_move_data.vy_filter_set) < 1.0e-6f){
                chassis_move_data.vy_filter_set = 0.0f;
            }
        }

        //底盘电机速度解算
        chassis_motor3508_data[0].target_speed = fn_WheelSpeedW1(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[1].target_speed = fn_WheelSpeedW2(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[2].target_speed = fn_WheelSpeedW3(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[3].target_speed = fn_WheelSpeedW4(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        
        //电机速度限制
        if(fn_scope_judgment(chassis_motor3508_data[0].target_speed,-AxleWMax,AxleWMax) || fn_scope_judgment(chassis_motor3508_data[1].target_speed,-AxleWMax,AxleWMax)
        || fn_scope_judgment(chassis_motor3508_data[2].target_speed,-AxleWMax,AxleWMax) || fn_scope_judgment(chassis_motor3508_data[3].target_speed,-AxleWMax,AxleWMax)){
            fp32 k = 1.0f;
            fp32 max_speed = 0.0f;
            for(uint8_t i = 0;i < 4;i++){
                if(max_speed < fabs(chassis_motor3508_data[i].target_speed)){
                    max_speed = fabs(chassis_motor3508_data[i].target_speed);
                }
            }
            if(max_speed >= AxleWMax){
                k = AxleWMax / max_speed;
            }
            for(uint8_t i = 0;i < 4;i++){
                chassis_motor3508_data[i].target_speed = k * chassis_motor3508_data[i].target_speed;
            }
        }

        //底盘电机电流计算
        for(uint8_t i = 0;i < 4;i++){
            chassis_motor3508_data[i].given_current = fn_PidClac(&chassis_motor3508_data[i].motor_pid3,chassis_motor3508_data[i].relative_raw_speed,chassis_motor3508_data[i].target_speed);
        }

        //功率限制
        //电容开启功率控制策略
        if(cap_FSM == cap_auto_charge){
            if(!IF_KEY_PRESSED_C){
                infact_Pmax = ext_robot_status.chassis_power_limit;
            }
            else{
                if(cap_data.Capacity >= 6 && low_vol_flag == 1){
                    infact_Pmax = 150.0f;
                }
                else{
                    infact_Pmax = ext_robot_status.chassis_power_limit;
                }
            }
        }
        //电容未开
        else{
            infact_Pmax = ext_robot_status.chassis_power_limit;
        }
        count_Pmax = fn_level_power_limit(infact_Pmax,ext_power_heat_data.buffer_energy);
        fn_low_filter(&filter_Pmax,count_Pmax,0.15f);
        fn_chassis_power_control(&chassis_motor3508_data[0],&chassis_motor3508_data[1],&chassis_motor3508_data[2],&chassis_motor3508_data[3],filter_Pmax);
        

        //确定当前的正前方为0°或者180°
        if(-PI/2 < gimbal_motor4310_data[0].position && gimbal_motor4310_data[0].position < PI/2){
            chassis_move_data.chassis_relative_angle_set = 0.0f;
        }
        else{
            chassis_move_data.chassis_relative_angle_set = PI;
        }
    }

    //吊射模式
    if(chassis_move_data.chassis_mode == chassis_not_follow){
        chassis_move_data.vx_filter_set = 0.0f;
        chassis_move_data.vy_filter_set = 0.0f;
        chassis_move_data.w_set = 0.0f;

        //底盘电机速度解算
        chassis_motor3508_data[0].target_speed = fn_WheelSpeedW1(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[1].target_speed = fn_WheelSpeedW2(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[2].target_speed = fn_WheelSpeedW3(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);
        chassis_motor3508_data[3].target_speed = fn_WheelSpeedW4(chassis_move_data.vx_filter_set,chassis_move_data.vy_filter_set,chassis_move_data.w_set,gimbal_motor4310_data[0].position);

        //底盘电机电流计算
        for(uint8_t i = 0;i < 4;i++){
            chassis_motor3508_data[i].given_current = fn_PidClac(&chassis_motor3508_data[i].motor_pid3,chassis_motor3508_data[i].relative_raw_speed,chassis_motor3508_data[i].target_speed);
        }
    }
}
