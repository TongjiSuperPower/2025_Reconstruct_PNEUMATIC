#include "telescope_task.h"
#include "struct_typedef.h"
#include "pid.h"
#include "math_lib.h"
#include "cmsis_os.h"
#include "can_task.h"
#include "chassis_task.h"

void fn_TelescopeInit(void);
void fn_TelescopeMode(void);

void Telescope_Task(void const * argument){
    
    vTaskDelay(200);
    fn_TelescopeInit();

    while(1){
        fn_TelescopeMode();
        vTaskDelay(1);
    }
}

void fn_TelescopeInit(void){
    //2006电机初始化
    fp32 af_GimbalMotor2006PosPid1[2][3] = {{GimbalMotor2006PID1_ID205_kp,GimbalMotor2006PID1_ID205_ki,GimbalMotor2006PID1_ID205_kd},
	                                        {GimbalMotor2006PID1_ID206_kp,GimbalMotor2006PID1_ID206_ki,GimbalMotor2006PID1_ID206_kd}};
    fp32 af_GimbalMotor2006PosPid2[2][3] = {{GimbalMotor2006PID2_ID205_kp,GimbalMotor2006PID2_ID205_ki,GimbalMotor2006PID2_ID205_kd},
	                                        {GimbalMotor2006PID2_ID206_kp,GimbalMotor2006PID2_ID206_ki,GimbalMotor2006PID2_ID206_kd}};
    for(uint8_t i = 0;i < 2;i++){

        gimbal_motor2006_data[i].round_num = 0;       
        gimbal_motor2006_data[i].relative_raw_angle = 0.0f;

		for(uint8_t m = 0;m < 6;m++){
            gimbal_motor2006_data[i].raw_angle[m] = 0.0f;
			gimbal_motor2006_data[i].raw_speed[m] = 0.0f; 
		}
		for(uint8_t n = 0;n < 2;n++){
			gimbal_motor2006_data[i].filter_angle[n] = 0.0f;
			gimbal_motor2006_data[i].filter_speed[n] = 0.0f;
		}

        gimbal_motor2006_data[i].relative_raw_speed = 0.0f;                 
        gimbal_motor2006_data[i].offecd_ecd = 0;   
	    gimbal_motor2006_data[i].target_angle = 0.0f;
		gimbal_motor2006_data[i].target_speed = 0.0f;          
        gimbal_motor2006_data[i].given_current = 0.0f;           
        gimbal_motor2006_data[i].double_pid_mid = 0.0f;

		//fn_PidInit(&gimbal_motor2006_data[i].motor_pid1,af_GimbalMotor2006PosPid1[i],-4000.0f,4000.0f,-500.0f,500.0f);
		//fn_PidInit(&gimbal_motor2006_data[i].motor_pid2,af_GimbalMotor2006PosPid2[i],-4000.0f,4000.0f,-500.0f,500.0f);
        fn_PidInit(&gimbal_motor2006_data[i].motor_pid1,af_GimbalMotor2006PosPid1[i], gimbal_motor2006_min_out,gimbal_motor2006_max_out,gimbal_motor2006_min_iout,gimbal_motor2006_max_iout);
        fn_PidInit(&gimbal_motor2006_data[i].motor_pid2,af_GimbalMotor2006PosPid2[i], gimbal_motor2006_min_out,gimbal_motor2006_max_out,gimbal_motor2006_min_iout,gimbal_motor2006_max_iout);
    }
}

void fn_TelescopeMode(void){
    if(chassis_move_data.chassis_mode == chassis_not_follow){
        gimbal_motor2006_data[0].target_angle = video_angle_on;
        gimbal_motor2006_data[1].target_angle = telescope_angle_on;
    }
    else{
        gimbal_motor2006_data[0].target_angle = video_angle_off;
        gimbal_motor2006_data[1].target_angle = telescope_angle_off;
    }

    for(uint8_t i = 0;i < 2;i ++){
        gimbal_motor2006_data[i].double_pid_mid = fn_PidClacAngle(&gimbal_motor2006_data[i].motor_pid1,gimbal_motor2006_data[i].relative_raw_angle,gimbal_motor2006_data[i].target_angle);
        gimbal_motor2006_data[i].given_current = fn_PidClac(&gimbal_motor2006_data[i].motor_pid1,gimbal_motor2006_data[i].relative_raw_speed,gimbal_motor2006_data[i].double_pid_mid);
    }
}
