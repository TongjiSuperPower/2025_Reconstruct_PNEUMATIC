#include "detect_task.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "can_task.h"
#include "ins_task.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_hal_cortex.h"

uint8_t Reset_cont;

//遥控器数据错误标志 1为出错 0为正常
uint8_t remote_control_data_error_flag;
//翻车标志 1为翻车 0为正常
uint8_t rollover_flag;
//拨弹轮电机堵堵转标志 1为堵转 0为正常
uint8_t trigger_motor3508_block_flag;
//遥控器数据接收状态检测 1为未正常收到值 0为正常
uint8_t rc_receive_flag;

void fn_ErrorFlagInit(void);

bool_t fn_RemoteControlDataErrorTest(void);

uint8_t fn_RolloverTest(void);

uint8_t fn_trigger_motor_block(void);

uint8_t fn_rc_work_state_test(void);



void Detect_Task(void const * argument){

    //等待一段时间
    vTaskDelay(100);

    //初始化为无错状态
    fn_ErrorFlagInit();

    //正常化后延时 确保检测时所有数据均为正常数据
    vTaskDelay(100);

    while(1){
        
        //遥控器数据是否出错
        remote_control_data_error_flag = fn_RemoteControlDataErrorTest();

        //翻车检测
        rollover_flag = fn_RolloverTest();

        //拨弹轮赌转检测
        trigger_motor3508_block_flag = fn_trigger_motor_block();

        //遥控器状态检测
        rc_receive_flag = fn_rc_work_state_test();
        
        if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_V && Reset_cont > 0){
            Reset_cont--;
        }
        else{
            Reset_cont = 50;
        }
        if(Reset_cont == 0){
            HAL_NVIC_SystemReset();
        }

        vTaskDelay(20);

    }
}


//初始化函数,初始化为无错状态
void fn_ErrorFlagInit(void){
    remote_control_data_error_flag = 0;
    rollover_flag = 0;
    trigger_motor3508_block_flag = 0;
    Reset_cont = 50;
}


//遥控器数据校验,数据正常则返回0，异常则返回1
uint8_t fn_RemoteControlDataErrorTest(void){
    
    uint8_t ErrorNum = 0;

    if(abs(ctl.rc.ch0 - RC_CH_VALUE_OFFSET) > 660 
      || abs(ctl.rc.ch1 - RC_CH_VALUE_OFFSET) > 660 
      || abs(ctl.rc.ch2 - RC_CH_VALUE_OFFSET) > 660 
      || abs(ctl.rc.ch3 - RC_CH_VALUE_OFFSET) > 660){
        //ctl.rc.ch0 = RC_CH_VALUE_OFFSET;
        //ctl.rc.ch1 = RC_CH_VALUE_OFFSET;
        //ctl.rc.ch2 = RC_CH_VALUE_OFFSET;
        //ctl.rc.ch3 = RC_CH_VALUE_OFFSET;
        ErrorNum++;
    }

    if((ctl.rc.s1 != RC_SW_UP) && (ctl.rc.s1 != RC_SW_MID) && (ctl.rc.s1 != RC_SW_DOWN)){
        //ctl.rc.s1 = RC_SW_DOWN;
        ErrorNum++;
    }

    if((ctl.rc.s2 != RC_SW_UP) && (ctl.rc.s2 != RC_SW_MID) && (ctl.rc.s2 != RC_SW_DOWN)){
        //ctl.rc.s2 = RC_SW_MID;
        ErrorNum++;
    }

    if(MOUSE_X_MOVE_SPEED < -32768 || MOUSE_X_MOVE_SPEED > 32767){
        //MOUSE_X_MOVE_SPEED = 0;
        ErrorNum++;
    }

    if(MOUSE_Y_MOVE_SPEED < -32768 || MOUSE_Y_MOVE_SPEED > 32767){
        //MOUSE_Y_MOVE_SPEED = 0;
        ErrorNum++;
    }

    if(MOUSE_Z_MOVE_SPEED < -32768 || MOUSE_Z_MOVE_SPEED > 32767){
        //MOUSE_Z_MOVE_SPEED = 0;
        ErrorNum++;
    }

    if(ErrorNum == 0){
        return 0;
    }

    return 1;

}

//翻车检测 翻车返回1 正常返回0
uint8_t fn_RolloverTest(void){
    uint8_t count = 0;
    static uint16_t rollover_num = 0;
    for(uint8_t i = 0;i < 6;i++){
        if(INS_accel_z[i] < 2.0f){
            count++;
        }
    }
    if(count == 6){
        if(rollover_num < 100){
            rollover_num++;
        }
    }
    else{
        rollover_num = 0;
    }

    if(rollover_num == 100){
        return 1;
    }
    return 0;
}

//拨弹轮堵转检测
uint8_t fn_trigger_motor_block(void){
    static uint16_t trigger_num = 0;
    if(!trigger_motor3508_block_flag){
			if(abs(trigger_motor3508_measure[0].given_current) > 6000 && 
           fabs((trigger_motor3508_data[0].filter_speed[0] + trigger_motor3508_data[0].filter_speed[1]) / 2.0f) < 0.5f){
            trigger_num++;
        }
        else{
            trigger_num = 0;
        }
    }
    //判断堵转
    if(trigger_num > 39){
        trigger_num = 0;
        return 1;
    }
    return 0;
}

//遥控器工作状态检测
uint8_t fn_rc_work_state_test(void){
    static uint16_t rc_num = 0;
    static uint16_t last_rc_receive_num = 0;

    if(rc_receive_num == last_rc_receive_num){
        if(rc_num < 100){
            rc_num++;
        }
    }
    else{
        rc_num = 0;
    }

    last_rc_receive_num = rc_receive_num;

    if(rc_num == 100){
        return 1;
    }
    else{
        return 0;
    }
}
