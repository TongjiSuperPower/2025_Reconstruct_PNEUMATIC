#include "led_task.h"
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "calibrate_task.h"
#include "detect_task.h"
#include "can_task.h"

uint16_t blue,green,red;

void fn_LedInit(void);
void fn_LedBlue(void);
void fn_LedGreen(void);
void fn_LedRed(void);



void LED_Task(void const * argument){

    vTaskDelay(200);  
	fn_LedInit();

    while(1){
        //如果遥控器数据错误，则一直亮红灯
        //if(remote_control_data_error_flag == 1){
        if(can2_flag == 0x01U){
            fn_LedRed();
        }
        //如果遥控器没有正常收到值，则一直亮蓝灯
        //else if(rc_receive_flag == 1){
        else if(can1_flag == 0x01U){
            fn_LedGreen();
        }
        else{
            fn_LedBlue();
            fn_LedGreen();
            fn_LedRed();
        }
    }
}



//LED灯初始化函数
void fn_LedInit(void){

    blue = 0;
    green= 0;
    red = 0;
    __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,0);
    __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0);
    __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0);

}

//蓝色灯2s周期呼吸
void fn_LedBlue(void){
    
    while(blue < 1000){
        blue++;
        __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,blue);
        vTaskDelay(1);
    }

    while(blue > 0){
        blue--;
        __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,blue);
        vTaskDelay(1);
    }

}

//绿色灯2s周期呼吸
void fn_LedGreen(void){
    
    while(green < 1000){
        green++;
        __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,green);
        vTaskDelay(1);
    }

    while(green > 0){
        green--;
        __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,green);
        vTaskDelay(1);
    }

}

//红色灯2s周期呼吸
void fn_LedRed(void){
    while(red < 1000){
        red++;
        __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,red);
        vTaskDelay(1);
    }

    while(red > 0){
        red--;
        __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,red);
        vTaskDelay(1);
    }
}
