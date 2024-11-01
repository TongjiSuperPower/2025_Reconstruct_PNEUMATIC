#ifndef _DETECT_TASK_H_
#define _DETECT_TASK_H_

#include "struct_typedef.h"



extern uint8_t remote_control_data_error_flag;
extern uint8_t rollover_flag;
extern uint8_t trigger_motor3508_block_flag;
extern uint8_t rc_receive_flag;

//初始化函数,初始化为无错状态
void fn_ErrorFlagInit(void);

//遥控器数据校验
bool_t fn_RemoteControlDataErrorTest(void);

//翻车检测 翻车返回1 正常返回0
uint8_t fn_RolloverTest(void);

#endif
