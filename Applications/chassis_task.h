#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "struct_typedef.h"
#include "pid.h"
#include "gimbal_task.h"


//底盘3508电机PID控制参数
//单速度环参数
#define ChassisMotor3508PID3_ID201_kp 800.0f
#define ChassisMotor3508PID3_ID201_ki 4.0f
#define ChassisMotor3508PID3_ID201_kd 100.0f

#define ChassisMotor3508PID3_ID202_kp 800.0f
#define ChassisMotor3508PID3_ID202_ki 4.0f
#define ChassisMotor3508PID3_ID202_kd 100.0f

#define ChassisMotor3508PID3_ID203_kp 800.0f
#define ChassisMotor3508PID3_ID203_ki 4.0f
#define ChassisMotor3508PID3_ID203_kd 100.0f

#define ChassisMotor3508PID3_ID204_kp 800.0f
#define ChassisMotor3508PID3_ID204_ki 4.0f
#define ChassisMotor3508PID3_ID204_kd 100.0f

#define chassis_motor3508_min_out -16000.0f
#define chassis_motor3508_max_out 16000.0f
#define chassis_motor3508_min_iout -7000.0f
#define chassis_motor3508_max_iout 7000.0f

//底盘跟随PID参数
#define chassis_follow_pid_kp 7.0f
#define chassis_follow_pid_ki 0.0f
#define chassis_follow_pid_kd 280.0f

#define chassis_follow_min_out -5.0f
#define chassis_follow_max_out 5.0f
#define chassis_follow_min_iout -0.2f
#define chassis_follow_max_iout 0.2f


//底盘参数设置cm/s
#define VxMinSpeed -400.0f
#define VxMaxSpeed 400.0f
#define VyMinSpeed -400.0f
#define VyMaxSpeed 400.0f

//底盘3508转轴最大转速限制rad/s
#ifdef RADUCTION_RATIO_1
#define AxleWMax 45.0f
#endif

#ifdef RADUCTION_RATIO_2
#define AxleWMax 62.0f
#endif

//遥感推到底的最大平移速度cm/s
#define RCcontrolMaxV 300.0f

//键鼠平移速度cm/s
#define KeycontrolV 300.0f                  //400能飞坡
//电容模式键鼠平移速度cm/s(飞坡模式)
#define KeycontrolCapV 400.0f

//小陀螺角速度rad/s
#define SpinW 9.0f                         //大约112w
#define SpinV 120.0f
//电容模式小陀螺角速度(暂时未使用)
#define SpinCapW 12.0f
#define SpinCapV 150.0f

//底盘跟随中心死区角速度
#define ZeroW 0.1f



//底盘模式参数
typedef enum{

    chassis_follow,
    chassis_not_follow,
    chassis_spin,
	chassis_down,

}chassis_mode_e;

//底盘参数
typedef struct{

    chassis_mode_e chassis_mode;

    fp32 vx_set;                      //底盘设定速度 前进方向 前为正， 单位 cm/s
	fp32 vx_filter_set;               //底盘滤波后设定速度 前进方向 前为正， 单位 cm/s
	fp32 vy_set;                      //底盘设定速度 左右方向 右为正， 单位 cm/s
	fp32 vy_filter_set;               //底盘滤波后设定速度 左右方向 右为正， 单位 cm/s
	fp32 w_set;                       //底盘设定旋转角速度，逆时针为正 单位 rad/s
	fp32 chassis_relative_angle;      //底盘与云台的相对角度，        单位 rad
	fp32 chassis_relative_angle_set;  //设置相对云台控制角度
	fp32 chassis_yaw_set;                

	fp32 vx_max_speed;  //前进方向最大速度 单位cm/s
	fp32 vx_min_speed;  //后退方向最大速度 单位cm/s
	fp32 vy_max_speed;  //左方向最大速度   单位cm/s
	fp32 vy_min_speed;  //右方向最大速度   单位cm/s 

	pid_type_def motor_pid1;

}chassis_move_t;


extern chassis_move_t chassis_move_data;

//底盘初始化函数
void fn_ChassisInit(void);

//底盘电机初始化函数
void fn_ChassisMotorInit(void);

//底盘模式选择
void fn_ChassisMode(void);

//底盘电流解算
void fn_ChassisMove(void);

#endif
