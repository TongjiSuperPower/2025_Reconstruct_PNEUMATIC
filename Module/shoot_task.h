#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "struct_typedef.h"
#include "gimbal_task.h"
#include "can_task.h"

/*RPMת���ɽ��ٶȵ�ϵ��*/
#define TriggerMotor3508_RPM_TO_SPEED  0.00284104f 
/*�������ٶȻ�pid����*/
#define TriggerMotor3508_SpeedPid_ID207_kp 400.0f
#define TriggerMotor3508_SpeedPid_ID207_ki 0.0f
#define TriggerMotor3508_SpeedPid_ID207_kd 0.0f
/*�����ֽǶȻ�pid����*/
#define TriggerMotor3508_AnglePid_ID207_kp 78.3f//85.0//79.0f//78.3f//70.0f//73.0f//66.0f//40.0f//37.0f
#define TriggerMotor3508_AnglePid_ID207_ki 0.0f
#define TriggerMotor3508_AnglePid_ID207_kd -6.5f//-6.0f//-5.0f//-4.1f


/*pid��ʼ��ʱ������ת���ĽǶȻ�����޷�Ϊ4.0*/
/*�����ֳ���ת���ٶȻ�pid����*/
#define TriggerMotor3508_ActionSpeedPid_ID207_kp 5000.0f
#define TriggerMotor3508_ActionSpeedPid_ID207_ki 0.0f
#define TriggerMotor3508_ActionSpeedPid_ID207_kd 0.0f
/*�����ֳ���ת���ǶȻ�pid����*/
#define TriggerMotor3508_ActionAnglePid_ID207_kp 78.3f
#define TriggerMotor3508_ActionAnglePid_ID207_ki 6.0f
#define TriggerMotor3508_ActionAnglePid_ID207_kd -6.5f

/*��ʱû����*/
#define TriggerMotor3508SpeedMinOut -100.0f
#define TriggerMotor3508SpeedMaxOut 100.0f
#define TriggerMotor3508SpeedMinIOut -40.0f
#define TriggerMotor3508SpeedMaxIOut 40.0f


/*������PID����޷�*/
#define TriggerMotor3508MinOut -16000.0f
#define TriggerMotor3508MaxOut 16000.0f
#define TriggerMotor3508MinIOut -7000.0f
#define TriggerMotor3508MaxIOut 7000.0f

#define AUTOAIM_SHOOT_KEY   KEY_PRESSED_OFFSET_V



//ÿ�������ȴʱ��?
#define ShootColdTime 400             //ms
//��������ģʽת����ȴʱ��
#define ShootModeTime 1000            //ms
//ÿ��������������?�Ƕ�
#define ShootAngleAdd 1.0472f         //rad   0.92��52.7��
//一次射击为了保证射出暂时�?�转的�?�度
#define ShootAngleAdd_addition 0.0f
//�����ֶ�ת��ת�Ƕ�
#define TriggerBackAngle 0.3f         //rad    ��10��
//��߸����ٳ���ʱ��?
#define ShootTimeUpper 7000
//��͸����ٳ���ʱ��?
#define ShootTimeLower 1000
//Ħ����ת��rad/s
#define FricSpeed 520.0f
//˫���ж�һ�����Ħ���ֽ������?
#define FricSpeedReduce 30.0f
//˫��������ʱ��
#define DetectTime 600

#define BLOCK_TRIGGER_SPEED         0.8f
#define BLOCK_TIME                  1000//700
#define REVERSE_TIME                800//800//500
#define REVERSE_SPEED_LIMIT         13.0f
#define MOVE_TIME                   1500

#define PI_FOUR                     0.78539816f
#define PI_TEN                      0.314f
#define PI_SEVEN					0.897f
#define PI_THREE					1.047198f

typedef enum{
    
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
    
}shoot_mode_e;

typedef enum
{
    TRIGGER_ACTION = 0,
	TRIGGER_ACTION_PLUS,
	TRIGGER_REVERSE,
	TRIGGER_DOWN,
} trigger_mode_e;

//����״̬��
typedef enum
{
    CYLIN_OPEN = 0,
	CYLIN_ACTION,
	CYLIN_CLOSE,
} cylinder_mode_e;

//���ŷ�״̬��
typedef enum
{
    QUIVAL_OPEN = 0,
	QUIVAL_CLOSE,
} quickvalve_mode_e;

typedef struct 
{
    shoot_mode_e shoot_mode;
	trigger_mode_e trigger_mode;
	cylinder_mode_e cylinder_mode;
	quickvalve_mode_e quickvalve_mode;

    /*trigger_motor3508_data[0]Ϊ�����ֵ��*/
    pid_type_def trigger_speed_pid; //�������ٶȻ�pid
	pid_type_def trigger_angle_pid; //�����ֽǶȻ�pid
	pid_type_def trigger_action_speed_pid; 
	pid_type_def trigger_action_angle_pid;

    fp32 trigger_speed; //�����ַ����ٶ� rpm*rpm_to_speed���˲�
    fp32 trigger_speed_set;
    fp32 trigger_angle;
    fp32 trigger_angle_set;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    
    //״̬��ת����ʱ��
    uint16_t cylin_action2close_count;  
    uint16_t cylin_open2action_count;   
	uint16_t cylin_close2open_count;
	uint16_t quival_open2close_count;
	uint16_t shoot_action_plus_count;
	uint8_t angle_change_flag;  //Ϊ0��ִ��һ��Ŀ��Ƕ��趨��֮��������Ϊ1���˺󲻻��ظ��趨
	uint8_t reverse_angle_change_flag;
	uint8_t cylinder_cmd;
	uint8_t quival_cmd;
    bool_t move_flag;
	bool_t move_flag2;
    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
    int16_t last_v_key;
	int16_t v_key;
	int16_t autoaim_mode;
	bool_t shoot_flag;
	bool_t last_shoot_flag;
    
    const f103_data_t *f103_data;
    uint8_t last_photogate;
	uint8_t photogate;//�����
    uint16_t block_time;
	uint16_t reverse_time;

    uint8_t shoot_enable_flag;
}shoot_control_t;




extern shoot_control_t shoot_control;

void fn_ShootMotorInit(void);



void fn_shoot_init(void);


void fn_shoot_control_loop(void);

/*������ʱ*/
extern uint16_t shoot_single_time_count;
//��������жϼ�����?
extern uint16_t shooting_single_count;

#endif
