#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "struct_typedef.h"
#include "gimbal_task.h"
#include "can_task.h"

/*RPM×ª»¡¶È/ÃëµÄÏµÊı*/
#define TriggerMotor3508_RPM_TO_SPEED  0.00284104f 
/*²¦µ¯ÂÖËÙ¶È»·PID²ÎÊı*/
#define TriggerMotor3508_SpeedPid_ID207_kp 400.0f
#define TriggerMotor3508_SpeedPid_ID207_ki 0.0f
#define TriggerMotor3508_SpeedPid_ID207_kd 0.0f
/*²¦µ¯ÂÖ½Ç¶È»·PID²ÎÊı*/
#define TriggerMotor3508_AnglePid_ID207_kp 78.3f//85.0//79.0f//78.3f//70.0f//73.0f//66.0f//40.0f//37.0f
#define TriggerMotor3508_AnglePid_ID207_ki 0.0f
#define TriggerMotor3508_AnglePid_ID207_kd -6.5f//-6.0f//-5.0f//-4.1f


/*pidï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ä½Ç¶È»ï¿½ï¿½ï¿½ï¿½ï¿½Ş·ï¿½Î?4.0*/
/*ÔÚ²¦µ¯ÂÖ¶¯×÷£¨Action£©¹ı³ÌÖĞÊ¹ÓÃµÄËÙ¶ÈÓë½Ç¶ÈPID²ÎÊı*/
#define TriggerMotor3508_ActionSpeedPid_ID207_kp 5000.0f
#define TriggerMotor3508_ActionSpeedPid_ID207_ki 0.0f
#define TriggerMotor3508_ActionSpeedPid_ID207_kd 0.0f
/*ï¿½ï¿½ï¿½ï¿½ï¿½Ö³ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½Ç¶È»ï¿½pidï¿½ï¿½ï¿½ï¿½*/
#define TriggerMotor3508_ActionAnglePid_ID207_kp 78.3f
#define TriggerMotor3508_ActionAnglePid_ID207_ki 6.0f
#define TriggerMotor3508_ActionAnglePid_ID207_kd -6.5f

/*ï¿½ï¿½Ê±Ã»ï¿½ï¿½ï¿½ï¿½*/
#define TriggerMotor3508SpeedMinOut -100.0f
#define TriggerMotor3508SpeedMaxOut 100.0f
#define TriggerMotor3508SpeedMinIOut -40.0f
#define TriggerMotor3508SpeedMaxIOut 40.0f


/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½PIDï¿½ï¿½ï¿½ï¿½Ş·ï¿?*/
#define TriggerMotor3508MinOut -16000.0f
#define TriggerMotor3508MaxOut 16000.0f
#define TriggerMotor3508MinIOut -7000.0f
#define TriggerMotor3508MaxIOut 7000.0f

#define AUTOAIM_SHOOT_KEY   KEY_PRESSED_OFFSET_V



//Ã¿ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È´Ê±ï¿½ï¿??
#define ShootColdTime 400             //ms
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½×ªï¿½ï¿½ï¿½ï¿½È´Ê±ï¿½ï¿½
#define ShootModeTime 1000            //ms
//Ã¿ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?ï¿½Ç¶ï¿½
#define ShootAngleAdd 1.0472f         //rad   0.92ï¿½ï¿½52.7ï¿½ï¿½
//ä¸€æ¬¡å°„å‡»ä¸ºäº†ä¿è¯å°„å‡ºæš‚æ—¶ï¿½?ï¿½è½¬çš„ï¿½?ï¿½åº¦
#define ShootAngleAdd_addition 0.0f
//ï¿½ï¿½ï¿½ï¿½ï¿½Ö¶ï¿½×ªï¿½ï¿½×ªï¿½Ç¶ï¿½
#define TriggerBackAngle 0.3f         //rad    ï¿½ï¿½10ï¿½ï¿½
//ï¿½ï¿½ß¸ï¿½ï¿½ï¿½ï¿½Ù³ï¿½ï¿½ï¿½Ê±ï¿½ï¿??
#define ShootTimeUpper 7000
//ï¿½ï¿½Í¸ï¿½ï¿½ï¿½ï¿½Ù³ï¿½ï¿½ï¿½Ê±ï¿½ï¿??
#define ShootTimeLower 1000
//Ä¦ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½rad/s
#define FricSpeed 520.0f
//Ë«ï¿½ï¿½ï¿½Ğ¶ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½Ä¦ï¿½ï¿½ï¿½Ö½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½?
#define FricSpeedReduce 30.0f
//Ë«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½
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

//Æø¸××´Ì¬»ú
typedef enum
{
    CYLIN_OPEN = 0,
	CYLIN_ACTION,
	CYLIN_CLOSE,
} cylinder_mode_e;

//¿ìÅÅ·§×´Ì¬»ú
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

    /*trigger_motor3508_data[0]?????????*/
    pid_type_def trigger_speed_pid; //²¦µ¯ÂÖËÙ¶È»·pid
	pid_type_def trigger_angle_pid; //²¦µ¯ÂÖ½Ç¶È»·pid
	pid_type_def trigger_action_speed_pid; 
	pid_type_def trigger_action_angle_pid;

    fp32 trigger_speed; //???????????? rpm*rpm_to_speed?????
    fp32 trigger_speed_set;
    fp32 trigger_angle;
    fp32 trigger_angle_set;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    
    //????????????
    uint16_t cylin_action2close_count;  
    uint16_t cylin_open2action_count;   
	uint16_t cylin_close2open_count;
	uint16_t quival_open2close_count;
	uint16_t shoot_action_plus_count;
	uint8_t angle_change_flag;  //Îª 0 Ê±Ö´ĞĞÒ»´ÎÄ¿±ê½Ç¶ÈÉèÖÃ£¬Ö®ºóÖÃÎª 1 ºó²»ÔÙÖØ¸´ÉèÖÃ
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
	uint8_t photogate;//?????
    uint16_t block_time;
	uint16_t reverse_time;

    uint8_t shoot_enable_flag;
}shoot_control_t;




extern shoot_control_t shoot_control;

void fn_ShootMotorInit(void);



void fn_shoot_init(void);


void fn_shoot_control_loop(void);

/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±*/
extern uint16_t shoot_single_time_count;
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¶Ï¼ï¿½ï¿½ï¿½ï¿½ï¿??
extern uint16_t shooting_single_count;

#endif
