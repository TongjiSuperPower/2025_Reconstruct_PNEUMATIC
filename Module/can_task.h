#ifndef _CAN_TASK_H_
#define _CAN_TASK_H_

#include "struct_typedef.h"
#include "pid.h"
#include "gimbal_task.h"
#include "can.h"

//�궨��
#define sp_RxID 0x300
#define sp_TxID 0x301
//״̬��־λ
#define sp_NORMAL
//������
#define CAP_AUTO			0x00	//�Զ�ģʽ��Ĭ�ϣ�
#define OUTPUT_DISABLE	    0x01	//ֻ�䲻��
#define INPUT_DISABLE		0x02	//ֻ�Ų��䣨�ֽ׶β����ã�
#define BOTH_DISABLE		0x03	//��Ŷ����?

//������
#define P_MIN -3.141593f
#define P_MAX 3.141593f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

//С�׵�����Ʋ�����ֵ����������??
#define T_MIN_MI -12.0f
#define T_MAX_MI 12.0f
#define MAX_P 720
#define MIN_P -720

//2PI/8192/3592*187/71*37 
#define MOTOR_ECD_TO_ANGLE19    0.000020808397f//0.000021726412189221046f 

//���ID����
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x04,

    CAN_TRIGGER_MOTOR_ID = 0x205,

	CAN_CAP_ID = 0X301,
	
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can2_msg_id_e;


typedef enum
{
    CAN_3508_FRICL_ID = 0x201,
    CAN_3508_FRICR_ID = 0x202,
    CAN_3508_FRICU_ID = 0x203,

    CAN_PIT_MOTOR_ID = 0x01,
    CAN_PIT_SEND_ID = 0x00,

    CAN_2006_MOTOR_1 = 0X206,
    CAN_2006_MOTOR_2 = 0X207,

    CAN_AUTOAIM_ID = 0x0FF,
    CAN_F103_ID = 0x306
} can1_msg_id_e;


//����?ʼ����
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;       //�˵�������������ͬ��
    uint8_t temperate;
    int16_t last_ecd;

    
} MotorMeasure_t;

//��������
typedef struct
{
    char vision_state;
    bool_t shoot;
    float yaw;
    float pitch;

} autoaim_measure_t;

//�µ��ݽṹ��
typedef struct
{	
	fp32 Cell_Power;		//��Դ���빦��*10
	fp32 Cap_Power;			//���ݳ�ŵ繦�ʣ����?���??��Ϊ�ŵ磩*10
	fp32 Capacity;			//���ݵ�ѹ���ŵ�3V��Ȼ��3v->6V����ٴηŵ�??*100
	uint8_t	Temputer;		//�¶�(�ֽ׶κ㶨Ϊ25)
	uint8_t	Status;			//״̬��־λ���ֽ׶α���Ϊ0x10��

}supercap_module_receive_new;


//f103
typedef struct
{
	uint8_t photogate;
	uint8_t quival_down;
} f103_data_t;

//�����������??
//DM4310
typedef struct{
    int32_t p_int;
    int32_t v_int;
    int32_t t_int;
    int32_t round_num;
    float last_raw_position;
    float raw_position;       //δ����Ȧ�����position
    float position;
    float velocity;
    float torque;

    fp32 offecd_angle;
    fp32 target_angle;

    pid_type_def motor_pid1;
    fp32 double_pid_mid;
    pid_type_def motor_pid2;

    float target_torque;

}DM_motor_data_t;

//3508
typedef struct{
	
    int32_t round_num;                //��Ȧ����Ȧ��

    fp32 relative_raw_angle;          //ת����������?ʼ���ݽ������δ�˲��Ƕ�??
    fp32 raw_angle[6];                //ת�������offecd.ecd�ĽǶȣ������ƣ���δ�˲���  ���ݣ�����--�ɣ�[0]--[5]��
    fp32 filter_angle[2];             //�˲���ĵ�ǰ�Ƕ�??                               ���ݣ�����--�ɣ�[0]--[2]��

    fp32 relative_raw_speed;          //ת����������?ʼ�ٶȽ������δ�˲��ٶ�??
    fp32 raw_speed[6];                //δ�˲�ת����ٶ�??                                 ���ݣ�����--�ɣ�[0]--[5]��
    fp32 filter_speed[2];             //�˲����ٶ�                             

    
    fp32 relative_angle_19laps;     //�����ֽǶ�

    uint16_t offecd_ecd;              //��ֵ����ֵ
    fp32 target_angle;                //Ŀ��Ƕ�
    fp32 target_speed;
    fp32 filter_given_current;        //�˲������
    fp32 given_current;               //������ĵ��� ������Χ[-16384,16384]

    pid_type_def motor_pid1;          //˫���⻷PID����
    fp32 double_pid_mid;              //˫��PID���м����??
	pid_type_def motor_pid2;          //˫���ڻ�PID����

    pid_type_def motor_pid3;          //���ٶȻ�PID����
	
}Motor3508Data_t;

//6020
typedef struct{
	
    //int32_t round_num;
    fp32 relative_raw_angle;          //ת����������?ʼ���ݽ������δ�˲��Ƕ�??
    fp32 raw_angle[6];                //ת�������offecd.ecd�ĽǶȣ������ƣ���δ�˲���  ���ݣ�����--�ɣ�[0]--[5]��
    fp32 filter_angle[2];             //�˲���ĵ�ǰ�Ƕ�??                               ���ݣ�����--�ɣ�[0]--[2]��

	fp32 relative_raw_speed;          //ת����ٶ�??

	uint16_t offecd_ecd;              //���ָ�����?��ǰ���ľ�������ֵ
	fp32 target_angle;                //ת����Ҫת���ĽǶ�
	fp32 given_voltage;               //���͸�����ĵ����?  ��Χ [-30000,30000]

    pid_type_def motor_pid1;          //˫���⻷PID����
    fp32 double_pid_mid;              //˫��PID���м����??
    pid_type_def motor_pid2;          //˫���ڻ�PID����

	//pid_type_def motor_pid_3;         //���ٶȻ�PID����
	
}Motor6020Data_t;

//С��
typedef struct
{
    uint32_t ecd;
    float speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int32_t last_ecd;
	fp32 relative_angle_19laps;
	int16_t ecd_count;
	uint8_t CAN_ID;//????damiao????????ID??zlw??
	uint8_t MCU_ID;
	float Torque;//��???dm????
	float Temp;  
	uint8_t error_code;

    // for dm_motor
    float pos;//??��rad??dm??
    float last_pos;
    int p_int;
    float vel;//??��rad/s??dm??
    int v_int;
    int t_int;
    uint8_t Master_ID;//?????ID
    int err_state;

    pid_type_def motor_pid1;          //˫���⻷PID����
    fp32 double_pid_mid;              //˫��PID���м����??
    pid_type_def motor_pid2;          //˫���ڻ�PID����

} motor_mi_measure_t;

typedef struct{
	
    int32_t round_num;                //ת�Ӷ�Ȧ����

    fp32 relative_raw_angle;          //ת����������?ʼ���ݽ������δ�˲��Ƕ�??
    fp32 raw_angle[6];                //ת�������offecd.ecd�ĽǶȣ������ƣ���δ�˲���  ���ݣ�����--�ɣ�[0]--[5]��
    fp32 filter_angle[2];             //�˲���ĵ�ǰ�Ƕ�??                               ���ݣ�����--�ɣ�[0]--[2]��

    fp32 relative_raw_speed;          //ת����������?ʼ�ٶȽ������δ�˲��ٶ�??
    fp32 raw_speed[6];                //δ�˲�ת����ٶ�??                                 ���ݣ�����--�ɣ�[0]--[5]��
    fp32 filter_speed[2];             //�˲���ĵ�ǰ�ٶ�??                                 ���ݣ�����--�ɣ�[0]--[2]��

    uint16_t offecd_ecd;              //����ָ����ǰ��ʱ��������?
    fp32 target_angle;                //��Ҫת��ת���ĽǶ�
    fp32 target_speed;
    fp32 filter_given_current;        //�˲���ķ��صĵ���??
    fp32 given_current;               //���͸�����ĵ����?  ��Χ [-16384,16384]

    pid_type_def motor_pid1;          //˫���⻷PID����
    fp32 double_pid_mid;              //˫��PID���м����??
	pid_type_def motor_pid2;          //˫���ڻ�PID����

    pid_type_def motor_pid3;          //���ٶȻ�PID����
	
}Motor_mi_data_t;


//����CANID����
#define Master_CAN_ID 0x00                      //����ID
//���������??��
#define Communication_Type_GetID 0x00           //��ȡ�豸��ID��64λMCUΨһ��ʶ��
#define Communication_Type_MotionControl 0x01 	//�������������Ϳ���ָ��
#define Communication_Type_MotorRequest 0x02	//���������������������״�?
#define Communication_Type_MotorEnable 0x03	    //���ʹ������??
#define Communication_Type_MotorStop 0x04	    //���ֹͣ����??
#define Communication_Type_SetPosZero 0x06	    //���õ����е���?
#define Communication_Type_CanID 0x07	        //���ĵ�ǰ���CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//��ȡ��������
#define Communication_Type_SetSingleParameter 0x12	//�趨��������
#define Communication_Type_ErrorFeedback 0x15	    //���Ϸ���֡
//������ȡ�궨��
#define Run_mode 0x7005	
#define Iq_Ref   0x7006
#define Spd_Ref  0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK 0X01

enum CONTROL_MODE   //����ģʽ����
{
    Motion_mode = 0,//�˿�ģʽ  
    Position_mode,  //λ��ģʽ
    Speed_mode,     //�ٶ�ģʽ  
    Current_mode    //����ģʽ
};
enum ERROR_TAG      //����ش�����??
{
    OK                 = 0,//�޹���
    BAT_LOW_ERR        = 1,//Ƿѹ����
    OVER_CURRENT_ERR   = 2,//����
    OVER_TEMP_ERR      = 3,//����
    MAGNETIC_ERR       = 4,//�ű������??
    HALL_ERR_ERR       = 5,//HALL�������??
    NO_CALIBRATION_ERR = 6//δ�궨
};



extern MotorMeasure_t chassis_motor3508_measure[4];    //id [0x201,0x202,0x203,0x204]   CAN2
extern MotorMeasure_t trigger_motor3508_measure[1];    //id [0x207]                     CAN2   
extern MotorMeasure_t gimbal_motor3508_measure[3];     //id [0x201,0x202]  [left,right] CAN1
extern MotorMeasure_t gimbal_motor2006_measure[2];
extern motor_mi_measure_t gimbal_motormi_measure[1];                 //Ԥ�ȶ���С�׵��??                CAN1

extern Motor3508Data_t chassis_motor3508_data[4];      //id [0x201,0x202,0x203,0x204]   CAN2
extern Motor3508Data_t trigger_motor3508_data[1];      //id [0x207]                     CAN2
extern Motor3508Data_t gimbal_motor3508_data[3];       //id [0x201,0x202]  [left,right] CAN1
extern Motor3508Data_t gimbal_motor2006_data[2];
extern DM_motor_data_t gimbal_motor4310_data[1];       //id [0x08]  [yaw]               CAN2

extern Motor_mi_data_t gimbal_motormi_data[1];  //Ԥ�ȶ���С�׵��??


extern supercap_module_receive_new cap_data;
extern const f103_data_t *fn_get_f103_data_point(void); //�����ַ�ʽ���ʷ�ֹf103�ṹ�屻�޸�

extern autoaim_measure_t autoaim_measure;

// ������ʹ��֡
void fn_DM_start_motor(void);

// �������������??
void fn_DM_record_init_state(void);

// CAN2 YAW������������?
void fn_ctrl_DM_motor(float _pos, float _vel, float _KP, float _KD, float _torq);

//С�׵����ʼ������?? �˿�ģʽ��0��ֻ�������һ������??
void init_cybergear(uint8_t mode);

//С���˿�ģʽ���ͺ���
void motor_mi_controlmode(float torque, float MechPosition, float speed, float kp, float kd);

//����3508�������ͺ���
void fn_cmd_CAN2ChassisMotor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

//��̨6020��3508�������ͺ���
void fn_cmd_CAN2TriggerMotor(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

//��̨6020��Ħ���ֵ���������ͺ���??
void fn_cmd_CAN1GimbalMotor1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

//������pitch����������ͺ���??
void fn_cmd_CAN1GimbalMotor2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

//��Ԫ�����ͺ���  CAN1
void fn_cmd_quat_to_computer(fp32 x, fp32 y, fp32 z, fp32 w);

//���״�?���ͺ���  CAN1
void fn_cmd_shoot_data_to_computer(fp32 speed, char mode);

//����3508������ݽ���??
void fn_ChassisMotor3508Data(uint8_t i);

//��̨6020������ݽ���??
void fn_GimbalMotor6020Data(uint8_t i);

//��̨3508���������ݽ���
void fn_TriggerMotor3508Data(uint8_t i);

//��̨3508Ħ�������ݽ���
void fn_GimbalMotor3508Data(uint8_t i);

void fn_cmd_supercap(int16_t flag, uint16_t chassis_power_limit, uint8_t chassis_output, fp32 chassis_power, uint16_t buffer_energy);

//��f103���Ϳ���֡
void fn_cmd_F103(uint8_t cylinder_push, uint8_t quick_valve_exhaust, int16_t press_time, int16_t exhaust_time);

extern HAL_StatusTypeDef can1_flag;
extern HAL_StatusTypeDef can2_flag;

#endif


