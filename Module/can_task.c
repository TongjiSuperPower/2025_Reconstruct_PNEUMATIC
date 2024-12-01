#include "can_task.h"
#include "main.h"
#include "struct_typedef.h"
#include "math_lib.h"
#include "pid.h"
#include "filter.h"
#include "supercap_task.h"
#include "gimbal_task.h"

HAL_StatusTypeDef can1_flag;
HAL_StatusTypeDef can2_flag;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//���ԭʼ��������
MotorMeasure_t chassis_motor3508_measure[4];    //id [0x201,0x202,0x203,0x204]             CAN2
MotorMeasure_t trigger_motor3508_measure[1];    //id [0x207]                               CAN2   
MotorMeasure_t gimbal_motor3508_measure[3];     //id [0x201,0x202,0x203]  [left,right,up]  CAN1
MotorMeasure_t gimbal_motor2006_measure[2];     //id [0x205,0x206]                         CAN1
motor_mi_measure_t gimbal_motormi_measure[1];   //Ԥ�ȶ���С�׵��                          CAN1
//���������������
Motor3508Data_t chassis_motor3508_data[4];      //id [0x201,0x202,0x203,0x204]             CAN2
Motor3508Data_t trigger_motor3508_data[1];      //id [0x207]                               CAN2
Motor3508Data_t gimbal_motor3508_data[3];       //id [0x201,0x202,0x203]  [left,right,up]  CAN1
Motor3508Data_t gimbal_motor2006_data[2];       //id [0x205,0x206]                         CAN1
DM_motor_data_t gimbal_motor4310_data[1];       //id [0x08]  [yaw]                         CAN2
Motor_mi_data_t gimbal_motormi_data[1];         //Ԥ�ȶ���С�׵��
//С�׵��
CAN_RxHeaderTypeDef rxMsg;
CAN_TxHeaderTypeDef txMsg;
uint8_t byte[4];              //ת����ʱ����
uint32_t Motor_Can_ID;        //�������ݵ��ID

//������������
supercap_module_receive_new cap_data;

//��������
autoaim_measure_t autoaim_measure;

//f103����
static f103_data_t f103_data;


//����3508������ݽ���
void fn_ChassisMotor3508Data(uint8_t i);

//��̨6020������ݽ���
//void fn_GimbalMotor6020Data(uint8_t i);

//��̨3508���������ݽ���
void fn_TriggerMotor3508Data(uint8_t i);

//��̨3508Ħ�������ݽ���
void fn_GimbalMotor3508Data(uint8_t i);

//С�׵�����ݽ���
void fn_GimbalMotorMidata(void);




//��ȡf103ָ��
const f103_data_t *fn_get_f103_data_point(void)
{
    return &f103_data;
}

//��ȡ����ظ�֡��չID�еĵ��CANID
static uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame)
{
	return (CAN_ID_Frame&0xFFFF)>>8;
}

//С�׵������ת��
uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}
int float_to_uint_mi(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

//f103���ݻ�ȡ
void get_f103_data(f103_data_t *ptr, uint8_t data[8])
{
	ptr->photogate = data[0];
	ptr->quival_down = (data[1]);
}

// ���������ݻ�ȡ
void get_dm_motor_data(DM_motor_data_t *data, uint8_t rx_data[8])
{
	data->p_int = (rx_data[1] << 8) | rx_data[2];
	data->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	data->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
	data->last_raw_position = data->raw_position;
	data->raw_position = uint_to_float(data->p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	//�����Ȧ�����ĽǶ�
//	if((data->raw_position - data->last_raw_position) < -4*PI){
//		data->round_num --;
//	}
//	if((data->raw_position - data->last_raw_position) > 4*PI){
//		data->round_num ++;
//	}
	data->position = (float)fn_RadFormat(data->raw_position - data->offecd_angle + (8*PI - 25) * data->round_num);
	data->velocity = uint_to_float(data->v_int, V_MIN, V_MAX, 12);						// (-45.0,45.0)
	data->torque = uint_to_float(data->t_int, T_MIN, T_MAX, 12);						// (-18.0,18.0)
}

//С�׵�����ݻ�ȡ
void get_mi_motor_data(motor_mi_measure_t *Motor,uint8_t DataFrame[8],uint32_t IDFrame)
{	
	Motor->last_ecd = Motor->ecd;
	Motor->ecd=DataFrame[0]<<8|DataFrame[1];
	Motor->speed_rpm=uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
	Motor->Torque=uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN_MI,T_MAX_MI,16);				
	Motor->Temp=(DataFrame[6]<<8|DataFrame[7])*Temp_Gain;
	Motor->error_code=(IDFrame&0x1F0000)>>16;	
}

//�󽮵�����ݻ�ȡ
void get_motor_measure(MotorMeasure_t* motor_state,uint8_t rx_data[8])
{
    (motor_state)->last_ecd = (motor_state)->ecd;                                   
    (motor_state)->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);            
    (motor_state)->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    (motor_state)->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);  
    (motor_state)->temperate = (rx_data)[6]; 
}

//��λ���������ݻ�ȡ
void get_autoaim_measure(autoaim_measure_t* autoaim_state,uint8_t rx_data[8])
{
    autoaim_state->vision_state = rx_data[0];
    autoaim_state->shoot = (rx_data[1] == 1);
    autoaim_state->yaw = (int16_t)(rx_data[2] << 8 | rx_data[3]) / 1e4f;
    autoaim_state->pitch = (int16_t)(rx_data[4] << 8 | rx_data[5]) / 1e4f;
}

void get_cap_data(supercap_module_receive_new *ptr, uint8_t *Data)
{
	(ptr)->Cell_Power = ((int16_t)(((Data)[1] << 8) | (Data)[0]))/10.0f;
	(ptr)->Cap_Power = ((int16_t)(((Data)[3] << 8) | (Data)[2]))/10.0f;
	(ptr)->Capacity = ((int16_t)(((Data)[5] << 8) | (Data)[4]))/100.0f;
	(ptr)->Temputer = (Data)[6];
	(ptr)->Status = (Data)[7];
}

//����ȡ�����ݽ��㵽ÿ�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if(rx_header.IDE == CAN_ID_STD)
	{
	    if(hcan==&hcan2)             
	    {
		    switch (rx_header.StdId)
		    {
		
			    case CAN_3508_M1_ID:
			    case CAN_3508_M2_ID:
			    case CAN_3508_M3_ID:
			    case CAN_3508_M4_ID:
			    {
			    	uint8_t i = 0;
			    	i = rx_header.StdId - CAN_3508_M1_ID;
			    	get_motor_measure(&chassis_motor3508_measure[i], rx_data);
			    	fn_ChassisMotor3508Data(i);
			    	break;
			    }
            
                case CAN_YAW_MOTOR_ID:
			    {
			    	uint8_t i = 0;
			    	get_dm_motor_data(&gimbal_motor4310_data[i], rx_data);
			    	break;
			    }

			    case CAN_TRIGGER_MOTOR_ID:
			    {
			    	uint8_t i = 0;
			    	get_motor_measure(&trigger_motor3508_measure[i], rx_data);
			    	fn_TriggerMotor3508Data(i);
			    	break;
			    }

			    case CAN_CAP_ID:
			    {
			    	get_cap_data(&cap_data, rx_data);
			    	break;
			    }


			    default:
			    {
			    	break;
			    }
		    }
	    }

	    if(hcan==&hcan1)
	    {
	    	switch (rx_header.StdId)
	    	{
		    
	    		case CAN_3508_FRICL_ID:
                case CAN_3508_FRICR_ID:
	    		case CAN_3508_FRICU_ID:
	    		{
	    			static uint8_t i = 0;
	    			i = rx_header.StdId - CAN_3508_FRICL_ID;
	    			get_motor_measure(&gimbal_motor3508_measure[i], rx_data);
	    			fn_GimbalMotor3508Data(i);
	    			break;
	    		}
            
                case CAN_AUTOAIM_ID:
	    		{
	    			get_autoaim_measure(&autoaim_measure, rx_data);
	    			fn_Fp32Limit(&autoaim_measure.pitch,-0.45f,0.35f);
	    			break;
	    		}
				case CAN_F103_ID:
				{
					get_f103_data(&f103_data,rx_data);
					break;
				}
	    		

	    		default:
	    		{
	    			break;
	    		}
	    	}
	    }
	}
	else
	{

		Motor_Can_ID = Get_Motor_ID(rx_header.ExtId);//���Ȼ�ȡ�ش����ID��Ϣ  
		switch(Motor_Can_ID)                   //����ӦID�����Ϣ��ȡ����Ӧ�ṹ��
		{
			case CAN_PIT_MOTOR_ID:  
				if(rx_header.ExtId>>24 != 0)               //����Ƿ�Ϊ�㲥ģʽ
				{
					get_mi_motor_data(&gimbal_motormi_measure[0],rx_data,rx_header.ExtId);
					fn_GimbalMotorMidata();
				}
				else 
					gimbal_motormi_measure[0].MCU_ID = rx_data[0];
				break;           
			default:
				break;		
		}
	}	
}

//��f103���Ϳ�������
void fn_cmd_F103(uint8_t cylinder_push, uint8_t quick_valve_exhaust, int16_t press_time, int16_t exhaust_time)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef shoot_tx_message;
	shoot_tx_message.StdId = 0x305;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;

	uint8_t shoot_can_send_data[8];
	shoot_can_send_data[0] = cylinder_push;
    shoot_can_send_data[1] = quick_valve_exhaust;
    shoot_can_send_data[2] = (press_time >> 8);
    shoot_can_send_data[3] = press_time;
    shoot_can_send_data[4] = (exhaust_time >> 8);
    shoot_can_send_data[5] = exhaust_time;
    shoot_can_send_data[6] = 0;
    shoot_can_send_data[7] = 0;

	HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

// ������ʹ��֡
void fn_DM_start_motor(void)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x08;
	Txheader2.RTR = CAN_RTR_DATA; // ��Ϣ����Ϊ����֡
	Txheader2.IDE = CAN_ID_STD;	  // ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;		  // ��Ϣ����Ϊ8�ֽ�
	
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = 0xFF;
	CAN_send_data2[1] = 0xFF;
	CAN_send_data2[2] = 0xFF;
	CAN_send_data2[3] = 0xFF;
	CAN_send_data2[4] = 0xFF;
	CAN_send_data2[5] = 0xFF;
	CAN_send_data2[6] = 0xFF;
	CAN_send_data2[7] = 0xFC;

	HAL_CAN_AddTxMessage(&hcan2, &Txheader2, CAN_send_data2, &send_mail_box2);
}

// �������������
void fn_DM_record_init_state(void)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x08;
	Txheader2.RTR = CAN_RTR_DATA; // ��Ϣ����Ϊ����֡
	Txheader2.IDE = CAN_ID_STD;	  // ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;		  // ��Ϣ����Ϊ8�ֽ�

	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = 0xFF;
	CAN_send_data2[1] = 0xFF;
	CAN_send_data2[2] = 0xFF;
	CAN_send_data2[3] = 0xFF;
	CAN_send_data2[4] = 0xFF;
	CAN_send_data2[5] = 0xFF;
	CAN_send_data2[6] = 0xFF;
	CAN_send_data2[7] = 0xFE;

	HAL_CAN_AddTxMessage(&hcan2, &Txheader2, CAN_send_data2, &send_mail_box2);
}

// CAN2 YAW�����������֡
void fn_ctrl_DM_motor(float _pos, float _vel, float _KP, float _KD, float _torq)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x08;
	Txheader2.RTR = CAN_RTR_DATA; // ��Ϣ����Ϊ����֡
	Txheader2.IDE = CAN_ID_STD;	  // ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;		  // ��Ϣ����Ϊ8�ֽ�

	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = (pos_tmp >> 8);
	CAN_send_data2[1] = pos_tmp;
	CAN_send_data2[2] = (vel_tmp >> 4);
	CAN_send_data2[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	CAN_send_data2[4] = kp_tmp;
	CAN_send_data2[5] = (kd_tmp >> 4);
	CAN_send_data2[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	CAN_send_data2[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan2, &Txheader2, CAN_send_data2, &send_mail_box2);
}

//С�׵��
//ʹ��
void start_cybergear(void)
{
	uint32_t send_mail_box2;
    uint8_t tx_data[8] = {0}; 
    txMsg.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|CAN_PIT_MOTOR_ID;
    HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, &send_mail_box2);
}
void Set_Motor_Parameter(uint16_t Index,float Value,char Value_type)
{
	uint32_t send_mail_box2;
	uint8_t tx_data[8];
	txMsg.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|CAN_PIT_MOTOR_ID;
	tx_data[0]=Index;
	tx_data[1]=Index>>8;
	tx_data[2]=0x00;
	tx_data[3]=0x00;
	if(Value_type == 'f'){
		Float_to_Byte(Value);
		tx_data[4]=byte[3];
		tx_data[5]=byte[2];
		tx_data[6]=byte[1];
		tx_data[7]=byte[0];		
	}
	else if(Value_type == 's'){
		tx_data[4]=(uint8_t)Value;
		tx_data[5]=0x00;
		tx_data[6]=0x00;
		tx_data[7]=0x00;				
	}
	HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, &send_mail_box2);	
}
void set_mode_cybergear(uint8_t Mode)
{	
	Set_Motor_Parameter(Run_mode,Mode,'s');
}
//С�׵����ʼ������ �˿�ģʽ��0��ֻ�������һ������
void init_cybergear(uint8_t mode)
{
    txMsg.StdId = 0;            //����CAN���ͣ���׼֡���� 
    txMsg.ExtId = 0;             //����CAN���ͣ���չ֡����     
    txMsg.IDE = CAN_ID_EXT;     //����CAN���ͣ���չ֡
    txMsg.RTR = CAN_RTR_DATA;   //����CAN���ͣ�����֡
    txMsg.DLC = 0x08;           //����CAN���ͣ����ݳ���
    
	set_mode_cybergear(mode);//���õ��ģʽ
	start_cybergear();        //ʹ�ܵ��
}

//С���˿�ģʽ���ͺ���
void motor_mi_controlmode(float torque, float MechPosition, float speed, float kp, float kd)
{   
	uint32_t send_mail_box2;
    uint8_t tx_data[8];//�������ݳ�ʼ��
    //װ�������
    tx_data[0]=float_to_uint_mi(MechPosition,P_MIN,P_MAX,16)>>8;  
    tx_data[1]=float_to_uint_mi(MechPosition,P_MIN,P_MAX,16);  
    tx_data[2]=float_to_uint_mi(speed,V_MIN,V_MAX,16)>>8;  
    tx_data[3]=float_to_uint_mi(speed,V_MIN,V_MAX,16);  
    tx_data[4]=float_to_uint_mi(kp,KP_MIN,KP_MAX,16)>>8;  
    tx_data[5]=float_to_uint_mi(kp,KP_MIN,KP_MAX,16);  
    tx_data[6]=float_to_uint_mi(kd,KD_MIN,KD_MAX,16)>>8;  
    tx_data[7]=float_to_uint_mi(kd,KD_MIN,KD_MAX,16); 
    
    txMsg.ExtId = Communication_Type_MotionControl<<24|float_to_uint_mi(torque,T_MIN,T_MAX,16)<<8|CAN_PIT_MOTOR_ID;//װ����չ֡����
    HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, &send_mail_box2);
}

//����3508�������ͺ���
void fn_cmd_CAN2ChassisMotor
	(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x200;
	Txheader2.IDE = CAN_ID_STD;   //ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;         //��Ϣ����Ϊ8�ֽ�
	Txheader2.RTR = CAN_RTR_DATA; //��Ϣ����Ϊ����֡
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor1 >> 8 ;
	CAN_send_data2[1] = motor1;
	CAN_send_data2[2] = motor2 >> 8 ;
	CAN_send_data2[3] = motor2;
	CAN_send_data2[4] = motor3 >> 8 ;
	CAN_send_data2[5] = motor3;
	CAN_send_data2[6] = motor4 >> 8 ;
	CAN_send_data2[7] = motor4;

	HAL_CAN_AddTxMessage
	(&hcan2,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}

//���̲�����
void fn_cmd_CAN2TriggerMotor
	(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x1FF;
	Txheader2.IDE = CAN_ID_STD;   //ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;         //��Ϣ����Ϊ8�ֽ�
	Txheader2.RTR = CAN_RTR_DATA; //��Ϣ����Ϊ����֡
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor5 >> 8 ;
	CAN_send_data2[1] = motor5;
	CAN_send_data2[2] = motor6 >> 8 ;
	CAN_send_data2[3] = motor6;
	CAN_send_data2[4] = motor7 >> 8 ;
	CAN_send_data2[5] = motor7;
	CAN_send_data2[6] = motor8 >> 8 ;
	CAN_send_data2[7] = motor8;

	can2_flag = HAL_CAN_AddTxMessage
	(&hcan2,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}

//��̨2006�������ͺ���
void fn_cmd_CAN1GimbalMotor2
	(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x1FF;
	Txheader2.IDE = CAN_ID_STD;   //ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;         //��Ϣ����Ϊ8�ֽ�
	Txheader2.RTR = CAN_RTR_DATA; //��Ϣ����Ϊ����֡
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor5 >> 8 ;
	CAN_send_data2[1] = motor5;
	CAN_send_data2[2] = motor6 >> 8 ;
	CAN_send_data2[3] = motor6;
	CAN_send_data2[4] = motor7 >> 8 ;
	CAN_send_data2[5] = motor7;
	CAN_send_data2[6] = motor8 >> 8 ;
	CAN_send_data2[7] = motor8;

	can1_flag = HAL_CAN_AddTxMessage
	(&hcan1,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}

//��̨Ħ���ֵ���������ͺ���
void fn_cmd_CAN1GimbalMotor1
	(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x200;
	Txheader2.IDE = CAN_ID_STD;   //ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;         //��Ϣ����Ϊ8�ֽ�
	Txheader2.RTR = CAN_RTR_DATA; //��Ϣ����Ϊ����֡
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor1 >> 8 ;
	CAN_send_data2[1] = motor1;
	CAN_send_data2[2] = motor2 >> 8 ;
	CAN_send_data2[3] = motor2;
	CAN_send_data2[4] = motor3 >> 8 ;
	CAN_send_data2[5] = motor3;
	CAN_send_data2[6] = motor4 >> 8 ;
	CAN_send_data2[7] = motor4;

	HAL_CAN_AddTxMessage
	(&hcan1,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}

//�������ݷ��ͺ���CAN2
void fn_cmd_supercap(int16_t flag, uint16_t chassis_power_limit, uint8_t chassis_output, fp32 chassis_power, uint16_t buffer_energy)
{
    uint32_t send_mail_box;

//	uint8_t Cap_qiangzhi=0x007;
    CAN_TxHeaderTypeDef supercap_tx_message;
    supercap_tx_message.StdId = 0x300;
    supercap_tx_message.IDE = CAN_ID_STD;
    supercap_tx_message.RTR = CAN_RTR_DATA;
    supercap_tx_message.DLC = 0x08;
	uint8_t supercap_can_send_data[8];

	supercap_can_send_data[0] = (unsigned char)(flag);
	supercap_can_send_data[1] = chassis_power_limit >> 8;
	supercap_can_send_data[2] = chassis_power_limit;
	supercap_can_send_data[3] = (uint16_t)(10*chassis_power) >> 8;
	supercap_can_send_data[4] = (uint16_t)(10*chassis_power);
	supercap_can_send_data[5] = ext_power_heat_data.buffer_energy >> 8;
	supercap_can_send_data[6] = ext_power_heat_data.buffer_energy;
	supercap_can_send_data[7] = chassis_output;
	
	HAL_CAN_AddTxMessage(&hcan2, &supercap_tx_message, supercap_can_send_data, &send_mail_box);
}

//��Ԫ�����ͺ���  CAN1
void fn_cmd_quat_to_computer(fp32 x, fp32 y, fp32 z, fp32 w)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x100;
	Txheader2.IDE = CAN_ID_STD;   //ID����Ϊ��׼ID
	Txheader2.DLC = 0x08;         //��Ϣ����Ϊ8�ֽ�
	Txheader2.RTR = CAN_RTR_DATA; //��Ϣ����Ϊ����֡
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = (int16_t)(x * 1e4f) >> 8 ;
	CAN_send_data2[1] = (int16_t)(x * 1e4f);
	CAN_send_data2[2] = (int16_t)(y * 1e4f) >> 8 ;
	CAN_send_data2[3] = (int16_t)(y * 1e4f);
	CAN_send_data2[4] = (int16_t)(z * 1e4f) >> 8 ;
	CAN_send_data2[5] = (int16_t)(z * 1e4f);
	CAN_send_data2[6] = (int16_t)(w * 1e4f) >> 8 ;
	CAN_send_data2[7] = (int16_t)(w * 1e4f);

	HAL_CAN_AddTxMessage
	(&hcan1,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}

//���״̬���ͺ���  CAN1 �������ӵ����ٶ� ģʽ��1Ϊ���顢2Ϊ���
void fn_cmd_shoot_data_to_computer(fp32 speed, char mode)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x101;
	Txheader2.IDE = CAN_ID_STD;   //ID����Ϊ��׼ID
	Txheader2.DLC = 0x03;         //��Ϣ����Ϊ3�ֽ�
	Txheader2.RTR = CAN_RTR_DATA; //��Ϣ����Ϊ����֡
	uint8_t CAN_send_data2[3];
	CAN_send_data2[0] = (uint16_t)(speed * 1e2f) >> 8 ;
	CAN_send_data2[1] = (uint16_t)(speed * 1e2f);
	CAN_send_data2[2] = mode ;

	HAL_CAN_AddTxMessage
	(&hcan1,&Txheader2,CAN_send_data2,&send_mail_box2);
	
}




//����3508������ݽ���
void fn_ChassisMotor3508Data(uint8_t i){
    
	//��Ȧ����
	//if((chassis_motor3508_measure[i].ecd - chassis_motor3508_measure[i].last_ecd) > 4096){
    //    chassis_motor3508_data[i].round_num --;
	//}
    //else if((chassis_motor3508_measure[i].ecd - chassis_motor3508_measure[i].last_ecd) < -4096){
    //    chassis_motor3508_data[i].round_num ++;
	//}
	//����Ƕ� rad
    //chassis_motor3508_data[i].relative_raw_angle = fn_RadFormat((float)((chassis_motor3508_measure[i].ecd - chassis_motor3508_data[i].offecd_ecd) + chassis_motor3508_data[i].round_num * 8192) / 8192.0f / 19.02f * 2 * PI);
	//for(uint8_t k = 5;0 < k;k--){

        //chassis_motor3508_data[i].raw_angle[k] = chassis_motor3508_data[i].raw_angle[k-1];

	//}
	//chassis_motor3508_data[i].raw_angle[0] = chassis_motor3508_data[i].relative_raw_angle;

    //�����ٶ� rad/s
	#ifdef RADUCTION_RATIO_1
	chassis_motor3508_data[i].relative_raw_speed = (float)chassis_motor3508_measure[i].speed_rpm * 2 * PI / 19.02f / 60.0f;
	#endif
	#ifdef RADUCTION_RATIO_2
	chassis_motor3508_data[i].relative_raw_speed = (float)chassis_motor3508_measure[i].speed_rpm * 2 * PI / 13.7f / 60.0f;
	#endif
	chassis_motor3508_data[i].filter_speed[1] = chassis_motor3508_data[i].filter_speed[0];
	fn_low_filter(&chassis_motor3508_data[i].filter_speed[0],chassis_motor3508_data[i].relative_raw_speed,0.06f);
	//for(uint8_t k = 5;0 < k;k--){

    //    chassis_motor3508_data[i].raw_speed[k] = chassis_motor3508_data[i].raw_speed[k-1];

	//}
	//chassis_motor3508_data[i].raw_speed[0] = chassis_motor3508_data[i].relative_raw_speed;

}




//3508���������ݽ���
void fn_TriggerMotor3508Data(uint8_t i){

    //��Ȧ����
	if((trigger_motor3508_measure[i].ecd - trigger_motor3508_measure[i].last_ecd) > 4096){
        trigger_motor3508_data[i].round_num --;
	}
    else if((trigger_motor3508_measure[i].ecd - trigger_motor3508_measure[i].last_ecd) < -4096){
        trigger_motor3508_data[i].round_num ++;
	}
	//����Ƕ� rad
    trigger_motor3508_data[i].relative_raw_angle = (float)fn_RadFormat(((trigger_motor3508_measure[i].ecd - trigger_motor3508_data[i].offecd_ecd) + trigger_motor3508_data[i].round_num * 8192) / 8192.0f / 19.02f * 2 * PI);

	//���㲦���ֽǶ�
	trigger_motor3508_data[i].relative_angle_19laps = (float)trigger_motor3508_data[i].round_num * 8192.0f + (float)trigger_motor3508_measure[i].ecd;
	while(trigger_motor3508_data[i].relative_angle_19laps >= (8192.0f*3592.0f/187.0f*71.0f/37.0f))
	{
		trigger_motor3508_data[i].relative_angle_19laps -= (8192.0f*3592.0f/187.0f*71.0f/37.0f);
	}
	trigger_motor3508_data[i].relative_angle_19laps = fn_RadFormat(trigger_motor3508_data[i].relative_angle_19laps * MOTOR_ECD_TO_ANGLE19);

    //�����ٶ� rad/s
	trigger_motor3508_data[i].relative_raw_speed = (float)trigger_motor3508_measure[i].speed_rpm * 2 * PI / 19.02f / 60.0f;
	trigger_motor3508_data[i].filter_speed[1] = trigger_motor3508_data[i].filter_speed[0];
	fn_low_filter(&trigger_motor3508_data[i].filter_speed[0],trigger_motor3508_data[i].relative_raw_speed,0.06f);

    fn_low_filter(&trigger_motor3508_data[0].filter_given_current,trigger_motor3508_measure[0].given_current,0.06f);
}


//��̨3508Ħ�������ݽ���
void fn_GimbalMotor3508Data(uint8_t i){

    
	gimbal_motor3508_data[i].relative_raw_speed = (float)gimbal_motor3508_measure[i].speed_rpm * 2 * PI / 60.0f;
	//gimbal_motor3508_data[i].filter_speed[1] = gimbal_motor3508_data[i].filter_speed[0];
	//fn_low_filter(&gimbal_motor3508_data[i].filter_speed[0],gimbal_motor3508_data[i].relative_raw_speed,0.06f);
}

//С�׵�����ݽ���
void fn_GimbalMotorMidata(void){
    
	//����Ƕ� rad
    gimbal_motormi_data[0].relative_raw_angle = fn_RadFormat((int32_t)(gimbal_motormi_measure[0].ecd - gimbal_motormi_data[0].offecd_ecd) / 16384.0f * 2 * PI);
	//�����ٶ� rad/s
    gimbal_motormi_data[0].relative_raw_speed = (float)gimbal_motormi_measure[0].speed_rpm;

}

void fn_GimbalMotor2006data(uint8_t i){
	//��Ȧ����
	if((gimbal_motor2006_measure[i].ecd - gimbal_motor2006_measure[i].last_ecd) > 4096){
        gimbal_motor2006_data[i].round_num --;
	}
    else if((gimbal_motor2006_measure[i].ecd - gimbal_motor2006_measure[i].last_ecd) < -4096){
        gimbal_motor2006_data[i].round_num ++;
	}
	//����Ƕ� rad
    gimbal_motor2006_data[i].relative_raw_angle = (float)fn_RadFormat(((gimbal_motor2006_measure[i].ecd - gimbal_motor2006_data[i].offecd_ecd) + gimbal_motor2006_data[i].round_num * 8192) / 8192.0f / 36.0f * 2 * PI);

    //�����ٶ� rad/s
	gimbal_motor2006_data[i].relative_raw_speed = (float)gimbal_motor2006_measure[i].speed_rpm * 2 * PI / 36.0f / 60.0f;
	gimbal_motor2006_data[i].filter_speed[1] = gimbal_motor2006_data[i].filter_speed[0];
	fn_low_filter(&gimbal_motor2006_data[i].filter_speed[0],gimbal_motor2006_data[i].relative_raw_speed,0.06f);
}

