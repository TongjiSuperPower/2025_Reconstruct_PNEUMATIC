#ifndef _PID_H_
#define _PID_H_
#include "struct_typedef.h"

//PID
typedef struct{
  //coefficient
	fp32 f_Kp;
	fp32 f_Ki;
	fp32 f_Kd;
	
	//error
	fp32 f_Error;       //���
	fp32 f_ErrorAngle; //
	fp32 f_LastError;  // ��һ�����ֵ
	
	//pid limitation
	fp32 f_MinIout;
	fp32 f_MaxIout;
	fp32 f_MinOut;
	fp32 f_MaxOut;
	
	//real and target
	fp32 f_Set;
	fp32 f_Fdb;
	
	//pid out
	fp32 f_Out;
	fp32 f_Pout;
	fp32 f_Iout;
	fp32 f_Dout;
	
}pid_type_def;


//PID��ʼ��
void fn_PidInit(pid_type_def *pid,const fp32 PID[3],fp32 min_out,fp32 max_out,fp32 min_iout,fp32 max_iout);

//PID����
fp32 fn_PidClac(pid_type_def *pid,fp32 ref,fp32 set);

fp32 fn_PidClacAngle(pid_type_def *pid,fp32 ref,fp32 set);

fp32 fn_Iclear_PidClac(pid_type_def *pid,fp32 ref,fp32 set);

fp32 fn_Iclear_PidClacAngle(pid_type_def *pid,fp32 ref,fp32 set);

fp32 fn_delta_PidClac(pid_type_def *pid, fp32 ref, fp32 set, fp32 error_delta);
#endif
