#include "pid.h"
#include "math_lib.h"
#include "struct_typedef.h"
#include "stdio.h"
#include "stdlib.h"

//PID��ʼ�� �˴���Сֵ������������ֵ
void fn_PidInit(pid_type_def *pid,const fp32 PID[3],fp32 min_out,fp32 max_out,fp32 min_iout,fp32 max_iout){
  if (pid == NULL || PID == NULL){
	  return;
	}
    pid->f_Kp = PID[0];
	pid->f_Ki = PID[1];
	pid->f_Kd = PID[2];
    pid->f_MinOut = min_out;
	pid->f_MaxOut = max_out;
    pid->f_MinIout = min_iout;
	pid->f_MaxIout = max_iout;
	pid->f_ErrorAngle = 0.0f;
	pid->f_Error = 0.0f;
	pid->f_LastError = 0.0f;
	pid->f_Iout = 0;
	pid->f_Dout = 0;
	pid->f_Pout = 0;
	pid->f_Out = 0;
}


//����λ�û�ʱ���ٶȵ���΢������
fp32 fn_delta_PidClac(pid_type_def *pid, fp32 ref, fp32 set, fp32 error_delta)
{
	if(pid == NULL)
		return 0.0f;
	pid->f_Error = set - ref;
	pid->f_Set = set;
	pid->f_Fdb = ref;
	
	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * error_delta;

	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;

}

//PID����
fp32 fn_PidClac(pid_type_def *pid,fp32 ref,fp32 set){
	
    pid->f_Set = set;
	pid->f_Fdb = ref;
	pid->f_Error = set-ref;

	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * (pid->f_Error - pid->f_LastError);
	pid->f_LastError = pid->f_Error;
	
	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;
	
}

//��תȡ�ӻ�PID
fp32 fn_PidClacAngle(pid_type_def *pid,fp32 ref,fp32 set){
	
    pid->f_Set = set;
	pid->f_Fdb = ref;
	pid->f_Error = fn_RadFormat(set-ref);       //����ת������
	pid->f_ErrorAngle = pid->f_Error*180/PI;   //jiao du

	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * (pid->f_Error - pid->f_LastError);
	pid->f_LastError = pid->f_Error;
	
	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;
	
}


//Ŀ��ֵ�ı�i�����PID����
fp32 fn_Iclear_PidClac(pid_type_def *pid,fp32 ref,fp32 set){
	
    // Ŀ��ı�ʱ������ֻ���������Ӧ�ٶ�
    if(set == 0) pid->f_Iout = 0.0f;
	
	pid->f_Set = set;
	pid->f_Fdb = ref;
	pid->f_Error = set-ref;

	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * (pid->f_Error - pid->f_LastError);
	pid->f_LastError = pid->f_Error;
	
	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;
	
}

fp32 fn_Iclear_PidClacAngle(pid_type_def *pid,fp32 ref,fp32 set){
	
  // Ŀ��ı�ʱ������ֻ���������Ӧ�ٶ�
    if(pid->f_Set == set) pid->f_Iout = 0.0f;  
	
	pid->f_Set = set;
	pid->f_Fdb = ref;
	pid->f_Error = fn_RadFormat(set-ref);       //����ת������
	pid->f_ErrorAngle = pid->f_Error*180/PI;   //jiao du

	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * (pid->f_Error - pid->f_LastError);
	pid->f_LastError = pid->f_Error;
	
	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;
	
}

