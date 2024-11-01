#ifndef _MATH_LIB_H_
#define _MATH_LIB_H_
#define PI 3.14159265358979f
#include "struct_typedef.h"


#define f_Lx 16.5f                     //������֮������һ��            cm
#define f_Ly 18.4f                     //�����֮������һ��            cm
#define f_R 7.7f                       //���ְ뾶                       cm
#define f_GimbalDistance 0.0f          //��̨λ����Ե���������ǰƫ����  cm



//�޷�������������ֵ������һ������,������(min,max)
void fn_Uint16Limit(uint16_t *input,uint16_t min,uint16_t max);

//�޷�������������ֵ������һ������,������(min,max)
void fn_Fp32Limit(fp32 *input,fp32 min,fp32 max);

//ѭ���޷���������������ת����һ������,������(minValue,maxValue)
uint16_t fn_Uint16LoopLimit(uint16_t Input,uint16_t minValue,uint16_t maxValue);

//ѭ���޷���������������ת����һ������,������(minValue,maxValue)
fp32 fn_Fp32LoopLimit(fp32 Input,fp32 minValue,fp32 maxValue);

//����ת��(-pi,pi)
fp32 fn_RadFormat(fp32 rad);

//ƽ��������
float fn_InvSqrt(float x);

//��Χ�жϺ��� �ڷ�Χ���򷵻�0 ������Χ�򷵻�1
bool_t fn_scope_judgment(fp32 angle,fp32 min_angle,fp32 max_angle);

int float_to_uint(float x, float x_min, float x_max, int bits);
    
float uint_to_float(int x_int, float x_min, float x_max, int bits);


//ͨ������õ��ĽǶȼ���ÿ�����ֵ�ת��    v1��v2�ĵ�λ����cm/min
fp32 fn_WheelSpeedW1(fp32 v1,fp32 v2,fp32 w,fp32 a);     //��ǰ

fp32 fn_WheelSpeedW2(fp32 v1,fp32 v2,fp32 w,fp32 a);     //��ǰ

fp32 fn_WheelSpeedW3(fp32 v1,fp32 v2,fp32 w,fp32 a);     //�Һ�

fp32 fn_WheelSpeedW4(fp32 v1,fp32 v2,fp32 w,fp32 a);     //���

#endif
