#include "math_lib.h"
#include "struct_typedef.h"
#include "math.h"
#include "can_task.h"
#include "ins_task.h"


//�޷�������������ֵ������һ������,������
void fn_Uint16Limit(uint16_t *input,uint16_t min,uint16_t max){
  if(*input > max){
	  *input = max;
	}
	else if(*input < min){
	  *input = min;
	}
}

//�޷�������������ֵ������һ������,������
void fn_Fp32Limit(fp32 *input,fp32 min,fp32 max){
  if(*input > max){
	  *input = max;
	}
	else if(*input < min){
	  *input = min;
	}
}

//ѭ���޷���������������ת����һ������,������
uint16_t fn_Uint16LoopLimit(uint16_t Input,uint16_t minValue,uint16_t maxValue){
     if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        uint16_t len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        uint16_t len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//ѭ���޷���������������ת����һ������,������
fp32 fn_Fp32LoopLimit(fp32 Input,fp32 minValue,fp32 maxValue){
     if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//����ת��
fp32 fn_RadFormat(fp32 rad){
  if(rad > PI){
	  while(rad > PI){
		  rad -= 2*PI;
		}
	}
	if(rad < -PI){
	  while(rad < -PI){
		  rad += 2*PI;
		}
	}
	return rad;
}

//ƽ��������
float fn_InvSqrt(float x){
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//��Χ�жϺ��� �ڷ�Χ���򷵻�0 ������Χ�򷵻�1
bool_t fn_scope_judgment(fp32 angle,fp32 min_angle,fp32 max_angle){
    if(min_angle < angle && angle < max_angle){
        return 0;
    }
    return 1;
}

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


//ͨ������õ��ĽǶȼ���ÿ�����ֵ�ת��    v1��v2�ĵ�λ����cm/s
//��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
fp32 fn_WheelSpeedW1(fp32 v1,fp32 v2,fp32 w,fp32 a){      //��ǰ
  fp32 vx = v1 * cos(a) + v2 * sin(a);
  fp32 vy = -v1 * sin(a) + v2 * cos(a);
  fp32 W1 = (vx + vy - ((f_Lx - f_GimbalDistance) + f_Ly) * w) / f_R;
  return W1;
}

fp32 fn_WheelSpeedW2(fp32 v1,fp32 v2,fp32 w,fp32 a){      //��ǰ
  fp32 vx = v1 * cos(a) + v2 * sin(a);
  fp32 vy = -v1 * sin(a) + v2 * cos(a);
  fp32 W2 = (vx - vy + ((f_Lx - f_GimbalDistance) + f_Ly) * w) / f_R;
  return -W2;
}

fp32 fn_WheelSpeedW3(fp32 v1,fp32 v2,fp32 w,fp32 a){      //�Һ�
  fp32 vx = v1 * cos(a) + v2 * sin(a);
  fp32 vy = -v1 * sin(a) + v2 * cos(a);
  fp32 W3 = (vx + vy + ((f_Lx + f_GimbalDistance) + f_Ly) * w) / f_R;
	
  return -W3;
}

fp32 fn_WheelSpeedW4(fp32 v1,fp32 v2,fp32 w,fp32 a){      //���
  fp32 vx = v1 * cos(a) + v2 * sin(a);
  fp32 vy = -v1 * sin(a) + v2 * cos(a);
  fp32 W4 = (vx - vy - ((f_Lx + f_GimbalDistance) + f_Ly) * w) / f_R;
  return W4;
}


