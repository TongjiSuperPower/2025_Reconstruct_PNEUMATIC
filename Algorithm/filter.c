#include "filter.h"
#include "struct_typedef.h"


//һ�׵�ͨ�˲���
void fn_low_filter(fp32 *filter_data,fp32 raw_data,fp32 x){
    *filter_data = *filter_data * (1-x) + raw_data * x;
}

