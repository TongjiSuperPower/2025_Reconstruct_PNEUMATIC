#ifndef _PLOT_H_
#define _PLOT_H_

#include "struct_typedef.h"

#pragma pack(1)
typedef struct 
{
    uint8_t start[2];
    uint8_t size;
    fp32 data[10];
} PlotFrame;
#pragma pack()

void plot1(fp32 value1);
void plot2(fp32 value1, fp32 value2);
void plot3(fp32 value1, fp32 value2, fp32 value3);
void plot4(fp32 value1, fp32 value2, fp32 value3, fp32 value4);
void plot5(fp32 value1, fp32 value2, fp32 value3, fp32 value4, fp32 value5);
void plot6(fp32 value1, fp32 value2, fp32 value3, fp32 value4, fp32 value5, fp32 value6);

#endif // _PLOT_H
