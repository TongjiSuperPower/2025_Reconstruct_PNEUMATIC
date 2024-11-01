#include "plot.h"
#include "usart.h"

PlotFrame plot_frame;

void plot1(fp32 value1)
{
    plot_frame.start[0] = 0xaa;
    plot_frame.start[1] = 0xbb;
    plot_frame.size = 4 * 1;
    plot_frame.data[0] = value1;
    HAL_UART_Transmit(&huart1, (uint8_t *)&plot_frame, 3 + plot_frame.size, 100);
}

void plot2(fp32 value1, fp32 value2)
{
    plot_frame.start[0] = 0xaa;
    plot_frame.start[1] = 0xbb;
    plot_frame.size = 4 * 2;
    plot_frame.data[0] = value1;
    plot_frame.data[1] = value2;
    HAL_UART_Transmit(&huart1, (uint8_t *)&plot_frame, 3 + plot_frame.size, 100);
}

void plot3(fp32 value1, fp32 value2, fp32 value3)
{
    plot_frame.start[0] = 0xaa;
    plot_frame.start[1] = 0xbb;
    plot_frame.size = 4 * 3;
    plot_frame.data[0] = value1;
    plot_frame.data[1] = value2;
    plot_frame.data[2] = value3;
    HAL_UART_Transmit(&huart1, (uint8_t *)&plot_frame, 3 + plot_frame.size, 100);
}

void plot4(fp32 value1, fp32 value2, fp32 value3, fp32 value4)
{
    plot_frame.start[0] = 0xaa;
    plot_frame.start[1] = 0xbb;
    plot_frame.size = 4 * 4;
    plot_frame.data[0] = value1;
    plot_frame.data[1] = value2;
    plot_frame.data[2] = value3;
    plot_frame.data[3] = value4;
    HAL_UART_Transmit(&huart1, (uint8_t *)&plot_frame, 3 + plot_frame.size, 100);
}

void plot5(fp32 value1, fp32 value2, fp32 value3, fp32 value4, fp32 value5)
{
    plot_frame.start[0] = 0xaa;
    plot_frame.start[1] = 0xbb;
    plot_frame.size = 4 * 5;
    plot_frame.data[0] = value1;
    plot_frame.data[1] = value2;
    plot_frame.data[2] = value3;
    plot_frame.data[3] = value4;
    plot_frame.data[4] = value5;
    HAL_UART_Transmit(&huart1, (uint8_t *)&plot_frame, 3 + plot_frame.size, 100);
}

void plot6(fp32 value1, fp32 value2, fp32 value3, fp32 value4, fp32 value5, fp32 value6)
{
    plot_frame.start[0] = 0xaa;
    plot_frame.start[1] = 0xbb;
    plot_frame.size = 4 * 6;
    plot_frame.data[0] = value1;
    plot_frame.data[1] = value2;
    plot_frame.data[2] = value3;
    plot_frame.data[3] = value4;
    plot_frame.data[4] = value5;
    plot_frame.data[5] = value6;
    HAL_UART_Transmit(&huart1, (uint8_t *)&plot_frame, 3 + plot_frame.size, 100);
}
