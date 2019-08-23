#ifndef __FILTERING_H
#define __FILTERING_H	 

#include "fdc2214.h"

//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter


float Get_Adc_Average(int ch,int times);
float Median_Filter(int ch);
float Slide_Filter(int ch);
float LAET_Filter(int ch);
float kalmanFilter(KFP *kfp,int ch);
int get_paper(float  y);
int get_paper_mi(float  y);
#endif


