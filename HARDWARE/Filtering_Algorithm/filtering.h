#ifndef __FILTERING_H
#define __FILTERING_H	 

#include "fdc2214.h"

//1. �ṹ�����Ͷ���
typedef struct 
{
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
}KFP;//Kalman Filter parameter


float Get_Adc_Average(int ch,int times);
float Median_Filter(int ch);
float Slide_Filter(int ch);
float LAET_Filter(int ch);
float kalmanFilter(KFP *kfp,int ch);
int get_paper(float  y);
int get_paper_mi(float  y);
#endif


