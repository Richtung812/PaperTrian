#include "fdc2214.h"
#include <math.h>
#include "filtering.h"
#include "oled.h"
#include "key.h"
extern float a[8];
extern float b[50];
	
char BUF[64];//OLED��ʾ������ÿ��16���ַ�����4�С�
/*
ȡ10����ƽ��ֵ��ƽ���˲�
*/
float Get_Adc_Average(int ch,int times)
{
	float temp_val=0;
	int t;
	for(t=0;t<times;t++)
	{
		temp_val+=Cap_Calculate(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 	 

/*
��λֵƽ���˲��㷨
*/
#define Median_FILTER_N 100
float Median_Filter(int ch) {
  int i, j;
  float filter_temp, filter_sum = 0;
  float filter_buf[Median_FILTER_N];
  for(i = 0; i < Median_FILTER_N; i++) {
    filter_buf[i]= Cap_Calculate(ch);
    delay_ms(1);
  }
  // ��С���������ð�ݣ�
  for(j = 0; j < Median_FILTER_N - 1; j++) {
    for(i = 0; i < Median_FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  // ȥ����ֵ����ƽ��
  for(i = 1; i < Median_FILTER_N - 1; i++) filter_sum += filter_buf[i];
  return filter_sum / (Median_FILTER_N - 2);
}

/*
����ƽ���˲�������ƽ���˲���
*/
#define Slide_FILTER_N 12
float filter_buf[Slide_FILTER_N + 1];
float Slide_Filter(int ch) {
  int i;
  float filter_sum = 0;
  filter_buf[Slide_FILTER_N] = Cap_Calculate(ch);
  for(i = 0; i < Slide_FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; 
    filter_sum += filter_buf[i];
  }
  return filter_sum / Slide_FILTER_N;
}

/*
�޷������˲� (no!)
*/
#define LAET_FILTER_A 1
#define LAET_FILTER_N 5
float LAET_Filter(int ch) {
	int i = 0;
	float Filter_Value;
	float Value=150;
  float NewValue;
  float new_value;
  NewValue = Cap_Calculate(ch);
  if(((NewValue - Value) > LAET_FILTER_A) || ((Value - NewValue) > LAET_FILTER_A))
    new_value = Value;
  else
    new_value = NewValue;
  if(Value != new_value) {
    i++;
    if(i > LAET_FILTER_N) {
      i = 0;
      Value = new_value;
    }
  }
  else
    i = 0;
  return Value;
}

/*
�������˲�
*/
//1. �ṹ�����Ͷ���
//typedef struct 
//{
//    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
//    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
//    float out;//�������˲������ ��ʼ��ֵΪ0
//    float Kg;//���������� ��ʼ��ֵΪ0
//    float Q;//��������Э���� ��ʼ��ֵΪ0.001
//    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
//}KFP;//Kalman Filter parameter

/**
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
float kalmanFilter(KFP *kfp,int ch)
{
		 float input;
		 input= Cap_Calculate(ch);
     //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
     kfp->Now_P = kfp->LastP + kfp->Q;
     //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
     //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
}

int get_paper(float  y)
{
   double x,x0,f1,f0;
	 double temp;
	 f1 = exp((220.64-y)/48.72);
	 f0 = (exp((220.64-y)/48.72))*(-48.72);
	 temp=(220.64-y)/48.72;
	 x= exp(temp);
	return x;
}


int newdon(void)
{
 	float x,x0,f,f1; 
 	x = 2.0;
	do{ 
	       x0=x;
	       f=2*x0*x0*x0-4*x0*x0+3*x0-6;
	       f1=6*x0*x0-8*x0+3;
	       x=x0-f/f1; 
	//����fabs���󸡵���x�ľ���ֵ
    //˵��������|x|, ��x��Ϊ��ʱ���� x�����򷵻� -x      
	}while(fabs(x-x0)>=1e-5);
 	return 0 ;
}


int get_paper_mi(float  y)
{
	//ǰ10���������ϵ��
	double a1 = 1.07e+05;//83.24;
	double b1 = -1.979;
	//�м�10���������ϵ��
	double a2 = 2.733e+07;
	double b2 = -3.16;
	//��10���������ϵ��
	double a3 = 4.516e+09;
	double b3 = -4.299;
	//��30+���������ϵ��
	double a4 = 3.321e+11;
	double b4 = -5.28;
	//40-50���������ϵ��
	double a5 = log(y)-log(75.969);
	double b5 = -0.004;
	//50-60���������ϵ��
	double a6 = log(y)-log(73.287);
	double b6 = -0.002;
	//60-65���������ϵ��
	double a7 = log(y)-log(71.262);
	double b7 = -0.003;
	
	double fx; //����С�����ֽ����
	int f=0;//������ֽ����
	if (y>a[1]-(a[1]-a[2])/2 && y<a[0]+1)//                1~10
	{
		sprintf(BUF,"a1:%.3f",a1/100000);
		OLED_ShowString(0,1,BUF,8);
		sprintf(BUF,"b1:%.3f",b1);
		OLED_ShowString(0,2,BUF,8);
		fx = a1 * powl(y,b1); //��ϼ���
		f = (int)(fx + 0.5) > (int)fx ? (int)fx + 1 : (int)fx;//��������ת����ʽ
		sprintf(BUF,"fx:%.3f",fx);
		OLED_ShowString(0,4,BUF,8);
	}
	else if (y>a[3]-(a[3]-a[4])/2 && y<a[2]+(a[1]-a[2])/2)//         11~20
	{
		sprintf(BUF,"a2:%.3f",a2/10000000);
		OLED_ShowString(0,1,BUF,8);
		sprintf(BUF,"b2:%.3f",b2);
		OLED_ShowString(0,2,BUF,8);
		fx = a2 * powl(y, b2);
		sprintf(BUF,"fx:%.3f",fx);
		OLED_ShowString(0,4,BUF,8);
		f = (int)(fx + 0.5) > (int)fx ? (int)fx + 1 : (int)fx;
	}
	else if (y>a[5]-(a[5]-a[6])/2 && y<a[4]+(a[3]-a[4])/2)//      21~30
	{
		sprintf(BUF,"a3:%.3f",a3/1000000000);
		OLED_ShowString(0,1,BUF,8);
		sprintf(BUF,"b3:%.3f",b3);
		OLED_ShowString(0,2,BUF,8);
		fx = a3 * powl(y, b3);
		sprintf(BUF,"fx:%.3f",fx);
		OLED_ShowString(0,4,BUF,8);
		f = (int)(fx + 0.5) > (int)fx ? (int)fx + 1 : (int)fx;
	}
	else if (y<a[6]+(a[5]-a[6])/2)//      31~43
	{
		sprintf(BUF,"a4:%.3f",a4/1000000000);
		OLED_ShowString(0,1,BUF,8);
		sprintf(BUF,"b4:%.3f",b4);
		OLED_ShowString(0,2,BUF,8);
		fx = a4 * powl(y, b4);
		sprintf(BUF ,"fx:%.3f",fx);
		OLED_ShowString(0,4,BUF,8);
		f = (int)(fx + 0.5) > (int)fx ? (int)fx + 1 : (int)fx;
	}
	if (y>300.0)
	{
		f = 1;
	}
	return f;
}

