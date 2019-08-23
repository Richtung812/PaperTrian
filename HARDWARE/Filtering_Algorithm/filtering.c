#include "fdc2214.h"
#include <math.h>
#include "filtering.h"
#include "oled.h"
#include "key.h"
extern float a[8];
extern float b[50];
	
char BUF[64];//OLED显示缓冲区每行16个字符，共4行。
/*
取10个数平均值，平均滤波
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
中位值平均滤波算法
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
  // 从小到大采样（冒泡）
  for(j = 0; j < Median_FILTER_N - 1; j++) {
    for(i = 0; i < Median_FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  // 去除极值后求平均
  for(i = 1; i < Median_FILTER_N - 1; i++) filter_sum += filter_buf[i];
  return filter_sum / (Median_FILTER_N - 2);
}

/*
递推平均滤波（滑动平均滤波）
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
限幅消抖滤波 (no!)
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
卡尔曼滤波
*/
//1. 结构体类型定义
//typedef struct 
//{
//    float LastP;//上次估算协方差 初始化值为0.02
//    float Now_P;//当前估算协方差 初始化值为0
//    float out;//卡尔曼滤波器输出 初始化值为0
//    float Kg;//卡尔曼增益 初始化值为0
//    float Q;//过程噪声协方差 初始化值为0.001
//    float R;//观测噪声协方差 初始化值为0.543
//}KFP;//Kalman Filter parameter

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanFilter(KFP *kfp,int ch)
{
		 float input;
		 input= Cap_Calculate(ch);
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
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
	//函数fabs：求浮点数x的绝对值
    //说明：计算|x|, 当x不为负时返回 x，否则返回 -x      
	}while(fabs(x-x0)>=1e-5);
 	return 0 ;
}


int get_paper_mi(float  y)
{
	//前10个数据拟合系数
	double a1 = 1.07e+05;//83.24;
	double b1 = -1.979;
	//中间10个数据拟合系数
	double a2 = 2.733e+07;
	double b2 = -3.16;
	//后10个数据拟合系数
	double a3 = 4.516e+09;
	double b3 = -4.299;
	//后30+个数据拟合系数
	double a4 = 3.321e+11;
	double b4 = -5.28;
	//40-50个数据拟合系数
	double a5 = log(y)-log(75.969);
	double b5 = -0.004;
	//50-60个数据拟合系数
	double a6 = log(y)-log(73.287);
	double b6 = -0.002;
	//60-65个数据拟合系数
	double a7 = log(y)-log(71.262);
	double b7 = -0.003;
	
	double fx; //带有小数点的纸张数
	int f=0;//整数的纸张数
	if (y>a[1]-(a[1]-a[2])/2 && y<a[0]+1)//                1~10
	{
		sprintf(BUF,"a1:%.3f",a1/100000);
		OLED_ShowString(0,1,BUF,8);
		sprintf(BUF,"b1:%.3f",b1);
		OLED_ShowString(0,2,BUF,8);
		fx = a1 * powl(y,b1); //拟合计算
		f = (int)(fx + 0.5) > (int)fx ? (int)fx + 1 : (int)fx;//四舍五入转换公式
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

