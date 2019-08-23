#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "led.h"
#include "oled.h"
#include "filtering.h"
#include "fdc2214.h"
#include "exti.h"
#include "beep.h"
//#include "McuDataSend.h"

void Sys_Init(void);
void calibration(void);
int read_paper(float max,float min,float max_paper,float now_res);
//u32 i=0;
float res0,res1,res2,res3,min_num=0,max_num=0;
float ave[4]={0.0};//����ƽ��ֵ����
int warn_start,key,paper_num=0;
char OLED_BUF[64];//OLED��ʾ������ÿ��16���ַ�����4�С�
u8 Paper_buf[]={1,10,11,20,21,30,31,43};//������ʾ��һ��Ҫ�ŵ�ֽ����
int numstar=0;

float a[8]={0};//��������

int kb=50;//ѧϰ�ɼ�������������
float b[50]={0};//ѧϰ�ɼ�������

int main(void)
{
	int temp,t=0,i=0,h=0,num;//�������������ת��ֵ
	float sum=0;
	warn_start=0;
  Sys_Init();
	//sprintf(OLED_BUF,"No: %d",i);
	OLED_ShowString(0,0,"learn will begin",8);
	delay_ms(1000); 
	OLED_Clear();
	//ѧϰ
	
	while (a[7]==0)//����4�����������
	{
		sprintf(OLED_BUF,"next paper: %d      ",Paper_buf[i]);
		OLED_ShowString(0,1,OLED_BUF,8);
		key=KEY_Scan(0);
		if (key==KEY0)
		{
			OLED_Clear();
			sum=0;
			for (h=0;h<kb;h++)
			{
				OLED_ShowString(0,0,"learning......",8);
				b[h]=Median_Filter(3);
				sum+=b[h];
			}
			
			a[i]=sum/kb;
			printf("%f,%d\r\n",a[i],i);
			sprintf(OLED_BUF,"No: %d           ",i);
			OLED_ShowString(0,0,OLED_BUF,8);
			sprintf(OLED_BUF,"ch3: %f          ",a[i]);
			OLED_ShowString(0,1,OLED_BUF,8);
			delay_ms(1000);
			OLED_Clear();
			i++;
		}
	}
	
	//ʵս
	while(1)
	{ 
	  res3 = Median_Filter(3);//�ɼ����ݣ�ԭʼ���ݣ�
		res3 = res3;//���ݽӿڿ��ؼ�����ʼֵ
		if (res3 <= 10.0){
			BEEP=1;
			OLED_Clear();
			OLED_ShowString(40,3,"Warning!",16);
			warn_start=1;
		}
		else
		{
			if (warn_start==1)
			{
				warn_start=0;
				BEEP=0;
				OLED_Clear();
			}
			paper_num=get_paper_mi(res3);
			sprintf(OLED_BUF,"CH3: %3.3f",res3);
			OLED_ShowString(0,0,OLED_BUF,8);
			key=KEY_Scan(0);
			if (key==KEY1)
			{				
				sprintf(OLED_BUF,"paper:%d  ",paper_num);
				OLED_ShowString(0,6,OLED_BUF,16);
				numstar=0;
				BEEP=1;
				delay_ms(100);
				BEEP=0;
			}
		}
		printf("%3.3f\r\n",res3);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	}
}

void Sys_Init(void)
{
	delay_init();	 
	NVIC_Configuration();
	uart_init(115200);	 //���ڲ�����115200 
	LED_Init();
	BEEP_Init();
	KEY_Init();
	OLED_Init();
	OLED_Clear();//����
	EXTIX_Init();         	//��ʼ���ⲿ�ж�����
	while(FDC2214_Init());
	led=0;
}

int read_paper(float max,float min,float max_paper,float now_res){
	float difference;
	float value;
	int paper_nums;
	value=max-min;
	difference = value / (max_paper-1);//(max_paper-1��)
	paper_nums=(int)((max-now_res)/difference);
	return paper_nums+1;
}
