#ifndef _CONTROL_h
#define _CONTROL_h
#include "stdbool.h"

#define LeftMaxAngle  -3300//��ת
#define RightMaxAngle 3300//��ת

extern float OutData[4];           //�ⲿ���ڱ�����ʾ

void Steer_adjust(void);              //�����ֵ����
void SC_black_Init(void);             //���ֵ����
void get_speed(void);                 //��ȡ�ٶ�
extern void Read_ADC(void);          //AD�ɼ�
extern void Read_powerU(void);         //��Դ��ѹ�ɼ�
extern void Read_powerI(void);
extern void Date_analyse();           //���ݷ���
extern void direction();              //�����ж�
extern void Error_analyse();           //���ݷ���
extern void motor(void);              //�������
extern void speed_set();              //�ٶȵ��趨
extern void motorPI(void);            //����PI���͵������
extern void motorPID(void);           //λ��ʽPID�������
extern void motorstop(void);          //ͣ������
extern void SpeedPID_analyse(void);   //����PI
extern void Error_end(void);          //ƫ�����������
extern void servoPD(void);            //���P��D
extern void Steer_angle(void);        //���ת�Ǽ���
extern void Steer(void);              //�������
extern void getAD(void);
void track(void);                      //����
void startrun(void);
extern void AtoB();                           //��������
int16 Least_squares(int16 start,int16 N,int16*buff,int16 x);//һ���������
void send();                       //��������
void pre_show(void);
void redraw();
void KeyScan(void); 
int16 abs(int16 x);
float abs_f(float x);
double ads_d(double x);
void  delay(void);          //��ʱԼ500ms
void  delayms(uint32  ms);
int   cyc(int a,int b);
float fade(float x,float y);
float fadd(float x,float y);
int   ade(int x,int y);
void mmagyfirst();
void body();
void mpu6050init();
void mmainit();
#endif