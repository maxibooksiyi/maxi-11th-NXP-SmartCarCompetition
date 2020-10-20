#ifndef _CONTROL_h
#define _CONTROL_h
#include "stdbool.h"

#define LeftMaxAngle  -3300//左转
#define RightMaxAngle 3300//右转

extern float OutData[4];           //外部用于蔽障显示

void Steer_adjust(void);              //舵机中值调节
void SC_black_Init(void);             //最大值采样
void get_speed(void);                 //获取速度
extern void Read_ADC(void);          //AD采集
extern void Read_powerU(void);         //电源电压采集
extern void Read_powerI(void);
extern void Date_analyse();           //数据分析
extern void direction();              //方向判断
extern void Error_analyse();           //数据分析
extern void motor(void);              //电机控制
extern void speed_set();              //速度的设定
extern void motorPI(void);            //增量PI典型电机控制
extern void motorPID(void);           //位置式PID电机控制
extern void motorstop(void);          //停车程序
extern void SpeedPID_analyse(void);   //智能PI
extern void Error_end(void);          //偏差的最终整合
extern void servoPD(void);            //舵机P，D
extern void Steer_angle(void);        //舵机转角计算
extern void Steer(void);              //舵机控制
extern void getAD(void);
void track(void);                      //赛道
void startrun(void);
extern void AtoB();                           //两车距离
int16 Least_squares(int16 start,int16 N,int16*buff,int16 x);//一阶线性拟合
void send();                       //发送数据
void pre_show(void);
void redraw();
void KeyScan(void); 
int16 abs(int16 x);
float abs_f(float x);
double ads_d(double x);
void  delay(void);          //延时约500ms
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