/*********************************
 任务：通用参数头文件
*********************************/

#ifndef _DEFINE_H_
#define _DEFINE_H_
#include "FTM.h"
//flash数据定义
#define	SECTOR_ADM	    252	        //AD垂直
#define	SECTOR_ADD	    251        //AD水平，斜角
#define SECTOR_TURN         250        //舵机中值 扇区

//舵机参数配置及控制定义

#define   SERVO_LEFT_MAX_PWM   13000   //舵机左打最大角度时的PWM
#define   SERVO_RIGHT_MAX_PWM  19800   //这个值需要自己根据舵机实际角度调试
#define   SERVO_MID_PWM        16400  //舵机中值16560   左右-3200——+3200

#define  Steer_init()     FTM1_PWM_init();      //转向舵机  300HZ     PA8 
#define  Steer_duty(x)    FTM1_PWM_Duty_STEER(x);             //舵机占空比

#define  Motor_go(x)      FTM0_PWM_Duty_MOTOR(x,0);            //电机前进
                          
#define  Motor_back(x)    FTM0_PWM_Duty_MOTOR(0,x);            //电机后退
                          
#define  Motor_stop()     FTM0_PWM_Duty_MOTOR(0,0);           //电机停止

                                               
//电机使能定义     电机10Khz
#define MOTOR_init()      FTM0_PWM_init_MOTOR(10000,0,0);
                                                

//编码器线数定义
#define  CODER    500
//GPIO口初始化
//6路拨码是上拉状态，打开就接地
//6路按键                             
//4路LED信号灯
//电平翻转口，E8,9                             
                                                     
#define  IO_init()      gpio_init(PORTB,16,GPI_UP,0);   \
                        gpio_init(PORTB,17,GPI_UP,0);   \
                        gpio_init(PORTB,18,GPI_UP,0);   \
                        gpio_init(PORTB,19,GPI_UP,0);   \
                        gpio_init(PORTB,20,GPI_UP,0);   \
                        gpio_init(PORTB,21,GPI_UP,0);   \
                        gpio_init(PORTD,6,GPI_UP,0);    \
                        gpio_init(PORTD,7,GPI_UP,0);    \
                        gpio_init(PORTD,8,GPI_UP,0);    \
                        gpio_init(PORTD,9,GPI_UP,0);    \
                        gpio_init(PORTD,10,GPI_UP,0);   \
                        gpio_init(PORTD,11,GPI_UP,0);   \
                        gpio_init(PORTA,17,GPO,1);   \
                        gpio_init(PORTC,0,GPO,1);    \
                        gpio_init(PORTD,15,GPO,1);   \
                        gpio_init(PORTE,26,GPO,1);   \
                        gpio_init(PORTE,8,GPO,0); 
                      
//拨码  
#define  K1          !gpio_get(PORTB,16)       //水平，斜角电感最大值采样     
#define  K2          !gpio_get(PORTB,17)
#define  K3          !gpio_get(PORTB,18)                        
#define  K4          !gpio_get(PORTB,19)      //                                     
#define  K5          !gpio_get(PORTB,20)      //                    
#define  K6          !gpio_get(PORTB,21)      //                                  
//按键                       
#define  Key1          !gpio_get(PORTD,6)      //          
#define  Key2          !gpio_get(PORTD,7)      //
#define  Key3          !gpio_get(PORTD,8)       //
#define  Key4          !gpio_get(PORTD,9)      //
#define  Key5          !gpio_get(PORTD,10)     //
#define  Key6          !gpio_get(PORTD,11)      //

                        
#define  qipao         gpio_get(PORTA,0)        //A0口电平，起跑线检测
                                                                                                             
//4个信号灯
#define   led1(x)        gpio_set(PORTA,17,x);       //停3s
#define   led2(x)        gpio_set(PORTC,0,x);        //试车程序
#define   led3(x)        gpio_set(PORTD,15,x);
#define   led4(x)        gpio_set(PORTE,26,x);       //控制中断         
#define   speaker(x)     gpio_set(PORTE,8,x);        //蜂鸣器        
                    
#endif 