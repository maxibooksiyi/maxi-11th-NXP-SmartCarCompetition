/*********************************
 ����ͨ�ò���ͷ�ļ�
*********************************/

#ifndef _DEFINE_H_
#define _DEFINE_H_
#include "FTM.h"
//flash���ݶ���
#define	SECTOR_ADM	    252	        //AD��ֱ
#define	SECTOR_ADD	    251        //ADˮƽ��б��
#define SECTOR_TURN         250        //�����ֵ ����

//����������ü����ƶ���

#define   SERVO_LEFT_MAX_PWM   13000   //���������Ƕ�ʱ��PWM
#define   SERVO_RIGHT_MAX_PWM  19800   //���ֵ��Ҫ�Լ����ݶ��ʵ�ʽǶȵ���
#define   SERVO_MID_PWM        16400  //�����ֵ16560   ����-3200����+3200

#define  Steer_init()     FTM1_PWM_init();      //ת����  300HZ     PA8 
#define  Steer_duty(x)    FTM1_PWM_Duty_STEER(x);             //���ռ�ձ�

#define  Motor_go(x)      FTM0_PWM_Duty_MOTOR(x,0);            //���ǰ��
                          
#define  Motor_back(x)    FTM0_PWM_Duty_MOTOR(0,x);            //�������
                          
#define  Motor_stop()     FTM0_PWM_Duty_MOTOR(0,0);           //���ֹͣ

                                               
//���ʹ�ܶ���     ���10Khz
#define MOTOR_init()      FTM0_PWM_init_MOTOR(10000,0,0);
                                                

//��������������
#define  CODER    500
//GPIO�ڳ�ʼ��
//6·����������״̬���򿪾ͽӵ�
//6·����                             
//4·LED�źŵ�
//��ƽ��ת�ڣ�E8,9                             
                                                     
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
                      
//����  
#define  K1          !gpio_get(PORTB,16)       //ˮƽ��б�ǵ�����ֵ����     
#define  K2          !gpio_get(PORTB,17)
#define  K3          !gpio_get(PORTB,18)                        
#define  K4          !gpio_get(PORTB,19)      //                                     
#define  K5          !gpio_get(PORTB,20)      //                    
#define  K6          !gpio_get(PORTB,21)      //                                  
//����                       
#define  Key1          !gpio_get(PORTD,6)      //          
#define  Key2          !gpio_get(PORTD,7)      //
#define  Key3          !gpio_get(PORTD,8)       //
#define  Key4          !gpio_get(PORTD,9)      //
#define  Key5          !gpio_get(PORTD,10)     //
#define  Key6          !gpio_get(PORTD,11)      //

                        
#define  qipao         gpio_get(PORTA,0)        //A0�ڵ�ƽ�������߼��
                                                                                                             
//4���źŵ�
#define   led1(x)        gpio_set(PORTA,17,x);       //ͣ3s
#define   led2(x)        gpio_set(PORTC,0,x);        //�Գ�����
#define   led3(x)        gpio_set(PORTD,15,x);
#define   led4(x)        gpio_set(PORTE,26,x);       //�����ж�         
#define   speaker(x)     gpio_set(PORTE,8,x);        //������        
                    
#endif 