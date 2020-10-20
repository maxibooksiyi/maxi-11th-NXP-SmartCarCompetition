/********************  ********************
 * �ļ���       ��include.h
 * ����         ������ģ��ͷ�ļ�

**********************************************************************************/

#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include <math.h>
#include  "arm_math.h"  //DSP��
#include  "adc.h"       //ADCģ��
#include  "dma.h"       //DMAģ��
#include  "exti.h"      //EXTI�ⲿGPIO�ж�
#include  "flash.h"     //flash����
#include  "FTM.h"       //FTMģ�飨FTM0��������� / ͨ�� /PWM     FTM1��2���������� / ͨ�� /PWM ��
#include  "gpio.h"      //IO�ڲ���
#include  "gpio_cfg.h"
#include  "OLED.h"      //OLED��ʾ
#include  "lptmr.h"     //�͹��Ķ�ʱ��(��ʱ)
#include  "uart.h"      //����
#include  "i2c.h"       //IICͨ��
#include  "mcg.h"       //��Ƶʱ��
#include  "sdhc.h"      //SDHC�ڴ濨ģ��
#include  "ff.h"        //FAT32����
#include  "diskio.h"
#include  "NRF24L0.h"   //NRF24L0����ͨ��ģ��
#include  "PIT.h"       //�����жϼ�ʱ��
#include  "spi.h"       //SPIͨѶ
//#include  "mpu6050.h"   //����ģ��
#include  "control.h"   //�����㷨
#include  "define.h"    //����
#include  "Data_send.h" //����͸��ʾ����
   
extern uint32 chaoshengboTime;//��ȡ���ĳ�����ʱ��
extern uint32 ABDistance;//�����ķ��ͽ���ģ��ľ���


#endif  //__INCLUDE_H__
