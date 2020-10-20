/********************  ********************
 * 文件名       ：include.h
 * 描述         ：工程模版头文件

**********************************************************************************/

#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */
#include <math.h>
#include  "arm_math.h"  //DSP库
#include  "adc.h"       //ADC模块
#include  "dma.h"       //DMA模块
#include  "exti.h"      //EXTI外部GPIO中断
#include  "flash.h"     //flash操作
#include  "FTM.h"       //FTM模块（FTM0：电机控制 / 通用 /PWM     FTM1、2：正交解码 / 通用 /PWM ）
#include  "gpio.h"      //IO口操作
#include  "gpio_cfg.h"
#include  "OLED.h"      //OLED显示
#include  "lptmr.h"     //低功耗定时器(延时)
#include  "uart.h"      //串口
#include  "i2c.h"       //IIC通信
#include  "mcg.h"       //主频时钟
#include  "sdhc.h"      //SDHC内存卡模块
#include  "ff.h"        //FAT32磁盘
#include  "diskio.h"
#include  "NRF24L0.h"   //NRF24L0无线通信模块
#include  "PIT.h"       //周期中断计时器
#include  "spi.h"       //SPI通讯
//#include  "mpu6050.h"   //六轴模块
#include  "control.h"   //控制算法
#include  "define.h"    //参数
#include  "Data_send.h" //无线透传示波器
   
extern uint32 chaoshengboTime;//读取到的超声波时间
extern uint32 ABDistance;//换算后的发送接收模块的距离


#endif  //__INCLUDE_H__
