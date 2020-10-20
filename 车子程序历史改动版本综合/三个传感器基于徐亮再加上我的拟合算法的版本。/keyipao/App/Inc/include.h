#ifndef __INCLUDE_H__
#define __INCLUDE_H__

//1 头文件
//1.1通用头文件
#include  "common.h"   //通用函数头文件

//1.2包含面向硬件对象头文件(即构件模块) 


#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO口操作
#include  "MK60_uart.h"     //串口
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //低功耗定时器(延时)
#include  "MK60_i2c.h"      //I2C
#include  "MK60_spi.h"      //SPI
#include  "MK60_ftm.h"      //FTM
#include  "MK60_pit.h"      //PIT
#include  "MK60_rtc.h"      //RTC
#include  "MK60_adc.h"      //ADC
#include  "MK60_FLASH.h"    //FLASH
#include  "MK60_it.h"
#include  "stdbool.h"

#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_MMA7455.h"      //三轴加速度MMA7455
#include  "VCAN_NRF24L0.h"      //无线模块NRF24L01+
#include  "VCAN_RTC_count.h"    //RTC 时间转换

#include  "LDC1000.h"


//2 宏定义
  
   





#endif  //__INCLUDE_H__
