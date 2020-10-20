/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外初学论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       misc.h
 * @brief      山外K60 平台其他需要用到的函数的实现
 * @author     山外科技
 * @version    v5.1
 * @date       2014-04-25
 */

#ifndef __MISC_H__
#define __MISC_H__

#include "common.h"

#define ARM_INTERRUPT_LEVEL_BITS     4//中断优先级宏定义 
#define EnableInterrupts asm(" CPSIE i");//开总中断
#define DisableInterrupts asm(" CPSID i");//关总中断

void write_vtor (int);                                              //设置中断向量表的地址
void set_vector_handler(VECTORn_t , void pfunc_handler(void));      //设置中断函数到中断向量表里             
void stop (void);//设置CPU为STOP模式  
void wait (void);//设置CPU为WAIT模式
void enable_irq (int);//使能irq中断
void disable_irq (int);//禁止irq中断 
void set_irq_priority (int, int);//设置irq中断和优先级 
    
    
void vcan_cpy( uint8 *dst, uint8 *src, uint32 count);

#endif  /* __MISC_H__ */

