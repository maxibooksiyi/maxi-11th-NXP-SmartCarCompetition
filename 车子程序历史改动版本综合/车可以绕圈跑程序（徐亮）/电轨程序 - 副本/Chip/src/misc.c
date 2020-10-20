/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外初学论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       misc.c
 * @brief      山外K60 平台其他需要用到的函数的实现
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */

#include "common.h"
#include "MK60_uart.h"


/*!
 *  @brief      设置中断向量表地址
 *  @param      vtor    新的中断向量表地址
 *  @since      v5.0
 *  @author     飞思卡尔公司
 *  Sample usage:       write_vtor ((uint32)__VECTOR_RAM);  //新的中断向量地址
 */
void write_vtor (int vtor)
{
    ASSERT(vtor % 0x200 == 0);   //Vector Table base offset field. This value must be a multiple of 0x200.

    /* Write the VTOR with the new value */
    SCB->VTOR = vtor;
}



/*!
 *  @brief      设置中断向量表里的中断复位函数
 *  @since      v5.0
 *  @warning    只有中断向量表位于icf指定的RAM区域时，此函数才有效
 *  Sample usage:       set_vector_handler(UART3_RX_TX_VECTORn , uart3_handler);    //把 uart3_handler 函数添加到中断向量表
 */
void set_vector_handler(VECTORn_t vector , void pfunc_handler(void))
{
    extern uint32 __VECTOR_RAM[];

    ASSERT(SCB->VTOR == (uint32)__VECTOR_RAM);  //断言，检测中断向量表是否在 RAM 里

    __VECTOR_RAM[vector] = (uint32)pfunc_handler;
}

void vcan_cpy( uint8 *dst, uint8 *src, uint32 count)
{
    uint32 n;
    if(count != 0)
    {
        //printf("-");

        n = (count + 7 ) / 8 ;
        switch (count % 8 )
        {
            do
            {
            case 0 :
                * dst ++ = * src ++ ;
            case 7 :
                * dst ++ = * src ++ ;
            case 6 :
                * dst ++ = * src ++ ;
            case 5 :
                * dst ++ = * src ++ ;
            case 4 :
                * dst ++ = * src ++ ;
            case 3 :
                * dst ++ = * src ++ ;
            case 2 :
                * dst ++ = * src ++ ;
            case 1 :
                * dst ++ = * src ++ ;
            }
            while ( -- n >    0 );
        }
    }
}

void stop (void)
{
    //置位SLEEPDEEP来使能STOP模式
    SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
    //进入STOP模式
    asm("WFI");
}

//-------------------------------------------------------------------------*
//函数名: wait                                                             *
//功  能: 设置CPU为WAIT模式                                                * 
//参  数: 无								   *	
//返  回: 无                                                               *
//说  明: 无                                                               *
//-------------------------------------------------------------------------*
void wait (void)
{
    //清SLEEPDEEP位来确定进入WAIT模式
    SCB_SCR &= ~SCB_SCR_SLEEPDEEP_MASK;	
    //进入WAIT模式
    asm("WFI");
}


//-------------------------------------------------------------------------*
//函数名: enable_irq                                                       *
//功  能: 使能irq中断                                                      * 
//参  数: irq:irq号       						   *	
//返  回: 无                                                               *
//说  明: irq号不是中断向量号                                              *
//-------------------------------------------------------------------------*
void enable_irq (int irq)
{
    int div;

    //确定irq号为有效的irq号
    if (irq > 91)	irq=91;
    
    //确定对应的NVICISER
    div = irq/32;
    
    switch (div)
    {
    	case 0x0:
              NVICICPR0 = 1 << (irq%32);
              NVICISER0 = 1 << (irq%32);
              break;
    	case 0x1:
              NVICICPR1 = 1 << (irq%32);
              NVICISER1 = 1 << (irq%32);
              break;
    	case 0x2:
              NVICICPR2 = 1 << (irq%32);
              NVICISER2 = 1 << (irq%32);
              break;
    }              
}

//-------------------------------------------------------------------------*
//函数名: disable_irq                                                      *
//功  能: 禁止irq中断                                                      * 
//参  数: irq:irq号       						   *	
//返  回: 无                                                               *
//说  明: irq号不是中断向量号                                              *
//-------------------------------------------------------------------------*
void disable_irq (int irq)
{
    int div;
    
    //确定irq号为有效的irq号
    if (irq > 91)	irq=91;
    
    //确定对应的NVICISER
    div = irq/32;
    
    switch (div)
    {
    	case 0x0:
               NVICICER0 = 1 << (irq%32);
              break;
    	case 0x1:
              NVICICER1 = 1 << (irq%32);
              break;
    	case 0x2:
              NVICICER2 = 1 << (irq%32);
              break;
    }              
}
 
//-------------------------------------------------------------------------*
//函数名: set_irq_priority                                                 *
//功  能: 设置irq中断和优先级                                              * 
//参  数: irq:irq号         						   *	
//        prio:优先级						           *	
//返  回: 无                                                               *
//说  明: irq号不是中断向量号                                              *
//-------------------------------------------------------------------------*
/*  配置IRQ的中断优先级   set_irq_priority (83,5);设置83中断优先级为5，注意越小优先级越大    */
void set_irq_priority (int irq, int prio)
{
    uint8 *prio_reg;

    //确定irq号和优先级有效
    if (irq > 91)	irq=91;
    if (prio > 15)	prio=15;

    //确定对应的NVICISER
    prio_reg = (uint8 *)(((uint32)&NVICIP0) + irq);
    //设置优先级
    *prio_reg = ( (prio&0xF) << (8 - ARM_INTERRUPT_LEVEL_BITS) );             
}
