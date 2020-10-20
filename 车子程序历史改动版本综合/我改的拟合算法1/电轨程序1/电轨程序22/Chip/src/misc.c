/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       misc.c
 * @brief      ɽ��K60 ƽ̨������Ҫ�õ��ĺ�����ʵ��
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */

#include "common.h"
#include "MK60_uart.h"


/*!
 *  @brief      �����ж��������ַ
 *  @param      vtor    �µ��ж��������ַ
 *  @since      v5.0
 *  @author     ��˼������˾
 *  Sample usage:       write_vtor ((uint32)__VECTOR_RAM);  //�µ��ж�������ַ
 */
void write_vtor (int vtor)
{
    ASSERT(vtor % 0x200 == 0);   //Vector Table base offset field. This value must be a multiple of 0x200.

    /* Write the VTOR with the new value */
    SCB->VTOR = vtor;
}



/*!
 *  @brief      �����ж�����������жϸ�λ����
 *  @since      v5.0
 *  @warning    ֻ���ж�������λ��icfָ����RAM����ʱ���˺�������Ч
 *  Sample usage:       set_vector_handler(UART3_RX_TX_VECTORn , uart3_handler);    //�� uart3_handler ������ӵ��ж�������
 */
void set_vector_handler(VECTORn_t vector , void pfunc_handler(void))
{
    extern uint32 __VECTOR_RAM[];

    ASSERT(SCB->VTOR == (uint32)__VECTOR_RAM);  //���ԣ�����ж��������Ƿ��� RAM ��

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
    //��λSLEEPDEEP��ʹ��STOPģʽ
    SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
    //����STOPģʽ
    asm("WFI");
}

//-------------------------------------------------------------------------*
//������: wait                                                             *
//��  ��: ����CPUΪWAITģʽ                                                * 
//��  ��: ��								   *	
//��  ��: ��                                                               *
//˵  ��: ��                                                               *
//-------------------------------------------------------------------------*
void wait (void)
{
    //��SLEEPDEEPλ��ȷ������WAITģʽ
    SCB_SCR &= ~SCB_SCR_SLEEPDEEP_MASK;	
    //����WAITģʽ
    asm("WFI");
}


//-------------------------------------------------------------------------*
//������: enable_irq                                                       *
//��  ��: ʹ��irq�ж�                                                      * 
//��  ��: irq:irq��       						   *	
//��  ��: ��                                                               *
//˵  ��: irq�Ų����ж�������                                              *
//-------------------------------------------------------------------------*
void enable_irq (int irq)
{
    int div;

    //ȷ��irq��Ϊ��Ч��irq��
    if (irq > 91)	irq=91;
    
    //ȷ����Ӧ��NVICISER
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
//������: disable_irq                                                      *
//��  ��: ��ֹirq�ж�                                                      * 
//��  ��: irq:irq��       						   *	
//��  ��: ��                                                               *
//˵  ��: irq�Ų����ж�������                                              *
//-------------------------------------------------------------------------*
void disable_irq (int irq)
{
    int div;
    
    //ȷ��irq��Ϊ��Ч��irq��
    if (irq > 91)	irq=91;
    
    //ȷ����Ӧ��NVICISER
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
//������: set_irq_priority                                                 *
//��  ��: ����irq�жϺ����ȼ�                                              * 
//��  ��: irq:irq��         						   *	
//        prio:���ȼ�						           *	
//��  ��: ��                                                               *
//˵  ��: irq�Ų����ж�������                                              *
//-------------------------------------------------------------------------*
/*  ����IRQ���ж����ȼ�   set_irq_priority (83,5);����83�ж����ȼ�Ϊ5��ע��ԽС���ȼ�Խ��    */
void set_irq_priority (int irq, int prio)
{
    uint8 *prio_reg;

    //ȷ��irq�ź����ȼ���Ч
    if (irq > 91)	irq=91;
    if (prio > 15)	prio=15;

    //ȷ����Ӧ��NVICISER
    prio_reg = (uint8 *)(((uint32)&NVICIP0) + irq);
    //�������ȼ�
    *prio_reg = ( (prio&0xF) << (8 - ARM_INTERRUPT_LEVEL_BITS) );             
}
