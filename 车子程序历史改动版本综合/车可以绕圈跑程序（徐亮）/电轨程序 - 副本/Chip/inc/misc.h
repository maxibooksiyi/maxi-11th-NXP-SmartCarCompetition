/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       misc.h
 * @brief      ɽ��K60 ƽ̨������Ҫ�õ��ĺ�����ʵ��
 * @author     ɽ��Ƽ�
 * @version    v5.1
 * @date       2014-04-25
 */

#ifndef __MISC_H__
#define __MISC_H__

#include "common.h"

#define ARM_INTERRUPT_LEVEL_BITS     4//�ж����ȼ��궨�� 
#define EnableInterrupts asm(" CPSIE i");//�����ж�
#define DisableInterrupts asm(" CPSID i");//�����ж�

void write_vtor (int);                                              //�����ж�������ĵ�ַ
void set_vector_handler(VECTORn_t , void pfunc_handler(void));      //�����жϺ������ж���������             
void stop (void);//����CPUΪSTOPģʽ  
void wait (void);//����CPUΪWAITģʽ
void enable_irq (int);//ʹ��irq�ж�
void disable_irq (int);//��ֹirq�ж� 
void set_irq_priority (int, int);//����irq�жϺ����ȼ� 
    
    
void vcan_cpy( uint8 *dst, uint8 *src, uint32 count);

#endif  /* __MISC_H__ */

