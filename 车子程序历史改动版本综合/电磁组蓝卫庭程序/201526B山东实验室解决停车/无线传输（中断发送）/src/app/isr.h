/*******************************************************************************
 ����˵�������¶����ж�������
          ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��       #undef  VECTOR_xxx
          �����¶��嵽�Լ���д���жϺ���       #define VECTOR_xxx    xxx_IRQHandler
   ���磺
        #undef  VECTOR_003
        #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 �Ŷӣ��װ�����
 ʱ�䣺2014,01,08
*********************************************************************************/ 
#ifndef	__ISR_H
#define	__ISR_H 1

#include  "include.h" 
//#undef  VECTOR_003
//#define VECTOR_003    HardFault_Handler   //����Ӳ���Ϸ� �����ǳ����ܷ��ˣ�     �Ƽ����Գ���ʱ���ø�LED��ʾ��ָʾ�����ܷ���


#undef  VECTOR_084                        //�ڲ��ж�   ��������жϣ�
#define VECTOR_084    PIT0_IRQHandler


#undef  VECTOR_085                        //�ڲ��ж�   ��������жϣ�
#define VECTOR_085    PIT1_IRQHandler


#undef  VECTOR_107
#define VECTOR_107    PORTE_IRQHandler    //PORTE25�ж�

/*
#undef  VECTOR_086                        //�ڲ��ж�   ��������жϣ�
#define VECTOR_086    PIT2_IRQHandler

*/
#undef  VECTOR_063                        //Ҫ��ȡ���ˣ���Ϊ��vectors.h��Ĭ���Ƕ���Ϊ default_isr  
#define VECTOR_063    USART1_IRQHandler   //���¶���63���жϵ�ISR��UART1��Single interrupt vector for UART status sources



/*
#undef  VECTOR_103
#define VECTOR_103    PORTA_IRQHandler    //PORTA�ж�
*/
#undef  VECTOR_104
#define VECTOR_104    PORTB_IRQHandler    //PORTB�ж�

//extern void HardFault_Handler(void);      //����Ӳ���Ϸã������ܷɣ�

extern void PIT0_IRQHandler();
extern void PIT1_IRQHandler();
extern void PORTE_IRQHandler();
extern void USART1_IRQHandler();          //����1 �жϽ��պ���
//extern void PORTA_IRQHandler();
 
extern void PORTB_IRQHandler();




#endif 
	