/*******************************************************************************
 任务说明：重新定义中断向量表
          先取消默认的中断向量元素宏定义       #undef  VECTOR_xxx
          再重新定义到自己编写的中断函数       #define VECTOR_xxx    xxx_IRQHandler
   例如：
        #undef  VECTOR_003
        #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
 团队：首安六队
 时间：2014,01,08
*********************************************************************************/ 
#ifndef	__ISR_H
#define	__ISR_H 1

#include  "include.h" 
//#undef  VECTOR_003
//#define VECTOR_003    HardFault_Handler   //发生硬件上访 （就是程序跑飞了）     推荐调试程序时，用个LED显示来指示程序跑飞了


#undef  VECTOR_084                        //内部中断   计数溢出中断，
#define VECTOR_084    PIT0_IRQHandler


#undef  VECTOR_085                        //内部中断   计数溢出中断，
#define VECTOR_085    PIT1_IRQHandler


#undef  VECTOR_107
#define VECTOR_107    PORTE_IRQHandler    //PORTE25中断

/*
#undef  VECTOR_086                        //内部中断   计数溢出中断，
#define VECTOR_086    PIT2_IRQHandler

*/
#undef  VECTOR_063                        //要先取消了，因为在vectors.h里默认是定义为 default_isr  
#define VECTOR_063    USART1_IRQHandler   //重新定义63号中断的ISR：UART1：Single interrupt vector for UART status sources



/*
#undef  VECTOR_103
#define VECTOR_103    PORTA_IRQHandler    //PORTA中断
*/
#undef  VECTOR_104
#define VECTOR_104    PORTB_IRQHandler    //PORTB中断

//extern void HardFault_Handler(void);      //发生硬件上访（程序跑飞）

extern void PIT0_IRQHandler();
extern void PIT1_IRQHandler();
extern void PORTE_IRQHandler();
extern void USART1_IRQHandler();          //串口1 中断接收函数
//extern void PORTA_IRQHandler();
 
extern void PORTB_IRQHandler();




#endif 
	