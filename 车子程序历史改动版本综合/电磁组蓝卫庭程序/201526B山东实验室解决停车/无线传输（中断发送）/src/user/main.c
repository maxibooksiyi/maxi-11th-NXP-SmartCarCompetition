//�̶�ͷ�ļ�
#include "include.h"
/*************************************************
 �װ�����
 ����ʱ����ϸ���ݼ�K60�ʼ�
 ��ͬ�㷨���Կ���control�н��м���
*************************************************/

void PIT1_IRQHandler(void);                    //�ܿ���
void PORTB_IRQHandler(void);                   //����������ж�
void PORTE_IRQHandler(void);                 //�ɻ��ⲿ�ж�
void USART1_IRQHandler(void);
void system_init(void);//�ܽų�ʼ������غ�����ʼ������
uint16 nsteer,ledn,countbody,countPI;
extern int8 send_flag;
int8 sendstop;
int8 speake;
char runU,stopU;
extern int8    front_tested_flag;
extern int8    behind_tested_flag;
extern uint32 stoptime,runtime,starttime=2800;
extern int8 enablestop;
extern int8 goflag,runflag;
extern int8 stopflag,stopflag1,stopflag2;
extern float  vn;
extern double Distance;//���˶�·��
extern double Direction;//��ת���ĽǶ�
extern double stopdistance;//�յ�ͣ������
volatile u32 i;
u8 status;	//�����жϽ���/����״̬
u8 txbuf[5]={0,1,2,3,4};	 //���ͻ���
u8 rxbuf[5];
u8 datago[1]={88};
u8 datastop[1]={9};
void main(void)
{  
   system_init(); 
   EnableInterrupts;      //�����ж�    
   while(1)     
   {    
        

        Read_powerU();      
        pre_show();                    
        redraw();//ˢ��if(K6)
        if(K6)
        {
            KeyScan();//����ɨ��
            enablestop=0;
        } 
        else
        {
           startrun();//�������´�������ʱ2s����
        }
        if(send_flag==1)
        {
          send(); 
          send_flag=0;
        }
   }  
}
/*******************************
�������ƣ�PIT0_IRQHandler
����˵����PIT0 ��ʱ�жϷ�����
*******************************/
void PIT1_IRQHandler(void)                    //�ܿ���400us
{    
    ++nsteer;
    ++countbody;
    ++countPI;
    ledn++;   
    Date_analyse();//���ݹ�һ���ͼ򵥴���240us
    Error_analyse();//·����ȡ��error����
    //track();
    Error_end();
    servoPD();
    Steer_angle();//������ת��
    get_speed();
    speed_set();
    body();
    if(goflag==1&&runflag==0)
    {   
        Distance=0;//�������ܣ�·������
        if(starttime>400)
        {
            starttime--; 
            if(starttime>2300)
            {speaker(1);}
            else speaker(0);
            status = NRF_ISR_Rx_Dat(rxbuf); //�жϽ�������
             if(status ==  RX_DR)
            {
                 //status = NRF_ISR_Rx_Dat(datago); //�жϽ�������
                 if(status ==  RX_DR)
                 {
		     if(rxbuf[0]==0)
                    {  
                    runflag=1;goflag=0;starttime=2800;
                    
                    }
                 }
            }
        }
        else if(starttime<=400)
        {
          starttime--;
          speaker(1);
          if(starttime<=200)
          {   speaker(0);
              runflag=1;goflag=0;starttime=2800;
          }
        }
        
    }
    if(runflag==1)//�ٰ������ſ�����һ�η���
    { 
       if(runtime>0)
       {   
           motorPI();
          // motor(); 
           stopflag=0;
           runtime--;

       }
       else
       {   
           runflag=0;
           stopflag=1;         
           runtime=40000;         
       }    
    }
    if(stopflag==1)
    {
       motorstop();
    }
    if(nsteer==1){;}
    if(nsteer==2){;}
    if(nsteer>=3) {nsteer=0;Steer(); }//�������    
      
    if(ledn<500){led4(0);}
    else if(ledn>=500&&ledn<1000){led4(1);}
    else ledn=0;
    send_flag=1;
    PIT_Flag_Clear(PIT1);             //���жϱ�־λ 
}
void PORTB_IRQHandler()
{ 
   
//  if(PORTB_ISFR & (1<<11))//��⵽IO���Ǹߵ�ƽ����ô����������
//  {//������������ж�
//    PORTB_ISFR  |= (1 << 11);        //д1���жϱ�־λ
//    if(gpio_get(PORTB,11) == 1)
//    {
//        PIT_TCTRL(PIT0) &= ~(PIT_TCTRL_TEN_MASK);                     //����PIT ���Ա����ü���ֵ��Ч
//        PIT_LDVAL(PIT0)  = ~0;                                          //��������ж�ʱ��
//        PIT_TCTRL(PIT0) &= ~ PIT_TCTRL_TEN_MASK;                        //��ֹPITn��ʱ����������ռ���ֵ��
//        PIT_TCTRL(PIT0)  = ( 0| PIT_TCTRL_TEN_MASK  );                  //ʹ�� PITn��ʱ��
//        led3(0);
//     }
//     else 
//    { 
//        chaoshengboTime = ((~0)-PIT_CVAL(PIT0))/(core_clk_mhz / (mcg_div.bus_div + 1));//50M����ʱ�ӣ�����õ�ʱ�䣬��λ��΢��
//        ABDistance = chaoshengboTime * 340/10000;//һ���ӵ������ٶȼ���Ϊ340�ף�����chaoshengboTime��λ��΢�룬/10000��õ���λ��cm 
//        PIT_TCTRL(PIT0)  &= ~PIT_TCTRL_TEN_MASK;//ֹͣ��ʱ��
//    }
//  }
//  else 
   if(PORTB_ISFR & (1 << 9))
  {  
      PORTB_ISFR  |= (1 << 9);        //д1���жϱ�־λ
      NRF_Handler();
  }
}
void PORTE_IRQHandler()
{   
    
    
    if((PORTE_ISFR & (1 << 25))||(PORTE_ISFR & (1 << 27)))           //PTE25�����ж�
    {   
        PORTE_ISFR  |= (1 << 25);        //д1���жϱ�־λ
        /*  ����Ϊ�û�����  */      
        PORTE_ISFR  |= (1 << 27);        //д1���жϱ�־λ
        /*  ����Ϊ�û�����  */  
        if((gpio_get(PORTE,25) == 0)||(gpio_get(PORTE,27) == 0))
        {
           if(Distance>4000)
           {   
               enablestop=1;
               stopdistance=Distance;
           }
           else 
           { enablestop=0;}
        }

    }
    
}
void USART1_IRQHandler()
{
}
void system_init(void)
{   
    DisableInterrupts; //�����ж�  ��ֹϵͳ��ʼ�������   
    IO_init();                 //gpio�ڳ�ʼ��,��define.h��
    
    adc_init(ADC1,AD8);        //PB0        
    adc_init(ADC1,AD9);        //PB1         
    adc_init(ADC0,AD12);       //PB2         
    adc_init(ADC0,AD13);       //PB3         
    adc_init(ADC1,AD10);       //PB4         
    adc_init(ADC1,AD11);       //PB5         
    adc_init(ADC1,AD12);       //PB6         
    //adc_init(ADC1,AD13);       //PB7         
    //adc_init(ADC1,AD14);       //PB10        
    //adc_init(ADC1,AD15);       //PB11        
    
    //adc_init(ADC1,AD16);       //ADC1_SE16   ��Դ��ѹ�ɼ�
    adc_init(ADC0,AD14);       //ADC0_SE14   ���������ɼ�
    //exti_init(PORTB,11,either_up);
    //enable_irq(PORTB+87);
    exti_init(PORTE,25,falling_up);   //�ɻɹ�,�ڲ����������͵�ƽ
    exti_init(PORTE,27,falling_up);   //�ɻɹ�,�ڲ����������͵�ƽ
    enable_irq(PORTE+87);         //ʹ��     ����2m��,ע����Գ������ͻ
    FTM1_PWM_init();//�����ʼ�� ���ڲ���������
    MOTOR_init();                //���������ʼ��
    LCD_Init();
    //UART_IRQ_DIS(UART1);                          //����1   �ؽ����ж�
    uart_init(UART1,9600);
    
    pit_init_ms(PIT1,1);           //PIT 1MS��ʱ�źŲɼ�Ƶ�� //������� = ����Ƶ�� * ʱ��     1/100.000.000  *  50000=1ms      *50=1us         *5000=100us
    enable_irq(PIT1+68);             //ʹ��   
    //pit_init_ms(PIT1,1);          //1ms  ������
    //enable_irq(PIT1+69);  
    FTM2_QUAD_init();         //FTM2��������A10,11
    flash_init();	     //��ʼ��flash 
    //delayms(50);	     //�ϵ���ʱ
    I2C_init(I2C1);
    delayms(10);
    mmainit();delayms(10);
    mpu6050init();          //��ʼ��MPU6050
    delayms(10);
   //�����жϼ���   
    set_irq_priority (PIT1+68, 1);     //�ܿ��Ƴ���   
   //set_irq_priority (PIT0+68, 0);     //������ȴ��ڲɼ�
    //set_irq_priority (PORTB+87, 0);    //���������
    set_irq_priority (PORTE+87, 0);    //���ܼ�� 
    //set_irq_priority (UART1+47, 2);
//   delayms(50);    
     Steer_adjust(); delayms(10);         //�����ֵ����   �Զ�     
     SC_black_Init(); delayms(10);        //������ֵ���������ڹ�һ����  ��̬�궨   K1 K2 
     mmagyfirst();delayms(10);
     //UART_IRQ_EN(UART1);                           //����1   �������ж�
    NRF_Init();
    delayms(10);
    Distance=0;
}
