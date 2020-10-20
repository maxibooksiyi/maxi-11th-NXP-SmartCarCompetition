//固定头文件
#include "include.h"
/*************************************************
 首安六队
 调试时，详细内容见K60笔记
 不同算法尝试可在control中进行即可
*************************************************/

void PIT1_IRQHandler(void);                    //总控制
void PORTB_IRQHandler(void);                   //超声波检测中断
void PORTE_IRQHandler(void);                 //干簧外部中断
void USART1_IRQHandler(void);
void system_init(void);//管脚初始化，相关函数初始化声明
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
extern double Distance;//总运动路程
extern double Direction;//总转过的角度
extern double stopdistance;//终点停车距离
volatile u32 i;
u8 status;	//用于判断接收/发送状态
u8 txbuf[5]={0,1,2,3,4};	 //发送缓冲
u8 rxbuf[5];
u8 datago[1]={88};
u8 datastop[1]={9};
void main(void)
{  
   system_init(); 
   EnableInterrupts;      //开总中断    
   while(1)     
   {    
        

        Read_powerU();      
        pre_show();                    
        redraw();//刷屏if(K6)
        if(K6)
        {
            KeyScan();//按键扫描
            enablestop=0;
        } 
        else
        {
           startrun();//开机按下触发键定时2s起跑
        }
        if(send_flag==1)
        {
          send(); 
          send_flag=0;
        }
   }  
}
/*******************************
函数名称：PIT0_IRQHandler
功能说明：PIT0 定时中断服务函数
*******************************/
void PIT1_IRQHandler(void)                    //总控制400us
{    
    ++nsteer;
    ++countbody;
    ++countPI;
    ledn++;   
    Date_analyse();//数据归一化和简单处理240us
    Error_analyse();//路况获取与error分析
    //track();
    Error_end();
    servoPD();
    Steer_angle();//计算舵机转角
    get_speed();
    speed_set();
    body();
    if(goflag==1&&runflag==0)
    {   
        Distance=0;//触发起跑，路程清零
        if(starttime>400)
        {
            starttime--; 
            if(starttime>2300)
            {speaker(1);}
            else speaker(0);
            status = NRF_ISR_Rx_Dat(rxbuf); //中断接收数据
             if(status ==  RX_DR)
            {
                 //status = NRF_ISR_Rx_Dat(datago); //中断接收数据
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
    if(runflag==1)//再按触发才可以又一次发车
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
    if(nsteer>=3) {nsteer=0;Steer(); }//舵机控制    
      
    if(ledn<500){led4(0);}
    else if(ledn>=500&&ledn<1000){led4(1);}
    else ledn=0;
    send_flag=1;
    PIT_Flag_Clear(PIT1);             //清中断标志位 
}
void PORTB_IRQHandler()
{ 
   
//  if(PORTB_ISFR & (1<<11))//检测到IO口是高电平，那么就是上升沿
//  {//如果是上升沿中断
//    PORTB_ISFR  |= (1 << 11);        //写1清中断标志位
//    if(gpio_get(PORTB,11) == 1)
//    {
//        PIT_TCTRL(PIT0) &= ~(PIT_TCTRL_TEN_MASK);                     //禁用PIT ，以便设置加载值生效
//        PIT_LDVAL(PIT0)  = ~0;                                          //设置溢出中断时间
//        PIT_TCTRL(PIT0) &= ~ PIT_TCTRL_TEN_MASK;                        //禁止PITn定时器（用于清空计数值）
//        PIT_TCTRL(PIT0)  = ( 0| PIT_TCTRL_TEN_MASK  );                  //使能 PITn定时器
//        led3(0);
//     }
//     else 
//    { 
//        chaoshengboTime = ((~0)-PIT_CVAL(PIT0))/(core_clk_mhz / (mcg_div.bus_div + 1));//50M总线时钟，计算得到时间，单位是微秒
//        ABDistance = chaoshengboTime * 340/10000;//一秒钟的声音速度假设为340米，由于chaoshengboTime单位是微秒，/10000后得到单位是cm 
//        PIT_TCTRL(PIT0)  &= ~PIT_TCTRL_TEN_MASK;//停止定时器
//    }
//  }
//  else 
   if(PORTB_ISFR & (1 << 9))
  {  
      PORTB_ISFR  |= (1 << 9);        //写1清中断标志位
      NRF_Handler();
  }
}
void PORTE_IRQHandler()
{   
    
    
    if((PORTE_ISFR & (1 << 25))||(PORTE_ISFR & (1 << 27)))           //PTE25触发中断
    {   
        PORTE_ISFR  |= (1 << 25);        //写1清中断标志位
        /*  以下为用户任务  */      
        PORTE_ISFR  |= (1 << 27);        //写1清中断标志位
        /*  以下为用户任务  */  
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
    DisableInterrupts; //关总中断  防止系统初始化被打断   
    IO_init();                 //gpio口初始化,在define.h中
    
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
    
    //adc_init(ADC1,AD16);       //ADC1_SE16   电源电压采集
    adc_init(ADC0,AD14);       //ADC0_SE14   驱动电流采集
    //exti_init(PORTB,11,either_up);
    //enable_irq(PORTB+87);
    exti_init(PORTE,25,falling_up);   //干簧管,内部上啦，检测低电平
    exti_init(PORTE,27,falling_up);   //干簧管,内部上啦，检测低电平
    enable_irq(PORTE+87);         //使能     起跑2m开,注意和试车程序冲突
    FTM1_PWM_init();//舵机初始化 ，在参数设置中
    MOTOR_init();                //电机驱动初始化
    LCD_Init();
    //UART_IRQ_DIS(UART1);                          //串口1   关接收中断
    uart_init(UART1,9600);
    
    pit_init_ms(PIT1,1);           //PIT 1MS定时信号采集频率 //溢出计数 = 总线频率 * 时间     1/100.000.000  *  50000=1ms      *50=1us         *5000=100us
    enable_irq(PIT1+68);             //使能   
    //pit_init_ms(PIT1,1);          //1ms  编码器
    //enable_irq(PIT1+69);  
    FTM2_QUAD_init();         //FTM2正交解码A10,11
    flash_init();	     //初始化flash 
    //delayms(50);	     //上电延时
    I2C_init(I2C1);
    delayms(10);
    mmainit();delayms(10);
    mpu6050init();          //初始化MPU6050
    delayms(10);
   //设置中断级别   
    set_irq_priority (PIT1+68, 1);     //总控制程序   
   //set_irq_priority (PIT0+68, 0);     //电机优先大于采集
    //set_irq_priority (PORTB+87, 0);    //超声波检测
    set_irq_priority (PORTE+87, 0);    //起跑检测 
    //set_irq_priority (UART1+47, 2);
//   delayms(50);    
     Steer_adjust(); delayms(10);         //舵机中值调节   自动     
     SC_black_Init(); delayms(10);        //电感最大值采样（用于归一化）  静态标定   K1 K2 
     mmagyfirst();delayms(10);
     //UART_IRQ_EN(UART1);                           //串口1   开接收中断
    NRF_Init();
    delayms(10);
    Distance=0;
}
