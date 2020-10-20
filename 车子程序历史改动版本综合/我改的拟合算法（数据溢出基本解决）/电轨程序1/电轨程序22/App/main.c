#include "include.h"
#include <math.h>


//按键
#define Go              (bool)(GPIOE_PDIR >> 28 & 0x00000001)
#define Change_page     (bool)(GPIOD_PDIR >> 0 & 0x00000001)
#define Add_line        (bool)(GPIOD_PDIR >> 2 & 0x00000001)
#define Sub_line        (bool)(GPIOD_PDIR >> 6 & 0x00000001)
#define Add             (bool)(GPIOD_PDIR >> 7 & 0x00000001)
#define Sub             (bool)(GPIOD_PDIR >> 10 & 0x00000001)

uint8 page_num=0;     //页序号
uint8 line_num=0;     //行序号
int32 mid_angle=5;
int32 top_k=2;
bool go;//发车标志

#define ERROR_HISTORY_NUM 5   //历史偏差记录个数
int worse=0;
int error=0;  //本次偏差
int error_history[ERROR_HISTORY_NUM]={0};  //历史偏差
uint8 servo_P=20;   //PID系数
uint8 servo_D=30;
int servo_middle=605;
int servo_PWM=0;//舵机PWM //超过char型变量的范围

int jiaodu;//我定义一个角度
float m;
int n;
float f;


char speed_expect=40 ; //给定速度
char real_speed=0;

extern int LDC_val1,LDC_val2;
extern signed int LDC_val;
extern uint8 proximtyData[2];

//***************************************************
//oled显示引用的函数
//***************************************************
extern void LCD_Init(void);
extern void LCD_clear_L(uint8 x,uint8 y);
extern void LCD_DLY_ms(uint16 ms);//屏幕延时去抖函数
extern void LCD_CLS(void);
extern void write_6_8_char(uint8 x,uint8 y,uint8 ucData) ;
extern void write_6_8_string(uint8 x,uint8 y,uint8 ch[]);
extern void write_6_8_number(uint8 x,uint8 y, float number);


void servo_control1(void);
int panduanweishu(int a);


void FLOAT_delay(unsigned int ms);
//***************************************************
//按键延时去抖函数
//***************************************************
void KEY_DLY_ms(unsigned int ms)
{
    unsigned int a;
    while(ms)
    {
    a=800;
    while(a--){};
    ms--;
    }
    return;
}

//***************************************************
//按键初始化
//***************************************************
void GIPO_init(void)
{
         gpio_init (PTD0, GPI_UP,1);
         gpio_init (PTD2,GPI_UP,1);
         gpio_init (PTD6, GPI_UP,1); 
         gpio_init (PTD7, GPI_UP,1);
         gpio_init (PTD10, GPI_UP,1); 
         gpio_init (PTE28, GPI_UP,1);
         
         FTM_PWM_init(FTM0,FTM_CH0,50,servo_middle);//舵机 PTC1
         FTM_PWM_init(FTM2,FTM_CH0,10000,0);//PTB18
         FTM_PWM_init(FTM2,FTM_CH1,10000,0);//PTB19

         
}
//***************************************************
//预显示
//***************************************************
void pre_show(void)
{  
  LCD_CLS();
   switch(page_num)
  { case 0: 
       if(line_num<=7)
          write_6_8_string(2,line_num+2,"*");
       else
           write_6_8_string(60,line_num-8,"*");
        
     write_6_8_number(10,0,(uint16)LDC_val1);
     write_6_8_number(10,1,(uint16)LDC_val2);
     
     write_6_8_string(6,2,"MID:");
     write_6_8_number(36,2,mid_angle);
     
     write_6_8_string(6,3,"top_k");
     write_6_8_number(36,3,top_k);
  }    
}

//***************************************************
//修改变量数值
//***************************************************
void change_value(uint8 page,uint8 m,float i)
{
  switch (page)
  {
    case 0:
    switch(m)
    {
    case 2:mid_angle+=5*i;
           LCD_clear_L(36,2);
          write_6_8_char(0,2,'*');
         write_6_8_number(36,2,mid_angle);

          break;
    case 3:top_k+=i;
          LCD_clear_L(36,3);
         write_6_8_char(0,3,'*');
         write_6_8_number(36,3,top_k);
          
          break;  
    }
  }
}  
//***************************************************
//按键扫描
//***************************************************
void scan_key(void)
{
  //切换页面
  if(!Change_page)  //如果检测到低电平，说明按键按下
  {
    KEY_DLY_ms(30); //延时去抖，一般10-20ms
    if(!Change_page)     //再次确认按键是否按下，没有按下则退出
   {
     while(!Change_page);//如果确认按下按键等待按键释放，没有释放则一直等待
     {
       if(page_num<2)    //页序号加操作
         page_num++;
       else
         page_num=0;
     }
     pre_show();
   }
  }
  if(page_num==0)     //如不为第一页，则进行下一步扫描
  {
  //切行
    if(!Add_line)  //如果检测到低电平，说明按键按下
    {
      KEY_DLY_ms(30); //延时去抖，一般10-20ms
      if(!Add_line)     //再次确认按键是否按下，没有按下则退出
      {
        while(!Add_line);//如果确认按下按键等待按键释放，没有释放则一直等待       
        if(line_num<15)    //行序号加操作
           line_num++;
        else
          line_num=0;
        pre_show();
      } 
    }
    if(!Sub_line)  //如果检测到低电平，说明按键按下
    {
      KEY_DLY_ms(30); //延时去抖，一般10-20ms
      if(!Sub_line)     //再次确认按键是否按下，没有按下则退出
      {
        while(!Sub_line);//如果确认按下按键等待按键释放，没有释放则一直等待

        if(line_num>0)    //行序号减操作
           line_num--;
        else
          line_num=15            ;
        pre_show();
      } 
    }
/*对应参数加*/
    if(!Add)  //如果检测到低电平，说明按键按下
    {
       KEY_DLY_ms(30); //延时去抖，一般10-20ms
       if(!Add)     //再次确认按键是否按下，没有按下则退出
       {
         while(!Add);//如果确认按下按键等待按键释放，没有释放则一直等待
         change_value(page_num,line_num,1);
         pre_show();
       }
    }
/*对应参数加*/
    if(!Sub)  
    {
      KEY_DLY_ms(30); 
      if(!Sub)
      {
        while(!Sub);
        change_value(page_num,line_num,-1); 
        pre_show();
      }
    }
  }
  
}


//***************
//get_position
//***************
void get_position(void)
{
LDC_val1=ldc_read_avr1()/10;   //采样滤波
LDC_val2=ldc_read_avr2()/10;
write_6_8_number(10,0,(uint16)LDC_val1);
write_6_8_number(10,1,(uint16)LDC_val2);

LDC_val=LDC_val1-LDC_val2;
}

//***************
//deal_position
//***************
void deal_position(void)
{
//if((LDC_val1>1340)&&(LDC_val1<1360))    {error=8;worse=2;}
 // if((LDC_val1>1340)&&(LDC_val1<1360))    {error=7;worse=2;}
  if((LDC_val>250)&&(LDC_val<300))    {error=6;worse=2;}
  if((LDC_val>200)&&(LDC_val<250))    {error=5;worse=2;}
  if((LDC_val>150)&&(LDC_val<200))    {error=4;worse=2;}
  if((LDC_val>100)&&(LDC_val<150))    {error=3;worse=2;}
  if((LDC_val>50)&&(LDC_val<100))    {error=2;worse=2;}
  if((LDC_val>30)&&(LDC_val<50))    {error=1;worse=2;}

  if((LDC_val>0)&&(LDC_val<30))    {error=0;worse=2;}
  
  if((LDC_val>-30)&&(LDC_val<0))   {error=-1;worse=2;}
  if((LDC_val>-50)&&(LDC_val<-30))    {error=-2;worse=2;}
  if((LDC_val>-100)&&(LDC_val<-50))    {error=-3;worse=2;}
  if((LDC_val>-150)&&(LDC_val<-100))    {error=-4;worse=2;}
  if((LDC_val>-200)&&(LDC_val<-250))    {error=-5;worse=2;}
  if((LDC_val>-300)&&(LDC_val<-250))    {error=-6;worse=2;}
 // if((LDC_val>1650)&&(LDC_val<1700))    {error=-7;worse=2;}
  //if((LDC_val>1700)&&(LDC_val<1750))    {error=-8;worse=2;}
  
  //if((LDC_val1>1800)&&(LDC_val1<1900)&&(LDC_val2>1700)&&(LDC_val2<1750))   {error=error_history[ERROR_HISTORY_NUM-1];worse++;}
}

//***************
//servo_control
//***************
int steer_value;
void servo_control(void)
{
char i=0;

    if(ABS(error-error_history[4])>=16) //如果两次的偏差过大，则保持上一次的偏差
        error=error_history[4];      //防止不确定情况（如红外管冲出跑道）造成的偏差跳变
    
    //根据偏差计算舵机角度输出
     steer_value=(servo_P*error    //比例
                   + servo_D*(error-error_history[3])//微分
               )/3;   
   servo_PWM= servo_middle+steer_value;
    if(servo_PWM<540) //限幅
    servo_PWM=540; //防止舵机超过极限值
    if(servo_PWM>670)
    servo_PWM=670;
    
FTM_PWM_Duty(FTM0,FTM_CH0,servo_PWM);//PTC1   

  for(i=0;i<ERROR_HISTORY_NUM-1;i++)    //记录偏差
    {
        error_history[i]=error_history[i+1];
    }
    error_history[ERROR_HISTORY_NUM-1]=error;

  
}


//用来判断一个整型数据的位数的。
int panduanweishu(int a)
{int i;
    do
    {
      i=i+1;
    }while((a/10^i)==0);
  
      return i;
}

  
  


void servo_control1(void)

{
  LDC_val1=LDC_val1-300;
  LDC_val2=LDC_val2-300;
  if((LDC_val1>1450)&&(LDC_val2<1400))
  {
    
    
    m=LDC_val1;//右边的
    n=1;
  }
    else
    {
    m=LDC_val2;
    
    n=-1;
  }
  
  //主要问题就是数据会溢出的，对吧。所以得想办法去取出最高位的数，我认为是可以实现的。计算机有什么不可能实现的呢对吧。
  //首先化为整数，再就是判断位数，再除以相应的10的多少次方就OK了
  m=m/100;
  //jiaodu=(2152/1000)*(((-1736/1000)*10^(-7))*m*m*m+(87/100000)*m*m-(1447/1000)*m+815);
  //jiaodu=-(1736/1000)*10^(-7)*m;
  //jiaodu=jiaodu/100000;
  //jiaodu=2.152*(-0.0001736*m*m*m+0.0866*m*m-14.47*m+814.9); 
  
  //那就一步步来
  f=-0.17*m*m*m+8.67*m*m-144*m+815;
  ;
  //jiaodu=-1.06*m*m*m-0.72*m*m-1.468*m+4.435;
 
 
  //jiaodu=2.152*(-0.173*m*m*m+8.66*m*m-144.7*m+814.9);

  // f=panduanweishu(jiaodu);
  // if(f>2)
  // jiaodu=jiaodu/10^(f-2);
 // jiaodu=2.152*(-0.000173*m*m*m+0.0866*m*m-14.47*m+814.9);
 // servo_PWM=605+n*1.25*jiaodu;
  
  
   write_6_8_number(10,4,f);
   write_6_8_number(10,5,m);
   write_6_8_number(10,6,jiaodu);
   write_6_8_number(10,7,servo_PWM);
  
}

//***************
//servo_control
//***************

void motor_control(void)
{
real_speed=speed_expect;
FTM_PWM_Duty(FTM2,FTM_CH0,real_speed);//PTB18
FTM_PWM_Duty(FTM2,FTM_CH1,0);//PTB19    
}

/**********************************************
//  @brief      main函数
//  @since      v1.0
//  @note       LDC1000模块测试程序
*********************************************/ 
 void main(void)
{
  
  char maxi='c';
   // DisableInterrupts;      //禁止总中断 
    GIPO_init();
    LCD_Init();//液晶显示初始化
    FLOAT_LDC_init1();//电轨传感器初始化  通道1
    FLOAT_LDC_init2();//通道2
    
    
    //uart_init(UART3,9600); 
    uart_init(UART3,9600); 
    // pit_init_ms(PIT0,3);
    // enable_pit_interrupt(PIT0);                      //开pit中断 
   
//    while(1)
//             
//                {
//             LDC_val1=filter1()/10;   //采样滤波
//             LDC_val2=filter2()/10;
//             write_6_8_number(10,0,(uint16)LDC_val1);
//             write_6_8_number(10,1,(uint16)LDC_val2);
//                } 
    // while(Go)      
    //{ 
      //  go=0;
      //  scan_key();
        pre_show();
        LCD_DLY_ms(2000);
        
        //uint8 maxi;
        
       // uart_init(UART1,9600);
        /**
        
	if(!Go)  //如果检测到低电平，说明按键按下
          {
  	    KEY_DLY_ms(10);   //延时去抖，一般10-20ms
            if(!Go)     //再次确认按键是否按下，没有按下则退出
	      {
              while(!Go);//如果确认按下按键等待按键释放，没有释放则一直等待
              go=1;
	      }
	   }
             if(go==1)
             {
            //EnableInterrupts;   //开总中断
           **/     
               while(1)
             
                {
                get_position();
                deal_position();
               // servo_control();
                servo_control1();
                 motor_control();
                 
            //  write_6_8_number(10,5,LDC_val1);
                 /**
                 uart_putchar(UART1,0x03); 
               //  write_6_8_number(10,5,1234);
                 uart_putchar(UART1,0xfc);
                 //maxi=uart_getchar(UART3);
                // uart_putchar(UART3,'1');
                 uart_putchar(UART1,(uint16)LDC_val1);
                 uart_putchar(UART1,(uint16)LDC_val2);
                 uart_putchar(UART1,0xfc); 
                 uart_putchar(UART1,0x03);
             **/
                }
       
         //     }
    // }

         
}


void FLOAT_delay(unsigned int ms)//为防止time_delay_ms();与lpt冲突编写的延时
{
  unsigned int j1,k_1;int i1;
  i1=ms;
  for(j1=0;j1<i1;j1++)   
    for(k_1=0;k_1<14120;k_1++);
}

