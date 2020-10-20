#include "include.h"
#include <math.h>
//再改

//按键
#define Go              (bool)(GPIOA_PDIR >> 0 & 0x00000001)
#define Change_page     (bool)(GPIOA_PDIR >> 2 & 0x00000001)
#define Add_line        (bool)(GPIOA_PDIR >> 1 & 0x00000001)
#define Sub_line        (bool)(GPIOE_PDIR >> 28 & 0x00000001)
#define Add             (bool)(GPIOA_PDIR >> 8 & 0x00000001)
#define Sub             (bool)(GPIOA_PDIR >> 9 & 0x00000001)

uint8 page_num=0;     //页序号
uint8 line_num=0;     //行序号
bool go;//发车标志


double jiaodu;//我定义一个角度
double m;
int n;
double f;
int servo_PWM=0;//舵机PWM //超过char型变量的范围


char speed_expect=30; //给定速度
char real_speed=0;

extern int LDC_val1,LDC_val2,LDC_val1_pre,LDC_val2_pre,LDC_val3;
int LDC_val11,LDC_val22;
extern uint8 proximtyData[2];

int adjustment[10]={0,0,0,0,0,0,0,0,0,0};
int ad[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char flag1=0,flag2=0,flag3=0,flag4=0;
int16 error[20],val1[20],val2[20];

int Duoji_error=0,Duoji_error_pre=0;
int DP=30,DD=20;
int Duoji_middle=6000;
int Duoji_PWM=0;
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

void Get_data(void);
void Data_nanlyze(void);
void Duoji_PD(void);
void FLOAT_delay(unsigned int ms)//为防止time_delay_ms();与lpt冲突编写的延时
{
  unsigned int j1,k_1;int i1;
  i1=ms;
  for(j1=0;j1<i1;j1++)   
    for(k_1=0;k_1<14120;k_1++);
}
//*****************
//按键延时去抖函数
//*****************
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

//*****************
//按键、舵机、电机 初始化
//******************
void GIPO_init(void)
{
         gpio_init (PTA0, GPI_UP,1);
         gpio_init (PTA1,GPI_UP,1);
         gpio_init (PTA2, GPI_UP,1); 
         gpio_init (PTA8, GPI_UP,1);
         gpio_init (PTA9, GPI_UP,1); 
         gpio_init (PTE28, GPI_UP,1);
         
         FTM_PWM_init(FTM0,FTM_CH0,50,Duoji_middle);//舵机 PTC1
         FTM_PWM_init(FTM2,FTM_CH0,10000,0);//PTB18
         FTM_PWM_init(FTM2,FTM_CH1,10000,0);//PTB19

         
}
//***************************************************
//预显示
//***************************************************
/**
void pre_show(void)
{  
  LCD_CLS();
   switch(page_num)
  { case 0:
           write_6_8_string(0,0,"LG RG:");
           write_6_8_number(55,0,(uint16)adjustment[8]);
           write_6_8_number(90,0,(uint16)adjustment[7]);        
           
           write_6_8_string(0,1,"Get differ");
           write_6_8_number(55,1,(uint16)adjustment[0]);
           
           write_6_8_string(0,2,"Rmax Lmin");
           write_6_8_number(55,2,(uint16)adjustment[1]);
           write_6_8_number(90,2,(uint16)adjustment[2]);
           
           write_6_8_string(0,3,"Lmax Rmin");
           write_6_8_number(55,3,(uint16)adjustment[4]);
           write_6_8_number(90,3,(uint16)adjustment[3]);
           
           write_6_8_string(0,4,"Mid:"); 
           write_6_8_number(55,4,(uint16)ad[1]);
           
           
           write_6_8_string(0,5,"Lm Rm:");
           write_6_8_number(55,5,(uint16)ad[0]);
           write_6_8_number(90,5,(uint16)ad[2]);
           
           write_6_8_string(0,6,"D_Mid");
           write_6_8_number(55,6,Duoji_middle);
           
           break;
           
  
    case 1:
          if(line_num<=7)
           write_6_8_string(0,line_num,"*");
          else
           write_6_8_string(60,line_num-8,"*");
         
          write_6_8_string(10,0,"DP:");
          write_6_8_number(36,0,DP);
     
          write_6_8_string(10,1,"DD");
          write_6_8_number(36,1,DD); 
         
         break;
    case 2:
      
          break;
  }  
  
}
**/
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
     write_6_8_number(10,1,(uint16)LDC_val3);
     
    // write_6_8_string(6,2,"MID:");
    // write_6_8_number(36,2,mid_angle);
     
    // write_6_8_string(6,3,"top_k");
    // write_6_8_number(36,3,top_k);
  }    
}

//***************************************************
//修改变量数值
//***************************************************
void change_value(uint8 page,uint8 m,float i)
{
  switch (page)
  {
    case 1:
    switch(m)
    {
    case 2:DP+=1*i;
           LCD_clear_L(36,0);
          write_6_8_char(0,0,'*');
         write_6_8_number(36,0,DP);

          break;
    case 3:DD+=i;
          LCD_clear_L(36,1);
         write_6_8_char(0,1,'*');
         write_6_8_number(36,1,DD);
          
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
  if(page_num==1)     //如不为第一页，则进行下一步扫描
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
//*************************
//传感器两路数据处理、分析
//*************************

void Get_data(void)
{
      int i1=0;
      write_6_8_string(0,0,"LG RG:");
      write_6_8_string(0,4,"Mid:");
      FLOAT_delay(3000);
       
       i1=100;       
       while(i1--)
       {
        adjustment[7]=ldc_read_avr1()/10;//右 val1
        adjustment[8]=ldc_read_avr2()/10;//左 val2
        ad[1]=ldc_read_avr3()/10;        //val3中间最大值
       } 
        write_6_8_number(55,0,(uint16)adjustment[8]);
        write_6_8_number(90,0,(uint16)adjustment[7]);
        write_6_8_number(55,4,(uint16)ad[1]);
       
        //获得偏差
       write_6_8_string(0,1,"Get differ");
        if(adjustment[7]>=adjustment[8])
         { flag1=1;
           adjustment[0]=adjustment[7]-adjustment[8];
         }
        else
        {
          flag2=1;
          adjustment[0]=adjustment[8]-adjustment[7]; 
        }
         write_6_8_number(55,1,(uint16)adjustment[0]);
    
     
       
       
          write_6_8_string(0,2,"Rmax Lmin");
          FLOAT_delay(3000);
          i1=100;
          while(i1--)
           {
            adjustment[2]=ldc_read_avr1()/10;//右max
           }
        
          i1=100;
          while(i1--)
           {
            adjustment[1]=ldc_read_avr2()/10;//左min
           }
           write_6_8_number(55,2,(uint16)adjustment[1]);
           write_6_8_number(90,2,(uint16)adjustment[2]);
          
           
           
           
           write_6_8_string(0,3,"Lmax Rmin");
           FLOAT_delay(3000);
           i1=100;
           while(i1--)
            {
             adjustment[3]=ldc_read_avr1()/10;//右min
            }
           
           
           
           i1=100;
           while(i1--)
           {
            adjustment[4]=ldc_read_avr2()/10;//左max
           }
           write_6_8_number(55,3,(uint16)adjustment[4]);
           write_6_8_number(90,3,(uint16)adjustment[3]);
          
           
           write_6_8_string(0,5,"Lm Rm:");
           FLOAT_delay(3000);
           i1=100;
          while(i1--)
           {
            ad[0]=ldc_read_avr3()/10;//val3左边最小值
           }
           write_6_8_number(55,5,(uint16)ad[0]);
          FLOAT_delay(3000);
          i1=100;
           while(i1--)
           {
            ad[2]=ldc_read_avr3()/10; //val3右边最小值
           }
           write_6_8_number(90,5,(uint16)ad[2]);
          
          
           
           //取左右最外侧值 中的较小者     较大者
            if(adjustment[1]<=adjustment[3])
              adjustment[5]=adjustment[1];//较小值
            else
              adjustment[5]=adjustment[3];//较小值
     
            if(adjustment[2]>=adjustment[4])
              adjustment[6]=adjustment[2];//较大值
            else
              adjustment[6]=adjustment[4];//较大值
   
}
 


                           //adjustment[7]:右边       adjustment[8]:左边
void Data_analyze(void)    //val1:右边                 val2:左边
{ 
 if(flag1)//右边比左边大    
  {
    LDC_val2=LDC_val2+adjustment[0];//右 
    Duoji_error=LDC_val2-LDC_val1;
    if(LDC_val3-ad[0]<20)//val1丢线  右转
    {
     //gpio_set (PTA17, 0);
     LDC_val11=adjustment[6];
     LDC_val22=adjustment[5];
     Duoji_error=(LDC_val22-LDC_val11)+50;
    }
     
    if( LDC_val3-ad[2]<20)//val2丢线 左转
    {
     // gpio_set (PTA17, 0);
     LDC_val22=adjustment[6];
     LDC_val11=adjustment[5];
     Duoji_error=(LDC_val22-LDC_val11)-50;
    }
  }
 
 
    if(flag2)//右边比左边大
    {
     LDC_val1=LDC_val1+adjustment[0];//左
     Duoji_error=LDC_val2-LDC_val1;
     if(LDC_val3-ad[0]<20)//val1丢线  右转
      {
       //gpio_set (PTA17, 0);
       LDC_val11=adjustment[6];
       LDC_val22=adjustment[5];
       Duoji_error=(LDC_val22-LDC_val11)+50;
       } 
      if( LDC_val3-ad[2]<20)//val2丢线 左转
       {
       // gpio_set (PTA17, 0);
       LDC_val22=adjustment[6];
       LDC_val11=adjustment[5];
       Duoji_error=(LDC_val22-LDC_val11)-50;
       }
    }   
   LDC_val1_pre=LDC_val1;
   LDC_val2_pre=LDC_val2;
  write_6_8_number(85,7,(uint16)LDC_val1);
  write_6_8_number(3,7,(uint16)LDC_val2); 
  
  //限幅
  if(LDC_val1<adjustment[5])         LDC_val1=adjustment[5];//右
   else if(LDC_val1>adjustment[6])   LDC_val1=adjustment[6];
   
  if(LDC_val2<adjustment[5])       LDC_val2=adjustment[5];//左
   else if(LDC_val2>adjustment[6]) LDC_val2=adjustment[6];

 
}

void Duoji_PD(void)
{
 
Duoji_PWM = (DP * Duoji_error+DD * (Duoji_error - Duoji_error_pre))/20;   
Duoji_error_pre=Duoji_error; 
     
if(Duoji_PWM<-800)Duoji_PWM=-800;
else if(Duoji_PWM>800)Duoji_PWM=800;

FTM_PWM_Duty(FTM0,FTM_CH0,6000+Duoji_PWM);
write_6_8_number(90,3,6000+Duoji_PWM);


}


void motor_control(void)
{
real_speed=speed_expect;
FTM_PWM_Duty(FTM2,FTM_CH0,real_speed);//PTB18
FTM_PWM_Duty(FTM2,FTM_CH1,0);//PTB19    
}

/********************************************
//  @brief      main函数
//  @since      v1.0
//  @note       LDC1000模块测试程序
*********************************************/ 
/**
 void main(void)
{
    DisableInterrupts;      //禁止总中断 
    GIPO_init();
    LCD_Init();//液晶显示初始化
    FLOAT_LDC_init1();//电轨传感器初始化  通道1
    FLOAT_LDC_init2();//通道2
    FLOAT_LDC_init3();//通道3
    uart_init(UART1,9600);//串口初始化
    Get_data();
    // pit_init_ms(PIT0,3);
    // enable_pit_interrupt(PIT0);                      //开pit中断 
   
//    while(1)
//    {
//    LDC_val1=ldc_read_avr1()/10; 
//     LDC_val2=ldc_read_avr2()/10;
//         write_6_8_number(85,4,(uint16)LDC_val1);
//    write_6_8_number(3,4,(uint16)LDC_val2); 
//
//    }
////    
     while(Go)      
    { 
        go=0;
        scan_key();
        pre_show();
        LCD_DLY_ms(2000);
       
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
                
                  while(1)
                  {
                   LDC_val1=ldc_read_avr1()/10; 
                   LDC_val2=ldc_read_avr2()/10;
                   LDC_val3=ldc_read_avr3()/10;
                   Data_analyze();
                   Duoji_PD();
                   motor_control();
                  }
       
              }
     }

         
}


**/


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
  //if((LDC_val1>1400)&&(LDC_val2<1400))
  if(LDC_val1>LDC_val2)
  {
    
    
    m=LDC_val1;//右边的
    n=-1;
  }
    else
    {
    m=LDC_val2;
    
    n=1;
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
  jiaodu=2.152*(f-31);
  ;
  //jiaodu=-1.06*m*m*m-0.72*m*m-1.468*m+4.435;
 
 
  //jiaodu=2.152*(-0.173*m*m*m+8.66*m*m-144.7*m+814.9);

  // f=panduanweishu(jiaodu);
  // if(f>2)
  // jiaodu=jiaodu/10^(f-2);
 // jiaodu=2.152*(-0.000173*m*m*m+0.0866*m*m-14.47*m+814.9);
  // servo_PWM=605+n*1.25*jiaodu;
   servo_PWM=6000+n*16.7*jiaodu;
  
  
   write_6_8_number(10,4,f-31);
   write_6_8_number(10,5,m);
   write_6_8_number(10,6,jiaodu);
   write_6_8_number(10,7,servo_PWM);
   
   FTM_PWM_Duty(FTM0,FTM_CH0,servo_PWM);//PTC1   
  
}
void main(void)
{
  
 // char maxi='c';
   // DisableInterrupts;      //禁止总中断 
    GIPO_init();
    LCD_Init();//液晶显示初始化
    FLOAT_LDC_init1();//电轨传感器初始化  通道1
    FLOAT_LDC_init2();//通道2
    FLOAT_LDC_init3();
    
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
             
                {LDC_val1=ldc_read_avr1()/10;   //采样滤波
                  LDC_val2=ldc_read_avr2()/10;
                  LDC_val3=ldc_read_avr3()/10;
                  write_6_8_number(10,0,(uint16)LDC_val1);
write_6_8_number(10,1,(uint16)LDC_val2);
write_6_8_number(10,2,(uint16)LDC_val3);

              //  get_position();
              //  deal_position();
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