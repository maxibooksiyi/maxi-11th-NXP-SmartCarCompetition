#include "include.h"
#include <math.h>
//�ٸ�

//����
#define Go              (bool)(GPIOA_PDIR >> 0 & 0x00000001)
#define Change_page     (bool)(GPIOA_PDIR >> 2 & 0x00000001)
#define Add_line        (bool)(GPIOA_PDIR >> 1 & 0x00000001)
#define Sub_line        (bool)(GPIOE_PDIR >> 28 & 0x00000001)
#define Add             (bool)(GPIOA_PDIR >> 8 & 0x00000001)
#define Sub             (bool)(GPIOA_PDIR >> 9 & 0x00000001)

uint8 page_num=0;     //ҳ���
uint8 line_num=0;     //�����
bool go;//������־


double jiaodu;//�Ҷ���һ���Ƕ�
double m;
int n;
double f;
int servo_PWM=0;//���PWM //����char�ͱ����ķ�Χ


char speed_expect=30; //�����ٶ�
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
//oled��ʾ���õĺ���
//***************************************************
extern void LCD_Init(void);
extern void LCD_clear_L(uint8 x,uint8 y);
extern void LCD_DLY_ms(uint16 ms);//��Ļ��ʱȥ������
extern void LCD_CLS(void);
extern void write_6_8_char(uint8 x,uint8 y,uint8 ucData) ;
extern void write_6_8_string(uint8 x,uint8 y,uint8 ch[]);
extern void write_6_8_number(uint8 x,uint8 y, float number);

void Get_data(void);
void Data_nanlyze(void);
void Duoji_PD(void);
void FLOAT_delay(unsigned int ms)//Ϊ��ֹtime_delay_ms();��lpt��ͻ��д����ʱ
{
  unsigned int j1,k_1;int i1;
  i1=ms;
  for(j1=0;j1<i1;j1++)   
    for(k_1=0;k_1<14120;k_1++);
}
//*****************
//������ʱȥ������
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
//�������������� ��ʼ��
//******************
void GIPO_init(void)
{
         gpio_init (PTA0, GPI_UP,1);
         gpio_init (PTA1,GPI_UP,1);
         gpio_init (PTA2, GPI_UP,1); 
         gpio_init (PTA8, GPI_UP,1);
         gpio_init (PTA9, GPI_UP,1); 
         gpio_init (PTE28, GPI_UP,1);
         
         FTM_PWM_init(FTM0,FTM_CH0,50,Duoji_middle);//��� PTC1
         FTM_PWM_init(FTM2,FTM_CH0,10000,0);//PTB18
         FTM_PWM_init(FTM2,FTM_CH1,10000,0);//PTB19

         
}
//***************************************************
//Ԥ��ʾ
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
//�޸ı�����ֵ
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
//����ɨ��
//***************************************************
void scan_key(void)
{
  //�л�ҳ��
  if(!Change_page)  //�����⵽�͵�ƽ��˵����������
  {
    KEY_DLY_ms(30); //��ʱȥ����һ��10-20ms
    if(!Change_page)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
   {
     while(!Change_page);//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
     {
       if(page_num<2)    //ҳ��żӲ���
         page_num++;
       else
         page_num=0;
     }
     pre_show();
   }
  }
  if(page_num==1)     //�粻Ϊ��һҳ���������һ��ɨ��
  {
  //����
    if(!Add_line)  //�����⵽�͵�ƽ��˵����������
    {
      KEY_DLY_ms(30); //��ʱȥ����һ��10-20ms
      if(!Add_line)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
      {
        while(!Add_line);//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�       
        if(line_num<15)    //����żӲ���
           line_num++;
        else
          line_num=0;
        pre_show();
      } 
    }
    if(!Sub_line)  //�����⵽�͵�ƽ��˵����������
    {
      KEY_DLY_ms(30); //��ʱȥ����һ��10-20ms
      if(!Sub_line)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
      {
        while(!Sub_line);//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�

        if(line_num>0)    //����ż�����
           line_num--;
        else
          line_num=15            ;
        pre_show();
      } 
    }
/*��Ӧ������*/
    if(!Add)  //�����⵽�͵�ƽ��˵����������
    {
       KEY_DLY_ms(30); //��ʱȥ����һ��10-20ms
       if(!Add)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
       {
         while(!Add);//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
         change_value(page_num,line_num,1);
         pre_show();
       }
    }
/*��Ӧ������*/
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
//��������·���ݴ�������
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
        adjustment[7]=ldc_read_avr1()/10;//�� val1
        adjustment[8]=ldc_read_avr2()/10;//�� val2
        ad[1]=ldc_read_avr3()/10;        //val3�м����ֵ
       } 
        write_6_8_number(55,0,(uint16)adjustment[8]);
        write_6_8_number(90,0,(uint16)adjustment[7]);
        write_6_8_number(55,4,(uint16)ad[1]);
       
        //���ƫ��
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
            adjustment[2]=ldc_read_avr1()/10;//��max
           }
        
          i1=100;
          while(i1--)
           {
            adjustment[1]=ldc_read_avr2()/10;//��min
           }
           write_6_8_number(55,2,(uint16)adjustment[1]);
           write_6_8_number(90,2,(uint16)adjustment[2]);
          
           
           
           
           write_6_8_string(0,3,"Lmax Rmin");
           FLOAT_delay(3000);
           i1=100;
           while(i1--)
            {
             adjustment[3]=ldc_read_avr1()/10;//��min
            }
           
           
           
           i1=100;
           while(i1--)
           {
            adjustment[4]=ldc_read_avr2()/10;//��max
           }
           write_6_8_number(55,3,(uint16)adjustment[4]);
           write_6_8_number(90,3,(uint16)adjustment[3]);
          
           
           write_6_8_string(0,5,"Lm Rm:");
           FLOAT_delay(3000);
           i1=100;
          while(i1--)
           {
            ad[0]=ldc_read_avr3()/10;//val3�����Сֵ
           }
           write_6_8_number(55,5,(uint16)ad[0]);
          FLOAT_delay(3000);
          i1=100;
           while(i1--)
           {
            ad[2]=ldc_read_avr3()/10; //val3�ұ���Сֵ
           }
           write_6_8_number(90,5,(uint16)ad[2]);
          
          
           
           //ȡ���������ֵ �еĽ�С��     �ϴ���
            if(adjustment[1]<=adjustment[3])
              adjustment[5]=adjustment[1];//��Сֵ
            else
              adjustment[5]=adjustment[3];//��Сֵ
     
            if(adjustment[2]>=adjustment[4])
              adjustment[6]=adjustment[2];//�ϴ�ֵ
            else
              adjustment[6]=adjustment[4];//�ϴ�ֵ
   
}
 


                           //adjustment[7]:�ұ�       adjustment[8]:���
void Data_analyze(void)    //val1:�ұ�                 val2:���
{ 
 if(flag1)//�ұ߱���ߴ�    
  {
    LDC_val2=LDC_val2+adjustment[0];//�� 
    Duoji_error=LDC_val2-LDC_val1;
    if(LDC_val3-ad[0]<20)//val1����  ��ת
    {
     //gpio_set (PTA17, 0);
     LDC_val11=adjustment[6];
     LDC_val22=adjustment[5];
     Duoji_error=(LDC_val22-LDC_val11)+50;
    }
     
    if( LDC_val3-ad[2]<20)//val2���� ��ת
    {
     // gpio_set (PTA17, 0);
     LDC_val22=adjustment[6];
     LDC_val11=adjustment[5];
     Duoji_error=(LDC_val22-LDC_val11)-50;
    }
  }
 
 
    if(flag2)//�ұ߱���ߴ�
    {
     LDC_val1=LDC_val1+adjustment[0];//��
     Duoji_error=LDC_val2-LDC_val1;
     if(LDC_val3-ad[0]<20)//val1����  ��ת
      {
       //gpio_set (PTA17, 0);
       LDC_val11=adjustment[6];
       LDC_val22=adjustment[5];
       Duoji_error=(LDC_val22-LDC_val11)+50;
       } 
      if( LDC_val3-ad[2]<20)//val2���� ��ת
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
  
  //�޷�
  if(LDC_val1<adjustment[5])         LDC_val1=adjustment[5];//��
   else if(LDC_val1>adjustment[6])   LDC_val1=adjustment[6];
   
  if(LDC_val2<adjustment[5])       LDC_val2=adjustment[5];//��
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
//  @brief      main����
//  @since      v1.0
//  @note       LDC1000ģ����Գ���
*********************************************/ 
/**
 void main(void)
{
    DisableInterrupts;      //��ֹ���ж� 
    GIPO_init();
    LCD_Init();//Һ����ʾ��ʼ��
    FLOAT_LDC_init1();//��촫������ʼ��  ͨ��1
    FLOAT_LDC_init2();//ͨ��2
    FLOAT_LDC_init3();//ͨ��3
    uart_init(UART1,9600);//���ڳ�ʼ��
    Get_data();
    // pit_init_ms(PIT0,3);
    // enable_pit_interrupt(PIT0);                      //��pit�ж� 
   
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
       
	if(!Go)  //�����⵽�͵�ƽ��˵����������
          {
  	    KEY_DLY_ms(10);   //��ʱȥ����һ��10-20ms
            if(!Go)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
	      {
              while(!Go);//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
              go=1;
	      }
	   }
             if(go==1)
             {
            //EnableInterrupts;   //�����ж�
                
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


//�����ж�һ���������ݵ�λ���ġ�
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
    
    
    m=LDC_val1;//�ұߵ�
    n=-1;
  }
    else
    {
    m=LDC_val2;
    
    n=1;
  }
  
  //��Ҫ����������ݻ�����ģ��԰ɡ����Ե���취ȥȡ�����λ����������Ϊ�ǿ���ʵ�ֵġ��������ʲô������ʵ�ֵ��ض԰ɡ�
  //���Ȼ�Ϊ�������پ����ж�λ�����ٳ�����Ӧ��10�Ķ��ٴη���OK��
  m=m/100;
  //jiaodu=(2152/1000)*(((-1736/1000)*10^(-7))*m*m*m+(87/100000)*m*m-(1447/1000)*m+815);
  //jiaodu=-(1736/1000)*10^(-7)*m;
  //jiaodu=jiaodu/100000;
  //jiaodu=2.152*(-0.0001736*m*m*m+0.0866*m*m-14.47*m+814.9); 
  
  //�Ǿ�һ������
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
   // DisableInterrupts;      //��ֹ���ж� 
    GIPO_init();
    LCD_Init();//Һ����ʾ��ʼ��
    FLOAT_LDC_init1();//��촫������ʼ��  ͨ��1
    FLOAT_LDC_init2();//ͨ��2
    FLOAT_LDC_init3();
    
    //uart_init(UART3,9600); 
    uart_init(UART3,9600); 
    // pit_init_ms(PIT0,3);
    // enable_pit_interrupt(PIT0);                      //��pit�ж� 
   
//    while(1)
//             
//                {
//             LDC_val1=filter1()/10;   //�����˲�
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
        
	if(!Go)  //�����⵽�͵�ƽ��˵����������
          {
  	    KEY_DLY_ms(10);   //��ʱȥ����һ��10-20ms
            if(!Go)     //�ٴ�ȷ�ϰ����Ƿ��£�û�а������˳�
	      {
              while(!Go);//���ȷ�ϰ��°����ȴ������ͷţ�û���ͷ���һֱ�ȴ�
              go=1;
	      }
	   }
             if(go==1)
             {
            //EnableInterrupts;   //�����ж�
           **/     
               while(1)
             
                {LDC_val1=ldc_read_avr1()/10;   //�����˲�
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