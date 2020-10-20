
/******************************** WUST Smart Car Team 0605 *************************************
                 some advice need to pay attention to when programming��
1ѡ����ʵ��㷨�����ݽṹ
2�������ͣ�ʹ�þ���С����������,����byte��char�Ĳ���int
3���������ǿ�ȣ�ʹ��������С��������ͬ�ı��ʽ�滻ԭ�����ӵĵı��ʽ,����λ�����ļ�����ø��ӵ���������,
          ��������ʵ�ֵĲ��ø�������ʵ��,������λʵ�ֳ˳�������,ʹ���Լӡ��Լ�ָ��

4ʹ��ָ���ʹ���������ɵĴ�����̣�ִ��Ч�ʸ��� (��һ��)
5�����ò����渴�ӵ���ѧ����

������������ȫ��,СS�ӽ�ֱ��,��������ʱ��̫����
�����ٶȿ�ʱ���ڻ�еԭ�����׷���    1/(2*pi*sqrt(0.0001*4.7*0.000001)) 1/(2*pi*sqrt(0.0001*4.7*0.000001))

�����ٶȻ�����


***********************************************************************************************/

 
 
#define N_CHANNEL 4    //����ͨ����
#define N_SIGNAL  80   //ÿ������AD�������źŸ���
#define N_CUN     50   //������ʷ��Ϣ����


#define Angle_mid 5678   
        
#define Angle_mid1 4740          //���������<360�������Ҵ�>-360     850�м�
#define Angle_min -900      //    �����ұ�
#define Angle_max 900

int cnt=0;
int startflag=0;
int startcheckcount=0;
int finishcount=0;

/////////////////////sensor///////////////////
byte  Pre_AD_Data[N_CHANNEL][N_SIGNAL];   //AD����ֵ�˲�֮ǰ�洢������ byte  Min_Pre_AD_Data[N_CHANNEL];  //��Сֵ���ڼ���Ϊ0����ÿ��ͨ��ֻȡһ������
byte  AD_Data[N_CHANNEL];                 //AD���˺���Ч��ֵ
//byte  PreAD_Data[N_CHANNEL];              //ǰһ��AD���˺���Ч��ֵ
byte  Max_six[5];
//byte  Min_six[7];
//byte  DC_AD_Data[N_CHANNEL]={127,128,125,126,126,125,127,127};
byte  DC_AD_Data[N_CHANNEL]={128,128,128,128};   //�������
byte  MaxAD_Data[N_CHANNEL],MinAD_Data[N_CHANNEL];  //���ڱ궨���ֵ����Сֵ��
byte  Ave_Max,Ave_Min;          //���ֵ��ˮƽ���ֵ��ƽ��ֵ  ������ֱ��percent
byte  percent[N_CHANNEL];       //��ǰÿ��������ֵ��С�İٷֱ�
//byte  dif_percent[N_CHANNEL];    //ǰһ��ÿ��������ֵ��С�İٷֱȵ�΢��   
//byte  Percent[N_CUN][N_CHANNEL];  //���������������ʷ�ٷֱ� 
int   distance=0;                //��ǰλ�ó��ӵ�ƫ��
int   predistance=0; 
byte  state=1;
int   dist=0;
int   slopedowncnt=0;
int   Dist[N_CUN]={0};
int   dist1=0;
int   Dist1[N_CUN]={0};
int   curve_value=0;
struct time_flag 
{
  unsigned long int time;//����
  unsigned char flag;    //��־
};
struct time_flag  mid,left,right,curve,slopeup,slopedown,cross,fail,ruwan;


/////////////////////servo////////////////////
int Angle=0;  //�������������Ҫ��ĽǶ�
int preAngle=0;
///////////////////
byte area=1;
int rad=0;
int Rad[N_CUN]={0};
int dif_rad=0;
int predif_rad=0;
int s_Kp=0;
int s_Kd=0;


//////////////////////motor/////////////////////
int MaxExpspeed=180;
int MinExpspeed=120;
int SpeedMin=0;
int SpeedMax=600;
int Realspeed=0;
int Expspeed=130;
int SpeedPID=0;
typedef struct PID{
  int Kp;
  int Ki;
  int Kd;
  int error;     //��ǰ�ٶ�ƫ��
  int preError;  //ǰһ�ε��ٶ�ƫ��
  int derror;    //��ǰ��ƫ��仯��
  int preDerror;   //��һ�ε�ƫ��仯��
}PID;
PID  Vpid;
int Speed_Straight_High,Speed_Straight_Low,Speed_BigCircle_High,Speed_BigCircle_Low;
int Speed_Curve90_High,Speed_Curve90_Low,Speed_Curve180_High,Speed_Curve180_Low,Speed_WaveCurve_High;
int Speed_WaveCurve_Low,Speed_NormalCurve_High,Speed_NormalCurve_Low,Speed_fail;





/////////////////////////////////////////////////////////////////////////////////
//��ÿ�������������ֵ����Ϊ85
//����λ�����м�ʱ�ķ�ֵ���ε���: 38 60 60 37  38 56 56 38
//�ٷֱ�:50 74 74 50  50 62 62 50 

void ResetTime_flag(void) 
{
   mid.time=0;
   curve.time=0;
   fail.time=0;
   left.time=0;
   right.time=0;
         
   mid.flag=1;       
   curve.flag=0;
   fail.flag=0;
}
void GetDistance3(void)      
{
  //����λ������ʱ 70 100 100 70
  //���м���������֮�� �߽紦100 32
  
  //38 115   124 113 53 32    29 47 122 127    35 55 122 123
   //0~ 46  -45~0
   //���µ���� ��̬��ʱ �¶���Ϊ 47  52  50  53   41  45  45  46
  if(abs(percent[2]-percent[1])<=8&&(percent[1]+percent[2])<150&&(percent[1]+percent[2])>=100&&abs(percent[3]-percent[0])<=30)
  {
    distance=0;area=1;
    
  }
  else if(abs(percent[2]-percent[1])<=20&&(percent[2]+percent[1])<150&&(percent[2]+percent[1])>=100&&abs(percent[3]-percent[0])<=30)
  {
    distance=140*(percent[2]-percent[1])/(percent[2]+percent[1]);area=2;
    
  }
  else if(abs(percent[2]-percent[1])<=6)
  {
      if(percent[2]+percent[1]>150)
      {
        distance=0;area=3;
        
      } 
       else if((percent[1]+percent[2])<=80 && Rad[1]<0) 
      {
        distance=-120;area=4;  //�߽�ֵ51,6
        
      }
      else if((percent[1]+percent[2])<=80 && Rad[1]>0) 
      {
        distance=120; area=5;
        
      } 
      else 
      {
        distance=predistance;
      }
  }
  //���µ����1 ��̬����ʱ ������
  
  //���м���������֮�� �߽紦100 32
  //0~45  ֱ�� Сs  �� 
  else if(abs(percent[2]-percent[1])>6) 
  {
    
      if(percent[2]>percent[1]&&percent[1]>85&&percent[1]<=97)   //��������ƫ ��percent[2]���Ƕ�
      {
          if(percent[1]>=97) distance=0;
          else
            distance=(97*8/(97-85))-(percent[1]*8/(97-85));
          area=6;
      }
      else if(percent[1]>percent[2]&&percent[2]>85&&percent[2]<=97)   //��������ƫ ��percent[1]���Ƕ�
      {
          if(percent[2]>=97) distance=0;
          else
             distance=(97*-8/(97-85))-(percent[2]*-8/(97-85));
          area=7;
      }
      else if(percent[2]>percent[1]&&percent[1]<=85&& percent[1]>53)    //��������ƫ ��percent[2]���Ƕ�
      {
          distance=8-(85*-32/(85-53))+(percent[1]*-32/(85-53));area=8;
          
      }
      else if(percent[1]>percent[2]&&percent[2]<=85&& percent[2]>53)   //��������ƫ ��percent[1]���Ƕ�
      {
          distance=-8-(85*32/(85-53))+(percent[2]*32/(85-53));
          area=9;
      } 
       else if(percent[2]>percent[1]&&percent[1]<=53&& percent[1]>41)    //��������ƫ ��percent[2]���Ƕ�
      {
          distance=40-(53*-24/(53-41))+(percent[1]*-24/(53-41));
          area=10;
      }
      else if(percent[1]>percent[2]&&percent[2]<=53&& percent[2]>41)   //��������ƫ ��percent[1]���Ƕ�
      {
          distance=-40-(53*24/(53-41))+(percent[2]*24/(53-41));
          area=11;
      } 
      //90~140   ��ת�� 
      else if(percent[2]>percent[1]&& percent[1]<=41&&percent[3]>percent[0])  //��������ƫ ��percent[1]���Ƕ�
      {
          distance=-(-187+(percent[1]*96/32));
          area=12;
      }
      else if(percent[1]>percent[2]&& percent[2]<=41&&percent[0]>percent[3]) //��������ƫ ��percent[2]���Ƕ�
      {
          distance=-(187-(percent[2]*96/32));
          area=13;
      } 
      //45~90   ��S
      else 
      {
           distance=predistance;
           area=14;
      }
  }
  else 
  {
       distance=predistance;
       area=15;
  }
  if((distance-predistance)>=15) 
  {
    distance=predistance+15;
    area=16;
  } 
  else if((distance-predistance)<=-15) 
  {
    distance=predistance-15;
    area=17;
  } 
   //Խ������
  if((percent[3]+percent[0])>=40&&(percent[1]+percent[2])<=40)
  {
      distance=predistance;
      area=18;
  } 
  if(percent[3]>percent[0]&&(percent[1]+percent[2]<=40)&&percent[1]>percent[2])
  {
      distance=120;
      area=19;
  } 
  if(percent[0]>percent[3]&&(percent[1]+percent[2]<=40)&&percent[2]>percent[1])
  {
      distance=-120;
      area=20;
  } 
}

void GetInformation() 
{
  Dist[0]=percent[3]-percent[0];
  dist+=Dist[0]-Dist[20];//ȡ20����ʷ����
  dist/=20;
  
  Dist1[0]=percent[2]-percent[1];
  dist+=Dist1[0]-Dist1[20];//ȡ20����ʷ����
  dist1/=20;
  
  
  
  if(abs(distance)>=130) 
  {
    fail.time++;
    if(fail.time>=1) 
    {
       fail.flag=1; //fail.time=0
       mid.flag=0; mid.time=0;
       curve.flag=0;curve.time=0;
       left.time=0;right.time=0;
       ruwan.flag=0;
    } 
    else
       fail.flag=0;
    //state=3;           //����ƫ���ܵ������
  } 
  else if(abs(dif_rad)<=1&&abs(dist)<=10&&abs(dist1)<=10&&abs(percent[3]-percent[0])<=40&&abs(percent[2]-percent[1])<=25&&(percent[2]+percent[1])>=100&&abs(distance)<=20)
  {
       mid.time++;      
       if(mid.time>=10)   //�����Ŷ�������     (&����������ٶ��趨)
       {
         curve.time=0;
         fail.time=0;
         left.time=0;right.time=0;
       }     
       if(mid.time>=20)  //ֱ��  
       {
         mid.flag=1;       
         curve.flag=0;
         fail.flag=0;
         ruwan.flag=0;
       }
       else mid.flag=0;
  } 
  else                  //��ͨ���
  {
      if(curve.time==0) 
      {
        curve_value=0;
      } 
      else 
      {
        curve_value+=distance;
      }
      curve.time++; 
      if(distance>0)      right.time++;
      else if(distance<0) left.time++;
      
      if(curve.time>=3 && mid.time>=1000&&abs(percent[3]-percent[0])>30)   //��ֱ�������
      {
         ruwan.flag=1;
      } 
      if(curve.time>=10) 
      {
         mid.time=0;
         fail.time=0;
      } 
      if(curve.time>=20)   
      {
          curve.flag=1;
          mid.flag=0;
          fail.flag=0;
      }     
      else 
        curve.flag=0;
  } 
}

void State_Judge() 
{
  if(fail.flag) 
  {
    //����ƫ���ܵ������
      state=3;    PORTB=0b00001100;
  } 
  else if(mid.flag)
  {
      state=1;    PORTB=0b00001110;//ֱ�����������ж�
  } 
  else if(ruwan.flag&&curve.time<=80&&curve.time!=0) 
  {
      state=4;    PORTB=0b00001011; //��ֱ������
  }
 
  else if(curve.flag)
  {
      state=5;    PORTB=0b00001010; //������������ж�
  } 
}

void SlopedownDetect() 
{
  if((percent[1]+percent[2])<=150&&(percent[1]+percent[2])>=100&&abs(percent[2]-percent[1])<=20&&abs(percent[3]-percent[0])<=20) 
  {
       slopedown.time++;      
       if(slopedown.time>=8)   //�����Ŷ�������     (&����������ٶ��趨)
       {
         slopedown.flag=1;
       }     
       else slopedown.flag=0;
  }
}


void Sensor_process(void)   //������������
{
     GetADValue();
     SelData();
     GetDistance3(); //��ú����복�ӵ�ƫ����
     SlopedownDetect();
     
     if(slopedown.flag==1)  //����
     {
       slopedowncnt++;
       PORTB=0b01010101;
     } 
     else 
     {
       slopedowncnt=0;
       PORTB=0x00;
     }
     if(slopedowncnt>140) 
     {
       slopedown.flag=0;
       slopedowncnt=0;
     }
     //GetInformation();  //�����ж�����
     //State_Judge();
} 


void Servo_control()   
//��ÿ����������ֵ���ε��� 50 70 70 50 50 69 69 50
{
  int i;

   if(abs(distance-predistance)>=120) 
  {
    distance=predistance;
  } 
  
  if(abs(distance)<=20) 
  {
    s_Kp=6;
    rad=6*distance;//-120-+120
  } 
   else if(distance>20&&distance<=40) 
  {
    s_Kp=8;
    rad=8*distance-40;//+128-+280
  }
   else if(distance<-20&&distance>=-40) 
  {
   s_Kp=8;
    rad=8*distance+40;//-128--280
  }
  
   else if(distance>40&&distance<=70) 
  {  
    s_Kp=10;
    rad=10*distance-120;//+280-+580
  }
   else if(distance<-40&&distance>=-70) 
  {
    s_Kp=10;
    rad=10*distance+120;//-280--580
  } 
   else if(distance>70) 
  {  
    s_Kp=12;
    rad=12*distance-260;//580-
  }
   else if(distance<-70) 
  {
    s_Kp=12;
    rad=12*distance+260;//-580
  } 
  
  
  for(i=N_CUN-1;i>=1;i--)     //��ʷ���ݵĴ���
  {   
      Rad[i]=Rad[i-1];
  }
  Rad[0]=distance;      
  dif_rad=(Rad[0]+Rad[1]+Rad[2]-(Rad[4]+Rad[5]+Rad[3]))/3;
  
  if(abs(dif_rad-predif_rad)>=20) 
  {
    dif_rad=predif_rad;
  }
   
   if(abs(dif_rad)<=1) 
   {
     s_Kd=60;
     Angle=rad+s_Kd*dif_rad;//+60
   } 
   
   else if(dif_rad==2) 
   {
     s_Kd=90;
     Angle=rad+s_Kd*(dif_rad-1)+60;//+150
   } 
   else if(dif_rad==-2) 
   {
     s_Kd=90;
     Angle=rad+s_Kd*(dif_rad+1)-60;//-150
   } 
   else if(dif_rad>2&&dif_rad<=4)
   {
     s_Kd=100;
     Angle=rad+s_Kd*(dif_rad-2)+150;//+150-+350
   }
   else if(dif_rad<-2&&dif_rad>=-4)
   {
     s_Kd=100;
     Angle=rad+s_Kd*(dif_rad+2)-150;//-150--350
   }
   else if(dif_rad>4)
   {
     s_Kd=120;
     Angle=rad+s_Kd*(dif_rad-4)+350;//+350--
   }
   else if(dif_rad<-4)                    
   {
     s_Kd=120;
     Angle=rad+s_Kd*(dif_rad+4)-350;//-350--
   } 
  if(Angle<=Angle_min)  Angle=Angle_min;
  else if(Angle>=Angle_max)  Angle=Angle_max;
  
  if(abs(Angle-preAngle)>=1200) 
  {  
     Angle=preAngle;
  }
  
  if(Angle<Angle_min)  //��������
    PWMDTY01=Angle_min+Angle_mid;
  else if(Angle>Angle_max)
    PWMDTY01=Angle_max+Angle_mid;
  else
    PWMDTY01=Angle+Angle_mid; 
 
}

void Speed_select(void)        //�ٶ�ѡ��  Ĭ���ٶȼ�7����
{
}

void Speed_set() //ֱ���ٶ�Ҫ�ﵽ3m/s,ƽ���ﵽ2.5m/s
{
  //����϶������ߣ�ʹ�ٶ���λ��΢����dif�Ķ��κ��������ٶȻ��������뾶���
      if(slopedowncnt<=140&&slopedown.flag==1&&PTIH_PTIH5==0&&MaxExpspeed>=170) 
      {
        Expspeed=MinExpspeed; 
      } 
      else if(MaxExpspeed<=170) 
      {
        Expspeed=MaxExpspeed;
      } 
      else if(MaxExpspeed<=180) 
      {
          if(abs(dif_rad)<=1) 
          {
            Expspeed=MaxExpspeed; 
          }
          else if(abs(dif_rad)>=4) 
          {
            Expspeed=MaxExpspeed-10*abs(dif_rad)+10; 
          }
          else if(abs(dif_rad)>=3) 
          {
            Expspeed=MaxExpspeed-20; 
          }
          else Expspeed=MaxExpspeed-10;
      }
      else if(MaxExpspeed<=190) 
      {
          if(abs(dif_rad)==0) 
          {
            Expspeed=MaxExpspeed; 
          }
          
          else if(abs(dif_rad)>=3) 
          {
            Expspeed=MaxExpspeed-10*abs(dif_rad); 
          }
          else if(abs(dif_rad)>=2) 
          {
            Expspeed=MaxExpspeed-20; 
          }
          else Expspeed=MaxExpspeed-10;
      }
      else if(MaxExpspeed<=200) 
      {
        if(abs(dif_rad)<=0) 
          {
            Expspeed=MaxExpspeed; 
          }
          
          else if(abs(dif_rad)>=3) 
          {
            Expspeed=MaxExpspeed-12*abs(dif_rad); 
          }
          else if(abs(dif_rad)>=2) 
          {
            Expspeed=MaxExpspeed-20; 
          }
          else Expspeed=MaxExpspeed-10;
      } 
      else   //if(MaxExpspeed>200) 
      {
          if(abs(dif_rad)==0) 
          {
            Expspeed=MaxExpspeed; 
          }
          
          else if(abs(dif_rad)>=3) 
          {
            Expspeed=MaxExpspeed-15*abs(dif_rad); 
          }
          else if(abs(dif_rad)>=2) 
          {
            Expspeed=MaxExpspeed-30; 
          }
          else Expspeed=MaxExpspeed-20;
      }
  
      if(Expspeed<=MinExpspeed) Expspeed=MinExpspeed;
  
} 


void Speed_control()         //�������������
{
  Vpid.error=Expspeed-Realspeed;            //��õ�ǰ���ٶ�ƫ��
  Vpid.derror=Vpid.error-Vpid.preError;     //��õ�ǰ�ٶ�ƫ��ı仯�� 
  //���ÿ����ͻ��ַ�            
  if(Vpid.error>=200)   //��������
  {
    SpeedPID=SpeedMax;   //��������ת��ȫ��ռ�ձ�
  } 
  else if(Vpid.error<=-200) 
  {
    SpeedPID=SpeedMin;    //����������ͣ�����һ����תռ�ձ�
  } 
  else if(abs(Vpid.error)<=10) 
  {
    SpeedPID=Expspeed;
  }
  else //ȥ�����ֻ���   ���ַ���
  {
    SpeedPID+=12*Vpid.derror+2*Vpid.error;//���������ʽ�ɣ���������
    if(SpeedPID>=SpeedMax)       //��������
	      SpeedPID=SpeedMax;
  	else if(SpeedPID<=SpeedMin)
  	    SpeedPID=SpeedMin;
  }
  
  PWMDTY23=SpeedPID;
  Vpid.preError=Vpid.error;      //���Ƹ�ֵ
  Vpid.preDerror=Vpid.derror; 
}



void StopCar()     //ͣ������
{
}  
void main(void) 
{
  /* put your own code here */
  
    int i;
    Init_PLL();
    Init_PWM();
    Init_ATD();        //ADת��
    Init_ECT();         //��ʱ��
    Init_IRQ();
    Init_SCI();        //���ڳ�ʼ��
    //Init_fuzzyPID();
    ResetTime_flag();
    Demarcate();  //����ǰ�궨�����Сֵ 
    Dly_ms(3000);        //������ʱ3������    
 	EnableInterrupts;
	  
	TSCR1_TEN =1;  //start Timer 
	EnableInterrupts;
  for(;;) 
  {
    _FEED_COP(); /* feeds the dog */
    
    DDRM=0xff;//M�����
    PTM=0b00000010;
    //PTM=0b00000010;
    //DDRM=0b00000011;
    Speed_select();  //�ٶ�ѡ��   ����ʱ�����벻����  ֱ���ٶ� ����ٶ� ֱ��������ٶȶ�Ҫ���ö����
       
    for(;;) 
    {
      //ִ������3.8ms
      //PTM=~PTM;
      cnt++;
      if(cnt>=65535) cnt=0;
      
      Sensor_process();   //��������Ϣ����
      Servo_control();    //������� 
      
      Speed_set();        //�趨�ٶ�//
      Speed_control();    //�������    
      for(i=N_CUN-1;i>=1;i--)     //��ʷ���ݵĴ���
      {   
          Dist[i]=Dist[i-1];
          Dist1[i]=Dist1[i-1];
      }  
      
    	preAngle=Angle; 
    	predistance=distance;
    	predif_rad=dif_rad;  
    }
    
  } /* loop forever */
}







