
/******************************** WUST Smart Car Team 0605 *************************************
                 some advice need to pay attention to when programming：
1选择合适的算法和数据结构
2数据类型：使用尽量小的数据类型,能用byte和char的不用int
3减少运算的强度：使用运算量小但功能相同的表达式替换原来复杂的的表达式,能用位操作的坚决不用复杂的算术运算,
          能用整形实现的不用浮点运算实现,尽量移位实现乘除法运算,使用自加、自减指令

4使用指针比使用数组生成的代码更短，执行效率更高 (不一定)
5尽量用查表代替复杂的数学运算

开环恒速跑完全程,小S接近直冲,仅过急弯时不太流畅
车子速度快时由于机械原因容易翻车    1/(2*pi*sqrt(0.0001*4.7*0.000001)) 1/(2*pi*sqrt(0.0001*4.7*0.000001))

整体速度还可以


***********************************************************************************************/

 
 
#define N_CHANNEL 4    //采样通道数
#define N_SIGNAL  80   //每个周期AD采样的信号个数
#define N_CUN     50   //保存历史信息次数


#define Angle_mid 5678   
        
#define Angle_mid1 4740          //增加则左打<360，减少右打>-360     850中间
#define Angle_min -900      //    打到最右边
#define Angle_max 900

int cnt=0;
int startflag=0;
int startcheckcount=0;
int finishcount=0;

/////////////////////sensor///////////////////
byte  Pre_AD_Data[N_CHANNEL][N_SIGNAL];   //AD采样值滤波之前存储的数组 byte  Min_Pre_AD_Data[N_CHANNEL];  //最小值由于几乎为0相差不大，每个通道只取一个足矣
byte  AD_Data[N_CHANNEL];                 //AD过滤后有效的值
//byte  PreAD_Data[N_CHANNEL];              //前一次AD过滤后有效的值
byte  Max_six[5];
//byte  Min_six[7];
//byte  DC_AD_Data[N_CHANNEL]={127,128,125,126,126,125,127,127};
byte  DC_AD_Data[N_CHANNEL]={128,128,128,128};   //电池满电
byte  MaxAD_Data[N_CHANNEL],MinAD_Data[N_CHANNEL];  //用于标定最大值和最小值的
byte  Ave_Max,Ave_Min;          //最大值的水平最大值的平均值  用于竖直求percent
byte  percent[N_CHANNEL];       //当前每个传感器值大小的百分比
//byte  dif_percent[N_CHANNEL];    //前一次每个传感器值大小的百分比的微分   
//byte  Percent[N_CUN][N_CHANNEL];  //保存各个传感器历史百分比 
int   distance=0;                //当前位置车子的偏差
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
  unsigned long int time;//计数
  unsigned char flag;    //标志
};
struct time_flag  mid,left,right,curve,slopeup,slopedown,cross,fail,ruwan;


/////////////////////servo////////////////////
int Angle=0;  //舵机向左向右需要打的角度
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
  int error;     //当前速度偏差
  int preError;  //前一次的速度偏差
  int derror;    //当前的偏差变化率
  int preDerror;   //上一次的偏差变化率
}PID;
PID  Vpid;
int Speed_Straight_High,Speed_Straight_Low,Speed_BigCircle_High,Speed_BigCircle_Low;
int Speed_Curve90_High,Speed_Curve90_Low,Speed_Curve180_High,Speed_Curve180_Low,Speed_WaveCurve_High;
int Speed_WaveCurve_Low,Speed_NormalCurve_High,Speed_NormalCurve_Low,Speed_fail;





/////////////////////////////////////////////////////////////////////////////////
//将每个传感器的最大值调至为85
//车子位于正中间时的峰值依次调至: 38 60 60 37  38 56 56 38
//百分比:50 74 74 50  50 62 62 50 

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
  //车子位于正中时 70 100 100 70
  //在中间两传感器之间 边界处100 32
  
  //38 115   124 113 53 32    29 47 122 127    35 55 122 123
   //0~ 46  -45~0
   //下坡的情况 姿态正时 坡顶处为 47  52  50  53   41  45  45  46
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
        distance=-120;area=4;  //边界值51,6
        
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
  //下坡的情况1 姿态不正时 车身不正
  
  //在中间两传感器之间 边界处100 32
  //0~45  直道 小s  大环 
  else if(abs(percent[2]-percent[1])>6) 
  {
    
      if(percent[2]>percent[1]&&percent[1]>85&&percent[1]<=97)   //车子向左偏 由percent[2]给角度
      {
          if(percent[1]>=97) distance=0;
          else
            distance=(97*8/(97-85))-(percent[1]*8/(97-85));
          area=6;
      }
      else if(percent[1]>percent[2]&&percent[2]>85&&percent[2]<=97)   //车子向右偏 由percent[1]给角度
      {
          if(percent[2]>=97) distance=0;
          else
             distance=(97*-8/(97-85))-(percent[2]*-8/(97-85));
          area=7;
      }
      else if(percent[2]>percent[1]&&percent[1]<=85&& percent[1]>53)    //车子向左偏 由percent[2]给角度
      {
          distance=8-(85*-32/(85-53))+(percent[1]*-32/(85-53));area=8;
          
      }
      else if(percent[1]>percent[2]&&percent[2]<=85&& percent[2]>53)   //车子向右偏 由percent[1]给角度
      {
          distance=-8-(85*32/(85-53))+(percent[2]*32/(85-53));
          area=9;
      } 
       else if(percent[2]>percent[1]&&percent[1]<=53&& percent[1]>41)    //车子向左偏 由percent[2]给角度
      {
          distance=40-(53*-24/(53-41))+(percent[1]*-24/(53-41));
          area=10;
      }
      else if(percent[1]>percent[2]&&percent[2]<=53&& percent[2]>41)   //车子向右偏 由percent[1]给角度
      {
          distance=-40-(53*24/(53-41))+(percent[2]*24/(53-41));
          area=11;
      } 
      //90~140   急转弯 
      else if(percent[2]>percent[1]&& percent[1]<=41&&percent[3]>percent[0])  //车子向左偏 由percent[1]给角度
      {
          distance=-(-187+(percent[1]*96/32));
          area=12;
      }
      else if(percent[1]>percent[2]&& percent[2]<=41&&percent[0]>percent[3]) //车子向右偏 由percent[2]给角度
      {
          distance=-(187-(percent[2]*96/32));
          area=13;
      } 
      //45~90   大S
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
   //越道处理
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
  dist+=Dist[0]-Dist[20];//取20次历史数据
  dist/=20;
  
  Dist1[0]=percent[2]-percent[1];
  dist+=Dist1[0]-Dist1[20];//取20次历史数据
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
    //state=3;           //严重偏离跑道的情况
  } 
  else if(abs(dif_rad)<=1&&abs(dist)<=10&&abs(dist1)<=10&&abs(percent[3]-percent[0])<=40&&abs(percent[2]-percent[1])<=25&&(percent[2]+percent[1])>=100&&abs(distance)<=20)
  {
       mid.time++;      
       if(mid.time>=10)   //消除扰动及抖动     (&参数需根据速度设定)
       {
         curve.time=0;
         fail.time=0;
         left.time=0;right.time=0;
       }     
       if(mid.time>=20)  //直道  
       {
         mid.flag=1;       
         curve.flag=0;
         fail.flag=0;
         ruwan.flag=0;
       }
       else mid.flag=0;
  } 
  else                  //普通弯道
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
      
      if(curve.time>=3 && mid.time>=1000&&abs(percent[3]-percent[0])>30)   //长直道进弯道
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
    //严重偏离跑道的情况
      state=3;    PORTB=0b00001100;
  } 
  else if(mid.flag)
  {
      state=1;    PORTB=0b00001110;//直道正常行走判断
  } 
  else if(ruwan.flag&&curve.time<=80&&curve.time!=0) 
  {
      state=4;    PORTB=0b00001011; //长直到入弯
  }
 
  else if(curve.flag)
  {
      state=5;    PORTB=0b00001010; //弯道正常行走判断
  } 
}

void SlopedownDetect() 
{
  if((percent[1]+percent[2])<=150&&(percent[1]+percent[2])>=100&&abs(percent[2]-percent[1])<=20&&abs(percent[3]-percent[0])<=20) 
  {
       slopedown.time++;      
       if(slopedown.time>=8)   //消除扰动及抖动     (&参数需根据速度设定)
       {
         slopedown.flag=1;
       }     
       else slopedown.flag=0;
  }
}


void Sensor_process(void)   //传感器主函数
{
     GetADValue();
     SelData();
     GetDistance3(); //获得黑线与车子的偏移量
     SlopedownDetect();
     
     if(slopedown.flag==1)  //下坡
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
     //GetInformation();  //粗略判断赛道
     //State_Judge();
} 


void Servo_control()   
//将每个传感器的值依次调至 50 70 70 50 50 69 69 50
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
  
  
  for(i=N_CUN-1;i>=1;i--)     //历史数据的处理
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
  
  if(Angle<Angle_min)  //调节死区
    PWMDTY01=Angle_min+Angle_mid;
  else if(Angle>Angle_max)
    PWMDTY01=Angle_max+Angle_mid;
  else
    PWMDTY01=Angle+Angle_mid; 
 
}

void Speed_select(void)        //速度选档  默认速度加7个档
{
}

void Speed_set() //直道速度要达到3m/s,平均达到2.5m/s
{
  //欲拟合二次曲线，使速度是位置微分量dif的二次函数，且速度还与赛道半径相关
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


void Speed_control()         //电机控制主函数
{
  Vpid.error=Expspeed-Realspeed;            //获得当前的速度偏差
  Vpid.derror=Vpid.error-Vpid.preError;     //获得当前速度偏差的变化率 
  //先用抗饱和积分法            
  if(Vpid.error>=200)   //棒棒控制
  {
    SpeedPID=SpeedMax;   //正大，则正转给全额占空比
  } 
  else if(Vpid.error<=-200) 
  {
    SpeedPID=SpeedMin;    //负大，则自由停车或给一个反转占空比
  } 
  else if(abs(Vpid.error)<=10) 
  {
    SpeedPID=Expspeed;
  }
  else //去掉积分环节   积分分离
  {
    SpeedPID+=12*Vpid.derror+2*Vpid.error;//这个是增量式吧？？？？？
    if(SpeedPID>=SpeedMax)       //死区调节
	      SpeedPID=SpeedMax;
  	else if(SpeedPID<=SpeedMin)
  	    SpeedPID=SpeedMin;
  }
  
  PWMDTY23=SpeedPID;
  Vpid.preError=Vpid.error;      //递推赋值
  Vpid.preDerror=Vpid.derror; 
}



void StopCar()     //停车函数
{
}  
void main(void) 
{
  /* put your own code here */
  
    int i;
    Init_PLL();
    Init_PWM();
    Init_ATD();        //AD转换
    Init_ECT();         //定时器
    Init_IRQ();
    Init_SCI();        //串口初始化
    //Init_fuzzyPID();
    ResetTime_flag();
    Demarcate();  //起跑前标定最大最小值 
    Dly_ms(3000);        //车子延时3秒钟起动    
 	EnableInterrupts;
	  
	TSCR1_TEN =1;  //start Timer 
	EnableInterrupts;
  for(;;) 
  {
    _FEED_COP(); /* feeds the dog */
    
    DDRM=0xff;//M口输出
    PTM=0b00000010;
    //PTM=0b00000010;
    //DDRM=0b00000011;
    Speed_select();  //速度选档   下坡时减速与不减速  直道速度 弯道速度 直道进弯的速度都要设置多个档
       
    for(;;) 
    {
      //执行周期3.8ms
      //PTM=~PTM;
      cnt++;
      if(cnt>=65535) cnt=0;
      
      Sensor_process();   //传感器信息处理
      Servo_control();    //舵机控制 
      
      Speed_set();        //设定速度//
      Speed_control();    //电机控制    
      for(i=N_CUN-1;i>=1;i--)     //历史数据的处理
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







