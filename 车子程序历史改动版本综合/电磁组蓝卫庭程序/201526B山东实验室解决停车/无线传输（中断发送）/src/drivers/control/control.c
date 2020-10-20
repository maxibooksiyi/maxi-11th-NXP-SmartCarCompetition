/********************
函数板块
********************/	
#include "include.h"
#include "flag.h"

int8    front_tested_flag;
int16   front_tested_cnt=0,front_error_add_cnt=0;
int8    behind_tested_flag;
int16   behind_tested_cnt=0,behind_error_add_cnt=0;
void startrun()
{
    if(!Add_1)  
    {
        delayms(5); 
        if(!Add_1)
        {
            while(!Add_1);
            goflag=1; 
            behind_tested_flag=0;
            front_tested_flag=0;
        }
    }
}
/*
     后车通过超声波测距控制AB两车距离
     思想：设置一个距离段30cm-50cm为自由追逐范围段，当距离超过50cm后后车由PD控制增加一项速度增量，当距离过大时要对增量限幅。
           当距离小于30cm时，由PD控制速度减去一个增量，不用限幅。
           当距离过小时（20cm）反向刹车。

*/
void AtoB()
{
    int16 distance;
    preABerror=ABerror; //保存上一次偏差
    distance=ABDistance;//获取当前距离
    if(distance>max_Expect_distance)
    {
      ABerror=distance-max_Expect_distance;
      dtABerror=ABerror-preABerror;
      
      disfb=0.6*ABerror+10*dtABerror;
      
      if(disfb>5)disfb=5;
    }
    else if(distance<25)
    {
      disfb=-(ve+20);
    }
    else if(distance<min_Expect_distance)
    {
      ABerror=distance-max_Expect_distance;
      dtABerror=ABerror-preABerror;
      
      disfb=0.6*ABerror+20*dtABerror;
      
    }
    else
    {
      disfb=0;
    }
    
}
/****************************
  函数名称   Date_analyse
  功能说明： 数据分析 8个电感   
****************************/
void Date_analyse()
{   
    int8  i,a;          
    Read_ADC();    //读取电感AD    
    /*********************归一化处理********************/
    for(i=0;i<7;i++)  //电感        0-100                      
    {  
       sensor_to_one[i] = (float)(AD_valu[i] - min_v[i])/(float)(max_v[i] - min_v[i]);    //最小取值有讲究
       if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
       //if(sensor_to_one[i]>1.0)  sensor_to_one[i]=1.0;  
       AD[i] = (uint16)(100 * sensor_to_one[i]);                 
    }
    for(a=0;a<7;a++)
    {
        if(AD[a]>=mavAD) AD[a]=mavAD;
    }
    //if(AD[3]>=105)  AD[3]=101; 
    if(AD[5]>=101)  AD[5]=100;
    if(AD[6]>=101)  AD[6]=100;
    value42s=AD[4]-AD[2]; 
    value10s=AD[1]-AD[0];
    value65s=AD[6]-AD[5];
    senddata[0]=AD[0];
    senddata[1]=AD[1];
    senddata[2]=AD[2];
    senddata[3]=AD[3];
    senddata[4]=AD[4];
    senddata[5]=AD[5];
    senddata[6]=AD[6];
}
void track()//赛道因素判断a
{   
    //左直角zhijiaoL
//  if(value65s<-40&&AD[3]<50&&AD[3]>AD[0]&&AD[3]>AD[1]&&AD[6]<15&&AD[0]<35&&AD[1]<35&&abs(value10s)<20)
//  {
//      zhijiaoL=1;difference=1;
//      state=1;
//  }
//  //右直角zhijiaoR
//  if(value65s>40&&AD[3]<50&&AD[3]>AD[0]&&AD[3]>AD[1]&&AD[5]<15&&AD[0]<35&&AD[1]<35&&abs(value10s)<20)
//  {
//      zhijiaoR=1;  difference=1;
//      state=2;
//  } 
//  if(difference==1)
//  {
//      speaker(1);
//      difference=0;
//  }
//  else 
//  {speaker(0);}
}
void Error_analyse()
{    int8 i,j;

     /*********************************后排***************************************/
      
      if(AD[2]<flagAD&&AD[4]<flagAD)    //一般非斜十字，其AD值不会超过flagAD，但大于100时
      { 
         area1=30;
        if(AD[2]>100||AD[4]>100)
        {
          if(AD[2]>100&&AD[2]>AD[4]) //AD[2]大于AD[4]
          {
            
            if(AD[4]>80)        error1=-(90-AD[4]);     //-10~-0
            else if(AD[4]>50)   error1=-10-2*(80-AD[4]);      //-40~-10
            else if(AD[4]>30)   error1=-70-3*(50-AD[4]);    //-80~-40
            else                error1=-130-4*(30-AD[4]);//-170~-80
          }
          else
          {
            
            if(AD[2]>80)        error1=(90-AD[2]);
            else if(AD[2]>50)   error1=10+2*(80-AD[2]);
            else if(AD[2]>30)   error1=70+3*(50-AD[2]);
            else                error1=130+4*(30-AD[2]);            
          }
        }
        else            //AD2和AD4都小于100
        {
          area1=40;
            if((AD[4]+2+AD[6]/3)>(AD[2]+AD[5]/3))
            {             
              if(AD[2]>=50)               error1=62-AD[2];        //0-12
              else if(AD[2]>=30)          error1=12+2*(50-AD[2]);  //12-52
              else if(AD[2]>=15)          error1=52+3*(30-AD[2]);  //52-97
              else if(AD[2]>=5)           error1=97+5*(15-AD[2]);  //97-147
              else                        error1=147+6*(5-AD[2]);  //147-177  
            
              if(error1<0)        error1=0;//防止AD2大于80时出错
            }
            else if((AD[2]+2+AD[5]/3)>(AD[4]+AD[6]/3))  
            {
              
              if(AD[4]>=50)               error1=-62+AD[4];    //-12-0   
              else if(AD[4]>=30)          error1=-12-2*(50-AD[4]);//-52--12
              else if(AD[4]>=15)          error1=-52-3*(30-AD[4]);//-97--52
              else if(AD[4]>=5)           error1=-97-5*(15-AD[4]);//-147--97
              else                        error1=-147-6*(5-AD[4]);//-177--147
            
              if(error1>0)        error1=0;
            }
          
        }
      }
      else      //AD2或AD4大于flagAD
      {
        area1=50;
        //这种情况应当分为三个阶段处理，分别用后排，综合，前排三段式处理
        //想在这种情况，不处理还能过，可能是遇到这种情况后保持了打角，然后直接过去了
//        if(AD[2]>=flagAD&&AD[4]<flagAD)
//        {
//            error1=error_1[19]-20;
//        }
//        else if(AD[4]>=flagAD&&AD[2]<flagAD)
//        {
//            error1=error_1[19]+20;
//        }
      }
      if(error1>170)error1=170;
      if(error1<-170)error1=-170;
      if(abs(error1-error_1[19])>190)error1=error_1[19];
     for(i=0;i<19;i++)     //20组历史数据
     {    error_1[i]=error_1[i+1];}
     error_1[19]=error1;
     
     error_1_ave[1]=0;error_1_ave[0]=0;
     for(i=0;i<20;i++)
     {
       error_1_ave[i/10]+=error_1[i];
     }
     error_1_ave[1]/=10;
     error_1_ave[0]/=10;
     
     error1=error_1_ave[1];
     dterror1=error_1_ave[1]-error_1_ave[0]; 
     if(dterror1>50)    dterror1=50;
     else if(dterror1<-50)      dterror1=-50; 
     if(abs(dterror1)>35)       error1=last_error1;
     /*******************************前排电感处理*******************************/
     
     if(AD[0]<flagAD&&AD[1]<flagAD)
     {area2=30;
       if(AD[0]>=100||AD[1]>=100)
       {
          if(AD[0]>100&&(AD[0])>AD[1]) 
          {
            if(AD[1]>80)        error2=-(90-AD[1]);     //-10~-0
            else if(AD[1]>50)   error2=-10-2*(80-AD[1]);      //-70~-10
            else if(AD[1]>30)   error2=-70-4*(50-AD[1]);      //-160~-100
            else                error2=-150-4*(30-AD[1]);    //-280~-160
          }
          else
          {
            if(AD[0]>80)        error2=(90-AD[0]);//-40~-20
            else if(AD[0]>50)   error2=10+2*(80-AD[0]);//-70~-40
            else if(AD[0]>30)   error2=70+4*(50-AD[0]);//-90~-70
            else                error2=150+4*(30-AD[0]);//-200~-90         
          }
       }
       else 
       {
         area2=40;
//           if(AD[3]>=80&&AD[3]<=100&&AD[5]<5&&AD[6]<5)
//           {    
//                state=12;
//                error1=90-AD[3];//-10-+10        
//                if(AD[4]>AD[2]) error2=abs(error2);
//                else            error2=-abs(error2); 
//           }
//          else 
            if((AD[0]+1+AD[5]/2)>(AD[1]+AD[6]/2))
          {
           
            if(AD[1]>=65)            error2=(AD[1]-76);  //-12~0
            else if(AD[1]>=50)       error2=-12-(65-AD[1]); //-27~-12
            else if(AD[1]>=30)       error2=-27-2*(50-AD[1]);//-72~-32
            else if(AD[1]>=15)       error2=-67-4*(30-AD[1]);//-132--72
            else                     error2=-127-5*(15-AD[1]);//-222~-132
          }
          else if((AD[1]+1+AD[6]/2)>(AD[0]+AD[5]/2))
          {
            
            if(AD[0]>=65)            error2=(76-AD[0]);  //0~12
            else if(AD[0]>=50)       error2=10+(65-AD[0]); //12~32
            else if(AD[0]>=30)       error2=27+2*(50-AD[0]);//32~72
            else if(AD[0]>=15)       error2=67+4*(30-AD[0]);//72~132
            else                     error2=127+5*(15-AD[0]);//132~202
          }

       }
     }
     else
     {
       area2=50;
//        if(AD[0]>=flagAD&&AD[1]<flagAD)
//        {
//            error2=error_2[19]-30;
//        }
//        else if(AD[2]>=flagAD&&AD[0]<flagAD)
//        {
//            error2=error_2[19]+30;
//        }
     }
      
      
      //if(error2!=NULL)   //如果前排电感不为无效数据，就进行如下处理
      //{  
        if(error2>200)error2=200;                  //限幅
        else if(error2<-200)error2=-200;
        if(abs(error2-error_2[19])>220)error2=error_2[19];
      
        for(i=0;i<19;i++)     //20组历史数据
        {    error_2[i]=error_2[i+1];}
        error_2[19]=error2;
     
        error_2_ave[1]=0;error_2_ave[0]=0;
        for(i=0;i<20;i++)
        {
          error_2_ave[i/10]+=error_2[i];
        }
        error_2_ave[1]/=10;
        error_2_ave[0]/=10;
        error2=error_2_ave[1];
     
        dterror2=error_2_ave[1]-error_2_ave[0]; 
        if(dterror2>50)    dterror2=50;
        else if(dterror2<-50)      dterror2=-50;
        if(abs(dterror2)>30)       error2=last_error2;
     
     
     for(i=0;i<7;i++)   //保存历史AD值
     {
       for(j=0;j<39;j++)
       {
         AD_history[i][j]=AD_history[i][j+1];
       }
       AD_history[i][39]=AD[i];
     }
     for(i=0;i<7;i++)   //求微分
     {
       for(j=0;j<39;j++)
       {
         AD_history_dif[i][j]=AD_history[i][j+1]-AD_history[i][j];
       }
     }
     for(i=0;i<7;i++)   //清零
     {
        AD_dif_sum[i]=0;
     }
     
     for(i=0;i<7;i++)
     {
       for(j=0;j<39;j++)
       {
         AD_dif_sum[i]+=AD_history_dif[i][j];
       }
     }
}
void Error_end()          //偏差的最终整合,两排偏差的融合,error0范围：-160-+160。error1lp范围：-200-+200。
{                         //主要以error1lp为主，作为判断 
      //error=error2;
  if(K2)
  {
      if(error1==170&&error2==-170)//反向最大
      {
         error=170;
      }
      else if(error1==-170&&error2==170)//反向最大
      {error=-170;}
      else if(error1*error2<0)//如果两个异号，一般来说，认为偏差小，相信数值小的
      {
        if(abs(error1)>50)    //异号，且都较大，相信后排
        {
          error=error1;
        }
        else if(abs(error1)>30)                     //异号，都不是很大，是不是应该相信前排？
        {
          if(abs(error2)>30)        error=error1;
          else if(abs(error2)>10)   error=(error1*8+error2*2)/10;
          else                      error=(error1*7+error2*3)/10;
        }
        else if(abs(error1)>10)
        {
          if(abs(error2)>30)        error=(error1*8+error2*2)/10;
          else if(abs(error2)>10)   error=(error1*7+error2*3)/10;
          else                      error=(error1*6+error2*4)/10;
        }
        else
        {
          if(abs(error2)>30)        error=(error1*8+error2*2)/10;
          else if(abs(error2)>10)   error=(error1*7+error2*3)/10;
          else                      error=(error1*6+error2*4)/10;
        }
      }
      else if(error1*error2>0)      //同号，
      {
          if(abs(error1)>140)
          {
            if(abs(error2)>140)        error=(7*error1+3*error2)/10;
            else if(abs(error2)>100)   error=(7*error1+3*error2)/10;      
            else if(abs(error2)>70)    error=(8*error1+2*error2)/10;
            else if(abs(error2)>50)   error=(9*error1+error2)/10;    //如果相差太大，就相信后排
            else                      error=error1;
          }
          
          else if(abs(error1)>120)
          {
            if(abs(error2)>130)        error=(error1*6+error2*4)/10;
            else if(abs(error2)>100)   error=(error1*6+error2*4)/10;      
            else if(abs(error2)>70)    error=(error1*7+error2*3)/10;
            else if(abs(error2)>50)   error=(error1*7+error2*3)/10;
            else if(abs(error2)>20)    error=(error1*8+error2*2)/10;
            else                      error=(error1*9+error2*1)/10;
          }
          else if(abs(error1)>98)
          {
            ;
          }
          else if(abs(error1)>70)
          {
            if(abs(error2)>130)        error=(error1*2+error2*8)/10;
            else if(abs(error2)>100)   error=(error1*3+error2*7)/10;      
            else if(abs(error2)>70)    error=(error1*4+error2*6)/10;
            else if(abs(error2)>50)   error=(error1*5+error2*5)/10;
            else if(abs(error2)>20)   error=(error1*7+error2*3)/10;
            else                      error=(error1*8+error2*2)/10;
          }
          else if(abs(error1)>68)
          {
            ;
          }
          else if(abs(error1)>40)
          {
            if(abs(error2)>130)        error=(error1*7+error2*3)/10;
            else if(abs(error2)>100)   error=(error1*8+error2*2)/10;      
            else if(abs(error2)>70)    error=(error1*7+error2*3)/10;
            else if(abs(error2)>50)   error=(error1*6+error2*4)/10;
            else if(abs(error2)>20)     error=(error1*7+error2*3)/10;
            else                      error=(error1*8+error2*2)/10;
          }
          else if(abs(error1)>38)
          {
            ;   //类似于继电器特性曲线，由于电感值有±1的跳动，当其在边界处跳动是，必须超过阈值才会跳变
          }
          else
          {
            if(abs(error2)>130)        error=(error1*7+error2*3)/10;
            else if(abs(error2)>100)   error=(error1*7+error2*3)/10;      
            else if(abs(error2)>70)    error=(error1*7+error2*3)/10;
            else if(abs(error2)>50)   error=(error1*6+error2*4)/10;
            else if(abs(error2)>20)   error=(error1*6+error2*4)/10;
            else                      error=(error1*5+error2*5)/10;
          }
      }
      else if(error1==0&&error2!=0)
      {
        if(abs(error2)>50)          error=error2/4;
        else if(abs(error2)>30)     error=4*error2/10;
        else                        error=5*error2/10;
      }
      else if(error1!=0&&error2==0)
      {
        if(abs(error1)>100)         error=error1;
        else if(abs(error1)>50)     error=error1;
        else if(abs(error1)>30)     error=error1*7/10;
        else                        error=error1*6/10;
      }
      else
      {
        error=error1;
      }
   }
  
  else{
      if(error1==170&&error2<=-170)//反向最大
      {
         error=170;
      }
      else if(error1==-170&&error2>=170)//反向最大
      {      
         error=-170;
      }
      else if(error1*error2<0)//如果两个异号，一般来说，认为偏差小，相信数值小的
      {
        if(abs(error1)>50)    //异号，且都较大，相信后排
        {
          error=error1;
        }
        else if(abs(error1)>30)                     //异号，都不是很大，是不是应该相信前排？
        {
          if(abs(error2)>30)        error=error1;
          else if(abs(error2)>10)   error=(error1*8+error2*2)/10;
          else                      error=(error1*7+error2*3)/10;
        }
        else if(abs(error1)>10)
        {
          if(abs(error2)>30)        error=(error1*8+error2*2)/10;
          else if(abs(error2)>10)   error=(error1*7+error2*3)/10;
          else                      error=(error1*6+error2*4)/10;
        }
        else
        {
          if(abs(error2)>30)        error=(error1*8+error2*2)/10;
          else if(abs(error2)>10)   error=(error1*7+error2*3)/10;
          else                      error=(error1*6+error2*4)/10;
        }
      }
      else if(error1*error2>0)      //同号，
      {
          if(abs(error1)>140)
          {
            if(abs(error2)>140)        error=(5*error1+5*error2)/10;
            else if(abs(error2)>100)   error=(6*error1+4*error2)/10;      
            else if(abs(error2)>70)    error=(7*error1+3*error2)/10;
            else if(abs(error2)>50)   error=(8*error1+2*error2)/10;    //如果相差太大，就相信后排
            else                      error=error1;
          }
          
          else if(abs(error1)>120)
          {
            if(abs(error2)>130)        error=(error1*5+error2*5)/10;
            else if(abs(error2)>100)   error=(error1*6+error2*4)/10;      
            else if(abs(error2)>70)    error=(error1*7+error2*3)/10;
            else if(abs(error2)>50)   error=(error1*8+error2*2)/10;
            else if(abs(error2)>20)    error=(error1*9+error2*1)/10;
            else                      error=(error1*9+error2*1)/10;
          }
          else if(abs(error1)>100)
          {
            if(abs(error2)>130)        error=(error1*4+error2*6)/10;
            else if(abs(error2)>100)   error=(error1*5+error2*5)/10;      
            else if(abs(error2)>70)    error=(error1*6+error2*4)/10;
            else if(abs(error2)>50)   error=(error1*7+error2*3)/10;
            else if(abs(error2)>20)    error=(error1*8+error2*2)/10;
            else                      error=(error1*9+error2*1)/10;
          }
          else if(abs(error1)>80)
          {
            if(abs(error2)>130)        error=(error1*3+error2*7)/10;
            else if(abs(error2)>100)   error=(error1*4+error2*6)/10;      
            else if(abs(error2)>70)    error=(error1*5+error2*5)/10;
            else if(abs(error2)>50)   error=(error1*6+error2*4)/10;
            else if(abs(error2)>20)   error=(error1*8+error2*2)/10;
            else                      error=(error1*9+error2*1)/10;
          }
          else if(abs(error1)>60)
          {
            if(abs(error2)>130)        error=(error1*2+error2*8)/10;
            else if(abs(error2)>100)   error=(error1*3+error2*7)/10;      
            else if(abs(error2)>70)    error=(error1*4+error2*6)/10;
            else if(abs(error2)>50)   error=(error1*5+error2*5)/10;
            else if(abs(error2)>20)   error=(error1*8+error2*2)/10;
            else                      error=(error1*9+error2*1)/10;
          }
          else if(abs(error1)>40)
          {
            if(abs(error2)>130)        error=(error1*1+error2*9)/10;
            else if(abs(error2)>100)   error=(error1*2+error2*8)/10;      
            else if(abs(error2)>70)    error=(error1*3+error2*7)/10;
            else if(abs(error2)>50)   error=(error1*4+error2*6)/10;
            else if(abs(error2)>20)   error=(error1*4+error2*6)/10;
            else                      error=(error1*5+error2*5)/10;
          }
          else if(abs(error1)>38)
          {
            ;   //类似于继电器特性曲线，由于电感值有±1的跳动，当其在边界处跳动是，必须超过阈值才会跳变
          }
          else
          {
            if(abs(error2)>130)        error=(error1*3+error2*7)/10;
            else if(abs(error2)>100)   error=(error1*3+error2*7)/10;      
            else if(abs(error2)>70)    error=(error1*4+error2*6)/10;
            else if(abs(error2)>50)   error=(error1*4+error2*6)/10;
            else if(abs(error2)>20)   error=(error1*5+error2*5)/10;
            else                      error=(error1*5+error2*5)/10;
          }
      }
      else if(error1==0&&error2!=0)
      {
        if(abs(error2)>50)          error=error2/2;
        else if(abs(error2)>30)     error=6*error2/10;
        else                        error=7*error2/10;
      }
      else if(error1!=0&&error2==0)
      {
        if(abs(error1)>100)         error=error1;
        else if(abs(error1)>50)     error=error1;
        else if(abs(error1)>30)     error=error1*7/10;
        else                        error=error1*6/10;
      }
      else
      {
        error=error1*4+error2*6;
      }
  }
  if(!right_angle_flag)
  {
    if(abs(value65s)>40&&(AD[5]<15||AD[6]<15))       //直角
    {
      if(AD[0]<50&&AD[1]<50&&abs(value10s)<20&&AD[3]>AD[1]&&AD[3]>AD[0])
      {
      
        if(AD_dif_sum[0]<0&&AD_dif_sum[1]<0&&AD_dif_sum[2]<0&&AD_dif_sum[3]<0&&AD_dif_sum[4]<0)
        {
          if(abs(error1)<20&&abs(error2)>100)
          {      
            right_angle_flag=1;
          }
          else if(abs(error1)<40&&abs(error2)>130)
          {
            right_angle_flag=1;
          }
          else if(abs(error1)<50&&abs(error2)>140)
          {
            right_angle_flag=1;
          }
          else if(AD[0]<30&&AD[1]<30)
          {
            right_angle_flag=1;
          }
//          else if(abs(value65s)>30)
//          {
//            right_angle_flag=1;
//          }
          zhijiao_start=Direction;
        }
      }
      else
      {
        ;
      }
    }
    if(AD[5]>80&&AD[6]>80)
    {
       if(error*value65s<0)
         error=error+value65s;
       error/=2;
    }
    else if(AD[5]>25&&AD[6]>25&&AD[3]>70)//十字
    {
      if(AD[3]>80&&abs(AD[2]-AD[4])<30&&abs(AD[0]-AD[1])>10&&(AD[2]>60||AD[4]>60))
      {
          error=(error1*7+error2*3)/10;
      }
      else
      {
        error=(6*error1+4*error2)/10;
      }   
    }
    else if(abs(AD[5]-AD[6])<10&&AD[5]<10&&AD[6]<10&&abs(value42s)<30&&abs(value10s)<30)//直道
    {
      if(AD[3]>80)
      {      
        error=(AD[4]-(AD[4]+AD[2])/2)/3;
      }
    }
    else//弯道
    {
    //error=error1;
    }
  }
  //直角处理
  if(right_angle_flag)
  {
    speaker(1);
    flag_cnt++;
    if((AD[5])>(AD[6]))
      error=-140;
    else if((AD[5])<(AD[6]))
      error=140;
    if(flag_cnt>200||abs_f(Direction-zhijiao_start)>70)
    {
      right_angle_flag=0;
      zhijiaoL=0;zhijiaoR=0;
      flag_cnt=0;
      speaker(0);
    }
  }
  if(K5)
  {
  //斜十字处理
  if(!front_tested_flag)
  { 
    if(((AD[0]>flagAD&&AD[1]<flagAD)||(AD[0]<flagAD&&AD[1]>flagAD))&&(AD[5]<20||AD[6]<20)&&abs(value42s)>40)
    {
        front_tested_flag=1;
        xieshi_front_start=Distance;
    }

  }
  
  if(front_tested_flag&&(Distance-xieshi_front_start>500))
  { 

    speaker(0);
    front_tested_flag=0;state=0;xieshi_front_start=0;
  }
  if(front_tested_flag==1)  
  {  speaker(1);
     state=30;
     if(error<0)      error-=40;
     else if(error>0) error+=40;
  }
  if(!behind_tested_flag)
  { 
    if((AD[2]>flagAD&&AD[4]<flagAD)||(AD[2]<flagAD&&AD[4]>flagAD)&&(AD[5]<20||AD[6]<20))
    {
        behind_tested_flag=1;
        xieshi_behind_start=Distance;
    }
  }
  
  if(behind_tested_flag&&(Distance-xieshi_behind_start>300))
  {     
    speaker(0);
      behind_tested_flag=0;state=0;xieshi_behind_start=0;

  }
  if(behind_tested_flag==1)
  {  speaker(1);
     state=30;
     if(error<0)      error-=40;
     else if(error>0) error+=40;
  }
  }


//     //坡道检测
//  if(ramp_start==0||Distance-ramp_start>5000)    //坡道
//  {
//    if(Angle_y>3000/*&&AD[1]>130&&AD[0]>120*/&&ramp1==0&&K3)
//    {
//      ramp1=1;
//      disable_irq(PORTE+87);
//      ramp_start=Distance;
//     }
//    if(Angle_y>4000&&AD[1]<50&&AD[0]<50&&AD[2]<50&&AD[3]<50&&AD[4]<50&&ramp1==1&&ramp2==0&&K3)
//    {
//      ramp1=0;
//      ramp2=1;
//    }
//    if(Angle_y<-1400&&AD[0]>70&&AD[1]>70&&AD[3]>70&&ramp2==1&&ramp3==0&&K3)
//    {
//      ramp2=0;
//      ramp3=1;
//    }
//  }
//    if(ramp1==1)
//    {
//      speaker(1);
//      error=value42s/3;
//      
//    }
//    if(ramp2==1)
//    {
//      speaker(1);
//      error=value42s/4;
//     
//    }
//    if(ramp3==1)
//    {
//      speaker(1);
//      error=value42s/3;
//      
//    }
//    if((ramp3==1||ramp1==1||ramp2==1)&&(Distance-ramp_start>ramp_distance))//ramp1==1||ramp2==1||
//   {
//      ramp1=0;
//      ramp2=0;
//      ramp3=0;speaker(0);ramp_over_time=runtime;
//      //ramp_delay=1000;
//   }
//     if(ramp_over_time-runtime==500)
//   {
//      enable_irq(PORTE+87);
//   }
    if(K3&&(ramp_start==0||(Distance-ramp_start>5000)))
  {
   if(Angle_y>3000&&ramp==0&&K3)
   {
      ramp=1;
      state=31;
      ramp_start=Distance;
   }
  }
   if(ramp==1&&(Distance-ramp_start>ramp_distance))
   {
      ramp=0;speaker(0);
      //ramp_delay=1000;
      enable_irq(PORTE+87);//重新使能干簧外部中断
   }
   if(ramp==1)   
   {  speaker(1);
      error=value42s/2;
      disable_irq(PORTE+87);//坡道关闭干簧外部中断
      enablestop=0;//不允许停车
   }
  
}
void servoPD()//error1lp范围-220~+220
{   
//     servo_kp=skp+abs(error)/40+(speedve-200)/10;
//     servo_kd=skd+2*(speedve-200)+2.5*abs(dterror1+dterror0);

}
void Steer_angle()        //舵机转角
{   
    int8 s;
    Angle=(int16)(servo_kp*error+skd*dterror2); 
    for(s=0;s<19;s++)    //记录角度PWM偏差
    {  Angle_history[s]=Angle_history[s+1];
    }
    Angle_history[19]=Angle;
    dtAngle=abs(Angle_history[19])-abs(Angle_history[10]);
    
    last_error=error;
    last_error1=error1;
    last_error2=error2;
}
/******************************
 舵机控制3100--3100
 目前解决：
******************************/
void Steer(void)
{   
    prepwm=servo_PWM;//上次转角
    
    servo_PWM=steer_mid+Angle;//本次转角
    dtspwm=servo_PWM-prepwm;//舵机实际转角的微分，即前后两次的变化程度
    if(servo_PWM<SERVO_LEFT_MAX_PWM)      //限幅
        servo_PWM=SERVO_LEFT_MAX_PWM;     //防止舵机超过极限值
    if(servo_PWM>SERVO_RIGHT_MAX_PWM)
        servo_PWM=SERVO_RIGHT_MAX_PWM;
    Steer_duty(servo_PWM);
}
void motor()
{
  Motor_go(speedve*2);   

}
/**************************
 获取速度
**************************/
void get_speed(void)
{  
   vnpoint++;                     //
   if(vnpoint>=Vnhnum)vnpoint=0;
   vnh[vnpoint]=FTM2_CNT;//测速
   FTM2_CNT=0;
   vnk=vnk+vnkk*(vnh[vnpoint]-vnk);//速度滤波
   vn=(int)vnk;//现在速度 
   Distance+=(double)(vn)/52.95;
}
/**************************
 速度分段给定
**************************/
void speed_set()              //速度的设定
{       
     if(K4)
     {
         // AtoB();
          //ve=speedve*53/100+disfb;
          ve=speedve*53/100;
          if(abs(Angle)<700)
          {ve=ve+70;}
          else if(abs(Angle)>700&&abs(Angle)<=1000)
          {
              ve=ve+(700-abs(Angle))/30+40;
          }
          else if(abs(Angle)>1000&&abs(Angle)<=1600)
          {
              ve=ve+(1000-abs(Angle))/60+30;
          }
          else if(abs(Angle)>1600&&abs(Angle)<=2200)
          {
            ve=ve+(1600-abs(Angle))/60+20;
          }
          else //if(abs(Angle)>3000&&abs(Angle)<4000)
          {
            ve=ve+10;

          }
     }

     else 
     {   //AtoB();
          //ve=speedve*53/100+disfb; 
          ve=speedve*53/100;
          if(abs(Angle)<700)
          {ve=ve+40;}
          else if(abs(Angle)>700&&abs(Angle)<1000)
         {
            ve=ve+(700-abs(Angle))/20+25;
          }
         else if(abs(Angle)>1000&&abs(Angle)<1300)
         {
            ve=ve+(1000-abs(Angle))/30+10;
          }
          
     }
     
     
      if(right_angle_flag)
      {
          ve=right_angle_v*53/100;
      }
//      else if(ramp1==1)
//      {
//          ve=ve_ramp*53/100+10;
//      }
//      else if(ramp2==1)
//      {
//          ve=ve_ramp*53/100-10;
//      }
//      else if(ramp3==1)
//      {
//          ve=ve_ramp*53/100+20;
//      }
       else if(ramp==1)
     {    
      ve=ve_ramp*53/100;
      
      }
      else if(state==30)
      {
          ve=200*53/100;
      }
      else if(K1&&(AD[5]<10||AD[6]<10)&&(area1==30||area2==30))
     {
        ve=ve-80;
        servo_kp=skp+2;
     }
      else
      {servo_kp=skp;}
     if(runflag==0)//定时时间到了，停车
     {ve=0;}
     else if(stopflag==1)//定时和终点到了停车
     {ve=0;}
     else if(enstop==1)//串口接收到了停车
     {ve=0;}
     else if(enablestop==1&&(Distance-stopdistance)>700)
     {ve=0;stopflagdelay=1;}
     
     if(stopflagdelay==1)
     {
          countstop++;
     }

}
/********************************************
  
  函数名称   motorPID()
  功能说明： 电机PID调节    由空载特性曲线为vn=0.4904PWM-25.88;
*************************/
void motorPID()              //PID典型电机控制
{
   prevnerr=vnerr;//上一次速度误差
   vnerr=ve-vn;//本次速度误差
   predtvn=dtvn;//上一次误差变化率
   dtvn=vnerr-prevnerr;//本次误差变化率
   mobase=mokb*ve;
   mobasev=mokb*vn;
   if(mobase>mobasev)mobase=mobasev;
   motorP=mokp*vnerr;
   motorP=fade(motorP,2000);
   motorI+=moki*vnerr;
   motorI=fade(motorI,moimax);
   motorD=mokd*dtvn;
   motorout=(int)((mobase+motorP+motorI+motorD)*power0/power);
   //motorout=ade(motorout,2000);
   if(motorout>2000)motorout=2000;
   if(motorout<-1500)motorout=-1500;
   //if(AD[2]<1&&AD[3]<1&&AD[4]<1&&AD[5]<1&&AD[6]<1){motorout=20;}
   //if(K8){motorout=0;}
   if(motorout>0){Motor_go(motorout);}
   else if(motorout<=0){Motor_back(abs(motorout));}
   else {Motor_go(1);}
   premotorout=motorout;
//  senddata[0]=ve;
//  senddata[1]=vn;
//  senddata[2]=motorP;
//  senddata[3]=motorI;
//  senddata[4]=motorD;
//  senddata[5]=motorout;
}
/********************************************
  
  函数名称   motorPI()
  功能说明： 电机PID调节    由空载特性曲线为vn=0.4904PWM-25.88;
*************************/
void motorPI()              //PI典型电机控制
{  
   //ve=speedchoose[SPN]/2;
   prevnerr=vnerr;//上一次速度误差
   vnerr=(int16)(ve-vn);//本次速度误差
   predtvn=dtvn;//上一次误差变化率
   dtvn=vnerr-prevnerr;//本次误差变化率
   if(vnerr<=-60)
   {
      motorout=-2000;
   }
   else if(vnerr>=60)
   {
      motorout=1500;
   }
   else
   {
   motorI=mki*vnerr;
   motorP=mkp*dtvn;
   motorpw=(int16)((motorP+motorI/2)/10);
   motorout+=motorpw;
   }
   if(motorout>1500)motorout=1500;
   if(motorout<-2000)motorout=-2000;
   if(AD[2]<1&&AD[3]<1&&AD[4]<1&&AD[5]<1&&AD[6]<1){motorout=20;}
   
   //堵转保护
//   if(abs(motorout)>1000&&abs_f(vn)<5)   motor_protect_cnt++;
//   else                                 motor_protect_cnt/=2;
   
   //if(motor_protect_cnt>50)     motorout=0;
   //if(enablestop==1&&(Distance-stopdistance)>100){motorout=5;} 
   if(countstop>=1500)
   {
       motorout=1;
       stopflagdelay=0;
   }
   if(motorout>0){Motor_go(motorout);}
   else if(motorout<=0){Motor_back(abs(motorout));}
   else {Motor_go(1);}
   premotorout=motorout;
  
}
void motorstop()
{
    speed_set();
    motorPI();  
}
/***********************************预显示**********************************/
void pre_show(void)
{    
   switch(page_num)
   { 
      case 0: LCD_CLS();
            write_6_8_number(120,7,page_num+1);
            write_6_8_char(0,line_num,'*');
            write_6_8_string(6,0,"ve:");write_6_8_number(24,0,speedve);
            write_6_8_string(6,1,"sP:");write_6_8_number(24,1,skp);            
            write_6_8_string(6,2,"sD:");write_6_8_number(24,2,skd);            
            write_6_8_string(6,3,"zjv:");write_6_8_number(30,3,right_angle_v);
            write_6_8_string(6,4,"rav:");write_6_8_number(30,4,ve_ramp);
            write_6_8_string(6,5,"raD:");write_6_8_number(30,5,ramp_distance);
            write_6_8_string(6,6,"fA:");write_6_8_number(24,6,flagAD);           
            write_6_8_string(60,6,"vn:");write_6_8_number(88,6,vn);            
            write_6_8_string(6,7,"pU:");write_6_8_number(24,7,powerU); 
            write_6_8_string(60,1,"Dist:");
            write_6_8_number(88,1,(int)(Distance));
            write_6_8_string(60,2,"Dire:");
            write_6_8_number(88,2,(int)(Direction));            
            write_6_8_string(60,3,"juli:");write_6_8_string(114,3,"cm");
            write_6_8_number(88,3,ABDistance);
            write_6_8_string(60,4,"runt:");
            write_6_8_number(88,4,runtime);
            write_6_8_string(60,5,"Go:");
            write_6_8_number(88,5,goflag);            
            break;
    case 1:LCD_CLS();
            write_6_8_number(120,60,page_num+1);       
            write_6_8_string(0,0,"AD0:");write_6_8_number(24,0,ADC[0]);write_6_8_number(50,0,(ADC[0]*3.3/1023));write_6_8_number(90,0,AD[0]);
            write_6_8_string(0,1,"AD1:");write_6_8_number(24,1,ADC[1]);write_6_8_number(50,1,(ADC[1]*3.3/1023));write_6_8_number(90,1,AD[1]);
            write_6_8_string(0,2,"AD2:");write_6_8_number(24,2,ADC[2]);write_6_8_number(50,2,(ADC[2]*3.3/1023));write_6_8_number(90,2,AD[2]);
            write_6_8_string(0,3,"AD3:");write_6_8_number(24,3,ADC[3]);write_6_8_number(50,3,(ADC[3]*3.3/1023));write_6_8_number(90,3,AD[3]);
            write_6_8_string(0,4,"AD4:");write_6_8_number(24,4,ADC[4]);write_6_8_number(50,4,(ADC[4]*3.3/1023));write_6_8_number(90,4,AD[4]);
            write_6_8_string(0,5,"AD5:");write_6_8_number(24,5,ADC[5]);write_6_8_number(50,5,(ADC[5]*3.3/1023));write_6_8_number(90,5,AD[5]);
            write_6_8_string(0,6,"AD6:");write_6_8_number(24,6,ADC[6]);write_6_8_number(50,6,(ADC[6]*3.3/1023));write_6_8_number(90,6,AD[6]);
            //write_6_8_string(0,7,"AD7:");write_6_8_number(24,6,ADC[7]);write_6_8_number(50,7,(ADC[7]*3.3/1023));write_6_8_number(90,7,AD[7]);
            
            break;
    case 2:LCD_CLS();
            write_6_8_number(120,60,page_num+1);
            write_6_8_string(0,0,"Agl:");write_6_8_number(24,0,Angle);
            write_6_8_string(0,1,"Spw:");write_6_8_number(24,1,servo_PWM);
            write_6_8_string(0,2,"v42:");write_6_8_number(24,2,value42s);write_6_8_string(60,2,"v10:");write_6_8_number(84,2,value10s);
            write_6_8_string(0,3,"er1:");write_6_8_number(24,3,error1);  write_6_8_string(60,3,"er2:");write_6_8_number(84,3,error2);
            write_6_8_string(0,4,"AD2:");write_6_8_number(24,4,AD[2]);   write_6_8_string(60,4,"AD0:");write_6_8_number(84,4,AD[0]);
            write_6_8_string(0,5,"AD4:");write_6_8_number(24,5,AD[4]);   write_6_8_string(60,5,"AD1:");write_6_8_number(84,5,AD[1]);
            write_6_8_string(0,6,"AD3:");write_6_8_number(24,6,AD[3]);   //write_6_8_string(60,6,"AD5:");write_6_8_number(84,6,AD[5]);      
            write_6_8_string(0,7,"erd:");write_6_8_number(24,7,error);  write_6_8_string(60,7,"sta:");write_6_8_number(84,7,state); 
            break;        
    }
}

/**************************************刷屏，显示时变变量*********************************/
void redraw()
{
   switch(page_num)
   { 
   case 0: 
            write_6_8_number(120,7,page_num+1);
            write_6_8_char(0,line_num,'*');
            write_6_8_number(24,0,speedve);
            write_6_8_number(24,1,skp);
            write_6_8_number(24,2,skd);
            write_6_8_number(30,3,right_angle_v);
            write_6_8_number(30,4,ve_ramp);
            write_6_8_number(30,5,ramp_distance);
            write_6_8_number(24,6,flagAD);
            write_6_8_number(24,7,powerU);write_6_8_number(60,7,powerUvalue);
            write_6_8_number(60,0,(int)(180*Angle_x/(32767*3.14)));
            write_6_8_number(90,0,(int)(180*Angle_y/(32767*3.14)));
            write_6_8_number(88,1,(int)(Distance));
            write_6_8_number(88,2,(int)(Direction));
            write_6_8_number(88,3,ABDistance);
            write_6_8_number(88,4,runtime);
            write_6_8_number(88,5,goflag);            
            write_6_8_number(88,6,vn);          
            
 
            break;
    case 1: 
            write_6_8_number(120,60,page_num+1);
            write_6_8_number(24,0,ADC[0]); write_6_8_number(50,0,(ADC[0]*3.3/1023)); write_6_8_number(90,0,AD[0]);         
            write_6_8_number(24,1,ADC[1]); write_6_8_number(50,1,(ADC[1]*3.3/1023)); write_6_8_number(90,1,AD[1]);          
            write_6_8_number(24,2,ADC[2]); write_6_8_number(50,2,(ADC[2]*3.3/1023)); write_6_8_number(90,2,AD[2]);         
            write_6_8_number(24,3,ADC[3]); write_6_8_number(50,3,(ADC[3]*3.3/1023)); write_6_8_number(90,3,AD[3]);          
            write_6_8_number(24,4,ADC[4]); write_6_8_number(50,4,(ADC[4]*3.3/1023)); write_6_8_number(90,4,AD[4]);          
            write_6_8_number(24,5,ADC[5]); write_6_8_number(50,5,(ADC[5]*3.3/1023)); write_6_8_number(90,5,AD[5]);         
            write_6_8_number(24,6,ADC[6]); write_6_8_number(50,6,(ADC[6]*3.3/1023)); write_6_8_number(90,6,AD[6]);
            //write_6_8_number(24,7,ADC[7]); write_6_8_number(50,7,(ADC[7]*3.3/1023)); write_6_8_number(90,7,AD[7]);
            
            break;
    case 2: 
            write_6_8_number(120,60,page_num+1);
            write_6_8_number(24,0,Angle);          
            write_6_8_number(24,1,servo_PWM);          
            write_6_8_number(24,2,value42s);write_6_8_number(84,2,value10s); 
            write_6_8_number(24,3,error1);write_6_8_number(84,3,error2);
            write_6_8_number(24,4,AD[2]); write_6_8_number(84,4,AD[0]);
            write_6_8_number(24,5,AD[4]); write_6_8_number(84,5,AD[1]);
            write_6_8_number(24,6,AD[3]);//write_6_8_number(84,6,AD[5]);
            write_6_8_number(24,7,error);write_6_8_number(84,7,state);
            break;
  } 
}
/*********************************修改变量数值***********************************/
void change_value(unsigned char page,unsigned char m,float i)
{  
  switch (page)
  {  
  case 0:
     switch(m)
        { 
         case 0:speedve+=i; LCD_clear_L(30,6);write_6_8_char(0,6,'*');write_6_8_number(24,6,speedve);break;                   
         case 1:skp+=i; LCD_clear_L(24,0);write_6_8_char(0,0,'*');write_6_8_number(24,0,skp);break;               
         case 2:skd+=i; LCD_clear_L(24,1);write_6_8_char(0,1,'*');write_6_8_number(24,1,skd);break;            
         case 3:right_angle_v+=i; LCD_clear_L(30,2);write_6_8_char(0,2,'*');write_6_8_number(30,2,right_angle_v);break;
         case 4:ve_ramp+=i; LCD_clear_L(30,3);write_6_8_char(0,3,'*');write_6_8_number(30,3,ve_ramp);break;
         case 5:ramp_distance+=i*100; LCD_clear_L(30,4);write_6_8_char(0,4,'*');write_6_8_number(30,4,ramp_distance);break;
         case 6:flagAD+=i; LCD_clear_L(24,5);write_6_8_char(0,5,'*');write_6_8_number(24,5,flagAD);break;       
         case 7:runtime+=i*1000; LCD_clear_L(88,4);write_6_8_char(0,4,'*');write_6_8_number(88,4,runtime);break;
        }
  }	 		    
}
void send()
{
        sendmydata((uint8 *)senddata,sizeof(senddata));
        //sendNData(senddata,5);//无线发送数据
}
/***************************************
 舵机中值调节函数
 舵机中值不对了可以脱机修改舵机中值 
***************************************/
void Steer_adjust(void)
{ 
    int16 adjust = 0;    
    adjust=(Key1==0)? 1:0; 
    turn_add = flash_read(SECTOR_TURN,0,int16);	 
    old_turn_add=turn_add;
    while(adjust)
    {         
       LCD_Print(45,2,"Steer"); 
       LCD_Print(30,4,"adjusting..."); 
       if(Key2)                  //左
       {delayms(1);turn_add += 10;while(Key2);     }
       if(Key3)                  //右
       {delayms(1);turn_add -= 10;while(Key3);     } 
       steer_mid = SERVO_MID_PWM +turn_add;
       Steer_duty(steer_mid);                    
       write_8_16_number(70,6,turn_add);      
       if(Key1)               
       {
           if(abs(turn_add-old_turn_add))                   //有改变写入
           {       flash_erase_sector(SECTOR_TURN);         //擦除扇区
                   flash_write(SECTOR_TURN,0,turn_add);     //参数写入扇区          
           }
           LCD_Print(30,6,"Quit");
           adjust = 0;
           while(Key1);               
           LCD_CLS();    
       }
    }
    turn_add = flash_read(SECTOR_TURN,0,int16);		
    steer_mid = SERVO_MID_PWM + turn_add;
}  
/********************************
  函数名称   Read_power
  功能说明： 采集电源电池电压和驱动电流
********************************/
void Read_powerU(void)
{    
     powerUvalue=ad_ave(ADC1,AD16,ADC_12bit,10);     // ADC1 通道     ADC1_SE16   采集电源电压
     powerU=((powerUvalue*3.3)/4095)*3.12;         //分压电阻网络不是很准,3.16的补偿还行  A车：2.92   B车：3.15
     power=powerUvalue;
      if(i2cdisable==1)
      {
        I2C0_D;
        I2C1_D;
        I2C0_C1 &= ~I2C_C1_MST_MASK;
        I2C0_C1 &= ~I2C_C1_TX_MASK;
        I2C1_C1 &= ~I2C_C1_MST_MASK;
        I2C1_C1 &= ~I2C_C1_TX_MASK;
        i2cdisable=0;
        mmainit();
        if(i2cdisable==0)
          mpu6050init();
      }
}
void Read_powerI(void)
{
      int8 i;
      preIvalue=Ivalue;
      powerIvalue=getad(ADC0,AD14,ADC_12bit);   // ADC0 通道     ADC0_SE16   采集驱动电流
      Ivalue=(powerIvalue+preIvalue*3)/4;
     for(i=0;i<19;i++)     //20组历史数据
     {    dtIvalue[i]=dtIvalue[i+1];}
      dtIvalue[19]=Ivalue-3110;
     for(i=0;i<20;i++)
     {
           sumI+=dtIvalue[i];
     }
     sumI=sumI/20;
     dianliu=sumI/80;
     powerI=((Ivalue*3.3)/4095);//电流转化为电压显示

}

/********************************
  函数名称   SC_black_Init
  功能说明： 最大值,水平值采样
********************************/
void SC_black_Init(void)
{
    uint16  i,j;
   //水平和斜电感采样
   if(K1)                                        
   {
       LCD_Print(25,2,"Collecting"); 
       LCD_Print(28,4,"samples(8)"); 
       max_v[1]=max_v[2]=max_v[3]=max_v[4]=max_v[0]= 0;    
       min_v[1]=min_v[2]=min_v[3]=min_v[4]=min_v[0]=10;              //最小有讲究？？？？    //实际取20差不多了
       for(i=0;i<1500;i++) 
       {
           AD_valu[0]=getad(ADC1,AD9,ADC_10bit);     // ADC1 通道     PTB1       前左斜
           AD_valu[1]=getad(ADC0,AD12,ADC_10bit);    // ADC0 通道     PTB2       前右斜           
           AD_valu[2]=getad(ADC1,AD10,ADC_10bit);    // ADC1 通道     PTB4       中左
           AD_valu[3]=getad(ADC1,AD11,ADC_10bit);    // ADC1 通道     PTB5       中中
           AD_valu[4]=getad(ADC1,AD12,ADC_10bit);    // ADC1 通道     PTB6       中右             
           for(j=0;j<5;j++) 
           {if(AD_valu[j] > max_v[j])max_v[j] = AD_valu[j];}
           delayms(1);           //延时	
       }
       flash_erase_sector(SECTOR_ADD);                  //擦除251扇区
       for(i=0; i<5; i++)
       {flash_write(SECTOR_ADD,i*4,max_v[i]);     //讲所有参数写入扇区
       } 
       LCD_Print(28,6,"finish");
   } 
   else
   {
       for(i=0;i<4;i++)
       { 
          for(j=0;j<5;j++)
          {   max_v[j] = flash_read(SECTOR_ADD,j*4,int16);}
          LCD_Print(29,2,"Reading"); 
          LCD_Print(28,4,"samples(8)"); 
          delayms(50);           
       }
   }
   while(K1);
   LCD_CLS();
   //垂直电感采样  
    if(K2)                                        
   {
       LCD_Print(25,2,"Collecting"); 
       LCD_Print(28,4,"samples(zhi)");        
       max_v[5]=max_v[6]= 0;    
       min_v[5]=min_v[6]= 20;     
       for(i=0;i<1200;i++) 
       {   AD_valu[5] = getad(ADC1,AD8,ADC_10bit);    //      左垂直       
           AD_valu[6] = getad(ADC0,AD13,ADC_10bit);    //      右垂直                                                            
           if(AD_valu[5] > max_v[5])   max_v[5] = AD_valu[5]; 
           if(AD_valu[6] > max_v[6])   max_v[6] = AD_valu[6];                                        
           delayms(1);           //延时	
       }     
       flash_erase_sector(SECTOR_ADM);                  //擦除252扇区
       flash_write(SECTOR_ADM,0,max_v[5]);          //讲所有参数写入扇区
       flash_write(SECTOR_ADM,4,max_v[6]);
       LCD_Print(28,6,"finish");      
   }
   else
   {
       for(i=0;i<3;i++)
       {  max_v[5] = flash_read(SECTOR_ADM,0,int16);
          max_v[6] = flash_read(SECTOR_ADM,4,int16);
          LCD_Print(29,2,"Reading"); 
          LCD_Print(28,4,"samples(zhi)"); 
          delayms(50);           
       }
   } 
   while(K2); 
   LCD_CLS(); 
}

/*******************
函数名称   Read_ADC
功能说明： AD采集
********************/
void Read_ADC(void)
{
     int16  i,j,k,temp; 
     int16  ad_valu[7][5],ad_valu1[7],ad_sum[7];           
     for(i=0;i<5;i++)//每个采5次
     {   
         ad_valu[0][i]=getad(ADC1,AD9,ADC_10bit);     // ADC1 通道     PTB1       
         ad_valu[1][i]=getad(ADC0,AD12,ADC_10bit);    // ADC0 通道     PTB2       
         ad_valu[2][i]=getad(ADC1,AD10,ADC_10bit);    // ADC1 通道     PTB4       
         ad_valu[3][i]=getad(ADC1,AD11,ADC_10bit);    // ADC1 通道     PTB5       
         ad_valu[4][i]=getad(ADC1,AD12,ADC_10bit);    // ADC1 通道     PTB7       
         ad_valu[5][i]=getad(ADC1,AD8,ADC_10bit);     // ADC1 通道     PTB0       
         ad_valu[6][i]=getad(ADC0,AD13,ADC_10bit);    // ADC0 通道     PTB3                 
             
     }
     //////////////////////冒泡排序////////////////////////////////
     for(i=0;i<7;i++)          
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
            
              if(ad_valu[i][k] > ad_valu[i][k+1])  //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              } 
           }
        }
     }
     for(i=0;i<7;i++)    //求中间三项的和      //舍去极端，中值滤波法
     {      
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];       
        ad_valu1[i] = ad_sum[i] / 3;
     }    
     ////////////////////////递推平均滤波/////////////////////////////      
     for(i = 0;i < NM-1;i ++)
     {      
         AD_V[0][i] = AD_V[0][i + 1];
         AD_V[1][i] = AD_V[1][i + 1];
         AD_V[2][i] = AD_V[2][i + 1];
         AD_V[3][i] = AD_V[3][i + 1];
         AD_V[4][i] = AD_V[4][i + 1];
         AD_V[5][i] = AD_V[5][i + 1];
         AD_V[6][i] = AD_V[6][i + 1];     
     }
     for(i=0;i<7;i++)
     {
         AD_V[i][NM-1]=ad_valu1[i];
     }    
     for(i = 0;i < NM;i ++)
     {      
         AD_sum[0] += AD_V[0][i];
         AD_sum[1] += AD_V[1][i];
         AD_sum[2] += AD_V[2][i];
         AD_sum[3] += AD_V[3][i];
         AD_sum[4] += AD_V[4][i];
         AD_sum[5] += AD_V[5][i];
         AD_sum[6] += AD_V[6][i];
     }   
     for(i=0;i<7;i++)  //求平均
     {        
         AD_valu[i] = (int16)(AD_sum[i]/NM);         
         AD_sum[i] = 0; 
     }  
     for(i=0;i<7;i++) //转化为具体电压值
     {
         ADC[i]=AD_valu[i];
         ADre[i]=(ADC[i]/1023)*3.3;
     }
}

  
/*************************按键扫描函数***************************/
void KeyScan(void)
{   //切屏
    if(!change_page)            //如果检测到低电平，说明按键按下
    {delayms(5);                //延时去抖，一般10-20ms  
       if(!change_page)         //再次确认按键是否按下，没有按下则退出
       {while(!change_page);    //如果确认按下按键等待按键释放，没有释放则一直等待
        if(page_num<3) page_num++;   //页序号加操作
	else           page_num=0;        
	line_num=0;	   
	pre_show();               
        }
     }   
    if(page_num==0||page_num==2) //如不为第一页，则进行下一步扫描
    {//切行
      if(!change_line)           //如果检测到低电平，说明按键按下
      {delayms(5);               //延时去抖，一般10-20ms
        if(!change_line)         //再次确认按键是否按下，没有按下则退出
        {while(!change_line);    //如果确认按下按键等待按键释放，没有释放则一直等待	     
	 if(page_num!=1)write_6_8_char(0,line_num,' ');
	 if(line_num<7) line_num++;   //行序号加操作
         else           line_num=0;
         if(page_num!=1)write_6_8_char(0,line_num,'*');                       
        } 
      }
     /*对应参数加十*/
    if(!Add_10)                 //如果检测到低电平，说明按键按下
    {delayms(5);                //延时去抖，一般10-20ms 
     if(!Add_10)                //再次确认按键是否按下，没有按下则退出
     {while(!Add_10);           //如果确认按下按键等待按键释放，没有释放则一直等待
      change_value(page_num,line_num,10);    }}
    /*对应参数加一*/
    if(!Add_1)  
    {delayms(5); 
     if(!Add_1)
     {while(!Add_1);
      change_value(page_num,line_num,1); }}
    /*对应参数减一*/
    if(!Sub_1)
    {delayms(5);  
     if(!Sub_1) 
     {while(!Sub_1);
      change_value(page_num,line_num,-1);  }}
    /*对应参数减十*/
    if(!Sub_10)
    {delayms(5);
     if(!Sub_10)
     {while(!Sub_10);
      change_value(page_num,line_num,-10); }}	  
   }
}
//*************************************************************
//线性拟合   2015.5.9         
//start:数组起始下标 
//N: 拟合长度
//buff: 数组
//x:需要计算的输入
//*************************************************************
int16 Least_squares(int16 start,int16 N,int16*buff,int16 x)
{
  int16 i=0;
  float t1=0,t2=0,t3=0,t4=0;  
  for(i=start;i<start+N;++i)  
  {   
      t1+=i*i;   
      t2+=i;   
      t3+=i*(float)buff[i];   
      t4+=(float)buff[i];   
  }
  k=(N*t3-t4*t2)/(N*t1-t2*t2);  
  b=(t1*t4-t2*t3)/(N*t1-t2*t2); 
  return (int16)(k*x+b);
}
int16 abs(int16 x)
{
    if(x<0)  return - x;
    else     return x;
}
float abs_f(float x)
{
    if(x<0)  return - x;
    else     return x;
}
double abs_d(double x)
{
    if(x<0)  return - x;
    else     return x;
}

void  delay(void)
{
    delayms(100);
}
void  delayms(uint32  ms)
{
    uint32  i, j;
    for(i = 0; i < ms; i++){   for(j = bus_clk_khz; j > 0; j--){  asm("nop"); }}  
}
int   cyc(int a,int b){while(a<0)a+=b;while(a>=b)a-=b;return a;}//周期
float fade(float x,float y){if(x<-y)return -y;else if(x>y)return y;else return x;}        //限幅
float fadd(float x,float y){if(-y<x&&x<y)return 0;else if(x>0)return x-y;else return x+y;}//死区
int   ade(int x,int y){if(x<-y)return -y;else if(x>y)return y;else return x;}        //限幅



void waitiic0(){int i=1000;while(i&&(I2C0_S&I2C_S_IICIF_MASK)==0){i--;if(i==1)i2cdisable=1;}I2C0_S|= I2C_S_IICIF_MASK;}
void waitiic1(){int i=1000;while(i&&(I2C1_S&I2C_S_IICIF_MASK)==0){i--;if(i==1)i2cdisable=1;}I2C1_S|= I2C_S_IICIF_MASK;}
void waitiicfree0(){int i=3000;while(i&&I2C0_S&I2C_S_BUSY_MASK){i--;if(i==1)i2cdisable=1;}}
void waitiicfree1(){int i=3000;while(i&&I2C1_S&I2C_S_BUSY_MASK){i--;if(i==1)i2cdisable=1;}}

void iic_write(I2Cn iicn, u8 id, u8 register_address, u8 data)
{
    /* send data to slave */
  if(iicn)
  {
    waitiicfree1();//等待总线空闲
    I2C1_C1|=I2C_C1_TX_MASK;
    I2C1_C1|=I2C_C1_MST_MASK;
    I2C1_D=id<<1;
    waitiic1();
    I2C1_D=register_address;
    waitiic1();
    I2C1_D=data;
    waitiic1();
    I2C1_C1&=~I2C_C1_MST_MASK;
    I2C1_C1&=~I2C_C1_TX_MASK;
  }
  else
  {
    waitiicfree0();//等待总线空闲
    I2C0_C1|=I2C_C1_TX_MASK;
    I2C0_C1|=I2C_C1_MST_MASK;
    I2C0_D=id<<1;
    waitiic0();
    I2C0_D=register_address;
    waitiic0();
    I2C0_D=data;
    waitiic0();
    I2C0_C1&=~I2C_C1_MST_MASK;
    I2C0_C1&=~I2C_C1_TX_MASK;
  }
}
//MMA8451初始化
void mmainit(){iic_write(I2C1,0x1c,0x2a,0x01);iic_write(I2C1,0x1d,0x2a,0x01);}

void getmma()
{ mmapoint=(mmapoint+1)%Mmahnum;
  waitiicfree1();//等待总线空闲
  I2C1_C1|=I2C_C1_MST_MASK;
  I2C1_C1|=I2C_C1_TX_MASK;
  I2C1_D=0x38;
  waitiic1();
  I2C1_D=0x01;
  waitiic1();
  I2C1_C1|=I2C_C1_RSTA_MASK;
  I2C1_D=0x39;
  waitiic1();
  I2C1_C1 &= ~I2C_C1_TX_MASK;
  I2C1_C1 &= ~I2C_C1_TXAK_MASK;
  mmax[mmapoint]=I2C1_D;
  waitiic1();
  mmax[mmapoint]=I2C1_D;mmax[mmapoint]<<=8;
  waitiic1();
  mmax[mmapoint]|=I2C1_D;mmax[mmapoint]/=4;
  waitiic1();
  mmay[mmapoint]=I2C1_D;mmay[mmapoint]<<=8;
  waitiic1();
  mmay[mmapoint]|=I2C1_D;mmay[mmapoint]/=4;
  waitiic1();
  I2C1_C1 |= I2C_C1_TXAK_MASK;
  mmaz[mmapoint]=I2C1_D;mmaz[mmapoint]<<=8;
  waitiic1();
  I2C1_C1&=~I2C_C1_MST_MASK;
  mmaz[mmapoint]|=I2C1_D;mmaz[mmapoint]/=4;
  I2C1_C1&=~I2C_C1_TX_MASK;

  mmax[mmapoint]=mmax[mmapoint]-mma0x;
  mmay[mmapoint]=mmay[mmapoint]-mma0y;
  mmaz[mmapoint]=mmaz[mmapoint]-mma0z;

  mmaxn+=mmak*(mmax[mmapoint]-mmaxn);
  mmayn+=mmak*(mmay[mmapoint]-mmayn);
  mmazn+=mmak*(mmaz[mmapoint]-mmazn);
}

void getmma0()
{
  waitiicfree1();
  I2C1_C1|=I2C_C1_MST_MASK;
  I2C1_C1|=I2C_C1_TX_MASK;
  I2C1_D=0x38;
  waitiic1();
  I2C1_D=0x01;
  waitiic1();
  I2C1_C1|=I2C_C1_RSTA_MASK;
  I2C1_D=0x39;
  waitiic1();
  I2C1_C1 &= ~I2C_C1_TX_MASK;
  I2C1_C1 &= ~I2C_C1_TXAK_MASK;
  mmax[0]=I2C1_D;
  waitiic1();
  mmax[0]=I2C1_D;mmax[0]<<=8;
  waitiic1();
  mmax[0]|=I2C1_D;mmax[0]/=4;
  waitiic1();
  mmay[0]=I2C1_D;mmay[0]<<=8;
  waitiic1();
  mmay[0]|=I2C1_D;mmay[0]/=4;
  waitiic1();
  I2C1_C1 |= I2C_C1_TXAK_MASK;
  mmaz[0]=I2C1_D;mmaz[0]<<=8;
  waitiic1();
  I2C1_C1&=~I2C_C1_MST_MASK;
  mmaz[0]|=I2C1_D;mmaz[0]/=4;
  I2C1_C1&=~I2C_C1_TX_MASK;

}


void mpu6050init()
{iic_write(I2C1,0x68,0x6b,0x01); //内部pll输入
iic_write(I2C1,0x68,0x24,0x09);  //iic频率       500k?
//iic_write(I2C1,0x68,0x19,0x01);  // 陀螺仪采样频率  1K?
iic_write(I2C1,0x68,0x1a,0x21);  // 低通滤波     3<<3|1
iic_write(I2C1,0x68,0x1b,0x08);  //角速度范围     1<<3
//iic_write(I2C1,0x68,0x1c,0x00);//加速度范围
}

void getgy()
{ gypoint++;
  if(gypoint>=Gyhnum)gypoint=0;
  waitiicfree1();
  I2C1_C1|=I2C_C1_MST_MASK;
  I2C1_C1|=I2C_C1_TX_MASK;
  I2C1_D=0xd0;
  waitiic1();
  I2C1_D=0x43;
  waitiic1();
  I2C1_C1|=I2C_C1_RSTA_MASK;
  I2C1_D=0xd1;
  waitiic1();
  I2C1_C1 &= ~I2C_C1_TX_MASK;
  I2C1_C1 &= ~I2C_C1_TXAK_MASK;
  gyx[gypoint]=I2C1_D;
  waitiic1();
  gyx[gypoint]=I2C1_D;
  gyx[gypoint]<<=8;
  waitiic1();
  gyx[gypoint]|=I2C1_D;
  waitiic1();
  gyy[gypoint]=I2C1_D;
  gyy[gypoint]<<=8;
  waitiic1();
  gyy[gypoint]|=I2C1_D;
  waitiic1();
  I2C1_C1 |= I2C_C1_TXAK_MASK;
  gyz[gypoint]=I2C1_D;
  gyz[gypoint]<<=8;
  waitiic1();
  I2C1_C1&=~I2C_C1_MST_MASK;
  gyz[gypoint]|=I2C1_D;
  I2C1_C1&=~I2C_C1_TX_MASK;

  gyx[gypoint]=gyx[gypoint]-gy0x;
  gyy[gypoint]=gyy[gypoint]-gy0y;
  gyz[gypoint]=gyz[gypoint]-gy0z;
}
void getgy0()
{ gypoint++;
  if(gypoint>=Gyhnum)gypoint=0;
  waitiicfree1();
  I2C1_C1|=I2C_C1_MST_MASK;
  I2C1_C1|=I2C_C1_TX_MASK;
  I2C1_D=0xd0;
  waitiic1();
  I2C1_D=0x43;
  waitiic1();
  I2C1_C1|=I2C_C1_RSTA_MASK;
  I2C1_D=0xd1;
  waitiic1();
  I2C1_C1 &= ~I2C_C1_TX_MASK;
  I2C1_C1 &= ~I2C_C1_TXAK_MASK;
  gyx[gypoint]=I2C1_D;
  waitiic1();
  gyx[gypoint]=I2C1_D;gyx[gypoint]<<=8;
  waitiic1();
  gyx[gypoint]|=I2C1_D;
  waitiic1();
  gyy[gypoint]=I2C1_D;gyy[gypoint]<<=8;
  waitiic1();
  gyy[gypoint]|=I2C1_D;
  waitiic1();
  I2C1_C1 |= I2C_C1_TXAK_MASK;
  gyz[gypoint]=I2C1_D;gyz[gypoint]<<=8;
  waitiic1();
  I2C1_C1&=~I2C_C1_MST_MASK;
  gyz[gypoint]|=I2C1_D;
  I2C1_C1&=~I2C_C1_TX_MASK;

}
void mmagyfirst()
{
    int i;
    mma0x=mma0y=mma0z=0;
    gy0x=gy0y=gy0z=0;

    mmaxn=mmayn=0;
    mmazn=4096;
    double ax,ay,az,gx,gy,gz;
    ax=ay=az=gx=gy=gz=0;
    for(i=0;i<20;i++)
    {
      getgy0();
      getmma0();
      delayms(2);
    }
    for(i=0;i<100;i++)
    {
      getgy0();
      getmma0();
      ax+=mmax[0];ay+=mmay[0];az+=mmaz[0]-4096;
      gx+=gyx[gypoint];gy+=gyy[gypoint];gz+=gyz[gypoint];
      delayms(2);
    }
    mma0x=(int)(ax/100);  mma0y=(int)(ay/100);  mma0z=(int)(az/100);
    gy0x=(int)(gx/100);  gy0y=(int)(gy/100);  gy0z=(int)(gz/100);
  write_6_8_string(60,0,"mx:");
  write_6_8_number(84,0,mma0x);
  write_6_8_string(60,1,"my:");
  write_6_8_number(84,1,mma0y);
  write_6_8_string(60,2,"mz:");
  write_6_8_number(84,2,mma0z);
  write_6_8_string(60,3,"gx:");
  write_6_8_string(60,4,"gy:");
  write_6_8_string(60,5,"gz:");
  write_6_8_number(84,3,gy0x);
  write_6_8_number(84,4,gy0y);
  write_6_8_number(84,5,gy0z);
  delayms(100);
  for(i=0;i<Mmahnum;i++){mmax[i]=0;mmay[i]=0;mmaz[i]=4096;}
  for(i=0;i<Gyhnum;i++){gyx[i]=0;gyy[i]=0;gyz[i]=0;}
}
void body()
{
    getmma();
    getgy();
    Direction+=0.500*(double)gyz[gypoint]/32768;

  double /*gyk,*/gxk;

/*gyk=*/gxk=0.001;
    Gx_omiga=8.7266*gyx[gypoint];
    Gy_omiga=8.7266*gyy[gypoint];

   float g2;

    arm_sqrt_f32(mmayn*mmayn+mmazn*mmazn,&g2);
    Gx_arfa=32768*atan2(mmaxn,mmazn);
    Gy_arfa=32768*atan2(mmayn,mmazn);
    Gx_angle=Gx_angle+Gx_omiga*0.001+gxk*(Gx_arfa-Gx_angle-Gx_omiga*0.001);
    Angle_x=-Gx_angle;
    Gy_angle=Gy_angle+Gy_omiga*0.001+0.002*(0-Gy_angle-Gy_omiga*0.001);
    Angle_y=-Gy_angle;
    carturnning+=gyz[gypoint];
}