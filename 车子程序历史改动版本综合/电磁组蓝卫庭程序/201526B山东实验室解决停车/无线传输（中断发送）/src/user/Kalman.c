#include "include.h"
#define  GYRO_Sensitivity  16.4  //����ѡ����������ȷ����MPU6050��GYRO
                                 //�ĸ���������250,500,1000,2000��Ӧ��GYRO_Sensitivity�ֱ�Ϊ131,65.5,32.8,16.4.  

int32   Accel_X,Accel_Y,Accel_Z;   //
int32   Gyro_X,Gyro_Y,Gyro_Z;      //
int16   Gyro_X_Offest=0,Gyro_Y_Offest=0,Gyro_Z_Offest=0;//
int16   Angle_AX,Angle_AY,Angle_AZ;//
int32   Gyro_XY,Gyro_YX,Gyro_ZZ;   //
int16   Angle_X,Angle_Y,Angle_Z;   //
int32   AngleSigma_X,AngleSigma_Y,AngleSigma_Z;
float   arc_r;
int16   arc_count;
int32   Gyro_Zz[3];

int16   jiaodu_y=0;   //��׼ֵ����


void Offest_Init(void)
{
     uint16 i;  
     if(K4)
     {
        LCD_CLS(); 
        LCD_Print(25,2,"Collecting"); 
        LCD_Print(28,4,"offest..."); 
        Gyro_X_Offest=0;Gyro_Y_Offest=0;Gyro_Z_Offest=0;  //��ƫ��0
     
        for(i=0;i<500;i++) //���Ư���벹��
        { 
             Gyro_X = GetData(GYRO_XOUT_H);	   
             Gyro_Y = GetData(GYRO_YOUT_H);	
             Gyro_Z = GetData(GYRO_ZOUT_H);
             delayms(2);           //��ʱ	
             Gyro_X_Offest += Gyro_X;	
             Gyro_Y_Offest += Gyro_Y;	
             Gyro_Z_Offest += Gyro_Z;  
        } 
        Gyro_X_Offest = Gyro_X_Offest/500;	
        Gyro_Y_Offest = Gyro_Y_Offest/500;	
        Gyro_Z_Offest = Gyro_Z_Offest/500; 
     
        AngleSigma_X=0;AngleSigma_Y=0;AngleSigma_Z=0;  //���ֹ�0
     
        flash_erase_sector(SECTOR_MPU);       //��������	
        flash_write(SECTOR_MPU,0,Gyro_X_Offest);  //д����
        flash_write(SECTOR_MPU,4,Gyro_Y_Offest);  //д����
        flash_write(SECTOR_MPU,8,Gyro_Z_Offest);  //д����
     }
     else 
     { 
          Gyro_X_Offest = flash_read(SECTOR_MPU,0,uint16);		
          Gyro_Y_Offest = flash_read(SECTOR_MPU,4,uint16);		
          Gyro_Z_Offest = flash_read(SECTOR_MPU,8,uint16);	
     } 
     
     while(K4);	
     LCD_CLS();      //����            
}

void CarVoltageGet(void)
{
      int32   lnDeltaValue;
      
      Accel_X  = GetData(ACCEL_XOUT_H);     //��ȡX����ٶ�
      Accel_Y  = GetData(ACCEL_YOUT_H);
      Accel_Z  = GetData(ACCEL_ZOUT_H);
      Gyro_X = GetData(GYRO_XOUT_H);	   //��ȡX����ٶ�
      Gyro_Y = GetData(GYRO_YOUT_H);	
      Gyro_Z = GetData(GYRO_ZOUT_H);
        
       /*******************X����̬����************************/   
       
        if(Accel_Y > 32767)     Accel_Y = Accel_Y - 65535;  
        if(Accel_Y <= -18000)   Accel_Y = -18000;          //���Ƽ��ٶȷ���
        Angle_AY = (int32)(Accel_Y * ACCE_Y_RATIO / 100);   //�Ƕȹ�һ��
   
        if(Gyro_X > 32767)      Gyro_X = Gyro_X - 65535;
	Gyro_XY = (int32)(Gyro_X *GYRO_X_RATIO / 100 );     //���ٶȹ�һ��
      
        Angle_X = (int16)(AngleSigma_X / 20);                      //X��Ƕȼ���
        lnDeltaValue = Angle_AY - Angle_X;                          //�Ƕ�У��
        lnDeltaValue = (int32)(lnDeltaValue * ACCE_XY_RATIO / 100); //�Ƕȷ���
        AngleSigma_X += (Gyro_XY + lnDeltaValue);	            //���ٶȻ��� 
        
       /*******************Y����̬����************************/               
        if(Accel_X > 32767)     Accel_X = Accel_X - 65535;  
        if(Accel_X <= -18000)   Accel_X = -18000;           //���Ƽ��ٶȷ���
        Angle_AX = (int32)(Accel_X * ACCE_X_RATIO / 100);   //�Ƕȹ�һ��
     
        if(Gyro_Y > 32767)      Gyro_Y = Gyro_Y - 65535;
        Gyro_Y = - Gyro_Y;       //��ת ʹ������ٶ�һ��
        Gyro_YX = (int32)(Gyro_Y *GYRO_Y_RATIO / 100 );    //���ٶȹ�һ��
      
        Angle_Y = (int16)(AngleSigma_Y / 20);                    //Y��Ƕȼ���
        lnDeltaValue = Angle_AX - Angle_Y;                          //�Ƕ�У��
        lnDeltaValue = (int32)(lnDeltaValue * ACCE_YX_RATIO / 100); //�Ƕȷ���
        AngleSigma_Y += (Gyro_YX + lnDeltaValue);	            //���ٶȻ��� 
         
        //  LCD_Show_Number(0,3,Angle_X);
        //  LCD_Show_Number(0,4,Angle_Y);
                  
}

void jiaodu_get(void)
{
  uint16 i;
  int32   jiaodu=0;
  if(K5)
  {
     LCD_CLS(); 
     LCD_Print(25,2,"Collecting"); 
     LCD_Print(28,4,"y_jiaodu...");
  
     for(i=0;i<200;i++)
     {
       CarVoltageGet();
       jiaodu=jiaodu+Angle_Y;    
     }
     
     jiaodu_y=jiaodu/200;
     
      flash_erase_sector(SECTOR_Y);       //��������	
      flash_write(SECTOR_Y,0,jiaodu_y);   //д����
     
  }
  
  else
    
     jiaodu_y=flash_read(SECTOR_Y,0,uint16);  
     while(K5);
     LCD_CLS();      //����
}




