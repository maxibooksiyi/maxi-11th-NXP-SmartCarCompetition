#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "LDC1000.H"
#include "MK60_spi.h"


#define uchar uint8 
#define NN  10

uint8 orgVal1[12]={0};
uint8 orgVal2[12]={0};
uint8 orgVal3[12]={0};

uint8 RPMAX1 =0x07; 
uint8 RPMIN1 =0x2f;
uint8 FREQ1 =0xA9;
uint8 LDCCONFIG1 =0x13;

uint8 RPMAX2 =0x07; 
uint8 RPMIN2 =0x2f;
uint8 FREQ2 =0xA9;
uint8 LDCCONFIG2 =0x13;

uint8 RPMAX3 =0x07; 
uint8 RPMIN3 =0x2f;
uint8 FREQ3 =0xA9;
uint8 LDCCONFIG3 =0x13;


uint8 rpi_max=20;
uint8 proximtyData1[2]={0};
uint8 proximtyData2[2]={0};
uint8 proximtyData3[2]={0};

unsigned long proximtyDataTEMP1=0,proximtyDataMAX1,proximtyDataMIN1,proximtyDataSUM1,proximtyDataAVE1,proximtyDataAVE_LAS1;
unsigned long proximtyDataTEMP2=0,proximtyDataMAX2,proximtyDataMIN2,proximtyDataSUM2,proximtyDataAVE2,proximtyDataAVE_LAS2;
unsigned long proximtyDataTEMP3=0,proximtyDataMAX3,proximtyDataMIN3,proximtyDataSUM3,proximtyDataAVE3,proximtyDataAVE_LAS3;


int LDC_val1=0,LDC_val2=0,LDC_val1_pre=0,LDC_val2_pre=0,LDC_val3=0;

unsigned long value_buf1[NN],new_value_buf1[NN];
unsigned long value_buf2[NN],new_value_buf2[NN];
unsigned long value_buf3[NN],new_value_buf3[NN];

void FLOAT_delay_us(int ms)//为防止time_delay_ms();与lpt冲突编写的延时
{
  int j1,k_1;int i1;
  i1=ms;
  for(j1=0;j1<i1;j1++)   
    for(k_1=0;k_1<8;k_1++);
}

 /*  @brief      LDC1000电轨传感器模块初始化
 *  @param      
 *  @param      
 *  @param      
 *  @since      
 *  @note       包含SPI初始化*/

/**************
通道1采集信号
**************/
void FLOAT_LDC_init1()
{
      while(orgVal1[1]!=RPMAX1||orgVal1[2]!=RPMIN1||orgVal1[3]!=FREQ1)//一旦在此循环说明初始化不成功
      {  
         FLOAT_SPI_init1();   
         FLOAT_delay_us(3000);
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_RPMAX, RPMAX1);
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_RPMIN, RPMIN1);//0x14
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_SENSORFREQ,  FREQ1);  //谐振频率计算方法见《浮点科技电轨传感器调试手册》
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_LDCCONFIG,   LDCCONFIG1);  /********/
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_CLKCONFIG,   0x00);  //L配置LDC1000的输出速率
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_THRESHILSB,  0x50);  /********/
	 FLOAT_Singal_SPI_Write1(LDC1000_CMD_THRESHIMSB,  0x14);
	 FLOAT_Singal_SPI_Write1(LDC1000_CMD_THRESLOLSB,  0xC0);
	 FLOAT_Singal_SPI_Write1(LDC1000_CMD_THRESLOMSB,  0x12);
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_INTCONFIG,   0x02);
         FLOAT_Singal_SPI_Write1(LDC1000_CMD_PWRCONFIG,   0x01);
         FLOAT_SPI_Read_Buf1(LDC1000_CMD_REVID,&orgVal1[0],12);//orgVal[]对应上面写入的值说明初始化正常
       
      }
} 

int ldc_read_avr1()
{

    char rpi=0;  //取rpi次平均值    
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf1(LDC1000_CMD_PROXLSB,&proximtyData1[0],2);  
      proximtyDataTEMP1 = ((unsigned char)proximtyData1[1]<<8) + proximtyData1 [0]; 
      proximtyDataSUM1 += proximtyDataTEMP1;
      if (proximtyDataTEMP1 < proximtyDataMIN1)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN1 = proximtyDataTEMP1;
      if (proximtyDataTEMP1 > proximtyDataMAX1)
        proximtyDataMAX1 = proximtyDataTEMP1;
    }
     proximtyDataAVE1 = proximtyDataSUM1 /rpi_max;
     proximtyDataSUM1=0;
     proximtyDataAVE_LAS1=proximtyDataAVE1;
  
    return   proximtyDataAVE1; 

}
long int filter1()//滤波函数，需要认真改写，否则数据跳动，会造成互感的假象
{
     int16  i,j,k,temp; 
     int16  ad_valu[5],ad_valu_avr,ad_sum; 
     int16  AD_sum,AD_V[7],AD_valu_avr;
     for(i=0;i<5;i++)//每个采5次
     {   
         ad_valu[i]=ldc_read_avr1()/10;     // ADC1 通道     PTB1       
               
         
     }
     //////////////////////冒泡排序////////////////////////////////
     
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
            
              if(ad_valu[k] > ad_valu[k+1])  //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[k+1];
                 ad_valu[k+1] = ad_valu[k];//从小到大排序
                 ad_valu[k] = temp;
              } 
           }
        }
    
          
        ad_sum = ad_valu[1] + ad_valu[2] + ad_valu[3];       
        ad_valu_avr = ad_sum / 3;
        
     ////////////////////////递推平均滤波/////////////////////////////      
     for(i = 0;i < NM-1;i ++)
     {      
         AD_V[i] = AD_V[i + 1];

     }

         AD_V[NM-1]=ad_valu_avr;
        
     for(i = 0;i < NM;i ++)
     {      
         AD_sum += AD_V[i];
       
     }   
     
            
         AD_valu_avr = (int16)(AD_sum/NM);         
         AD_sum = 0; 
      
     return AD_valu_avr;

}

void FLOAT_SPI_init1()
{  
       
         gpio_init (PTE1, GPI,1);//MISO
         gpio_init (PTE3 ,GPO,1);//MOSI
         gpio_init (PTE5, GPO,1);// CSN
         gpio_init (PTE7, GPO,0);//SCK
         
         CSN_H1;
         SCK_L1;
         MOSI_H1;
         
  
}
/****************************************************************************************************
* Function Name: uchar FLOAT_SPI_RW(uchar wdata)
* Description  : read and write of SPI.
* Arguments    : wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_SPI_RW1(uchar rwdata)
{  
    
	uchar spi_rw_i=0;	
        uchar temp=0;
        for(spi_rw_i=0;spi_rw_i<8;spi_rw_i++)   	// output 8-bit
   	{
   	        /*** prepare the write data of read before the coming of rising up******/
	          if(rwdata & 0x80)
                    MOSI_H1;
   		  else 
                    MOSI_L1;
   		  rwdata<<=1;           		// shift next bit to MSB
                  temp<<=1;
		SCK_H1;             //Set SCK high    Rising up 
               
   		if(MISO1) 
                  temp|=1;
   		SCK_L1;            //set  SCK low     Falling down
                
   	}
    return(temp);           		  		// return read byte
    
 
}
/****************************************************************************************************
* Function Name: uchar FLOAT_Singal_SPI(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_Singal_SPI_Read1(uchar reg)
{
	uchar rdata;
	
	CSN_L1;                        // CSN low, initialize SPI communication...
       
        FLOAT_delay_us(2);
         
         reg=reg|0x80;         //read com
	FLOAT_SPI_RW1(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         
	rdata = FLOAT_SPI_RW1(NULL);    // then read registervalue
       
        FLOAT_delay_us(1700);
	CSN_H1;                 // CSN high, terminate SPI communication
	
	return rdata;         // return register value
}
/****************************************************************************************************
* Function Name: void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
void FLOAT_Singal_SPI_Write1(uchar reg,uchar wdata)
{
	
	CSN_L1;                // CSN low, initialize SPI communication...
      
        FLOAT_delay_us(2);//2us
        reg=reg&~0x80;
	FLOAT_SPI_RW1(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
        
	FLOAT_SPI_RW1(wdata);    // ..then read registervalue
        FLOAT_delay_us(1700);//875us
	CSN_H1;              // CSN high, terminate SPI communication
       
	
}

/****************************************************************************************************
* Function Name: void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len)
* Description  : read muche registers
* Arguments    : reg,len
* Return Value : *pBuf
****************************************************************************************************/
void FLOAT_SPI_Read_Buf1(uchar reg, uchar *pBuf, uchar len)
{
	uchar spi_rw_i;
	
	CSN_L1;                   		// Set CSN low, init SPI tranaction
       
        reg=reg|0x80;                            //read
	FLOAT_SPI_RW1(reg);       		// Select register to write to and read status uchar
	
	for(spi_rw_i=0;spi_rw_i<len;spi_rw_i++)
        {  
	pBuf[spi_rw_i] = FLOAT_SPI_RW1(NULL);    
	 }
	CSN_H1;     
       

}

/**************
通道2采集信号
**************/
void FLOAT_LDC_init2()
{
      while(orgVal2[1]!=RPMAX2||orgVal2[2]!=RPMIN2||orgVal2[3]!=FREQ2)//一旦在此循环说明初始化不成功
      {  
         FLOAT_SPI_init2();   
         FLOAT_delay_us(3000);
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_RPMAX, RPMAX2);
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_RPMIN, RPMIN2);//0x14
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_SENSORFREQ, FREQ2);  //谐振频率计算方法见《浮点科技电轨传感器调试手册》
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_LDCCONFIG,   LDCCONFIG2);  /********/
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_CLKCONFIG,   0x00);  //L配置LDC1000的输出速率
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_THRESHILSB,  0x50);  /********/
	 FLOAT_Singal_SPI_Write2(LDC1000_CMD_THRESHIMSB,  0x14);
	 FLOAT_Singal_SPI_Write2(LDC1000_CMD_THRESLOLSB,  0xC0);
	 FLOAT_Singal_SPI_Write2(LDC1000_CMD_THRESLOMSB,  0x12);
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_INTCONFIG,   0x02);
         FLOAT_Singal_SPI_Write2(LDC1000_CMD_PWRCONFIG,   0x01);
         FLOAT_SPI_Read_Buf2(LDC1000_CMD_REVID,&orgVal2[0],12);//orgVal[]对应上面写入的值说明初始化正常
       
      }
} 

int ldc_read_avr2()
{

    char rpi=0;  //取rpi次平均值    
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf2(LDC1000_CMD_PROXLSB,&proximtyData2[0],2);  
      proximtyDataTEMP2 = ((unsigned char)proximtyData2[1]<<8) + proximtyData2 [0]; 
      proximtyDataSUM2 += proximtyDataTEMP2;
      if (proximtyDataTEMP2 < proximtyDataMIN2)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN2 = proximtyDataTEMP2;
      if (proximtyDataTEMP2 > proximtyDataMAX2)
        proximtyDataMAX2 = proximtyDataTEMP2;
    }
     proximtyDataAVE2 = proximtyDataSUM2 /rpi_max;
     proximtyDataSUM2=0;
     proximtyDataAVE_LAS2=proximtyDataAVE2;
  
    return   proximtyDataAVE2; 

}
long int filter2()
{
    int16  i,j,k,temp; 
     int16  ad_valu[5],ad_valu_avr,ad_sum; 
     int16  AD_sum,AD_V[7],AD_valu_avr;
     for(i=0;i<5;i++)//每个采5次
     {   
         ad_valu[i]=ldc_read_avr2()/10;     // ADC1 通道     PTB1       
               
         
     }
     //////////////////////冒泡排序////////////////////////////////
     
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
            
              if(ad_valu[k] > ad_valu[k+1])  //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[k+1];
                 ad_valu[k+1] = ad_valu[k];//从小到大排序
                 ad_valu[k] = temp;
              } 
           }
        }
    
          
        ad_sum = ad_valu[1] + ad_valu[2] + ad_valu[3];       
        ad_valu_avr = ad_sum / 3;
        
     ////////////////////////递推平均滤波/////////////////////////////      
     for(i = 0;i < NM-1;i ++)
     {      
         AD_V[i] = AD_V[i + 1];

     }

         AD_V[NM-1]=ad_valu_avr;
        
     for(i = 0;i < NM;i ++)
     {      
         AD_sum += AD_V[i];
       
     }   
     
            
         AD_valu_avr = (int16)(AD_sum/NM);         
         AD_sum = 0; 
      
     return AD_valu_avr;

}

void FLOAT_SPI_init2()
{  
       
         gpio_init (PTE8, GPI,1);//MISO
         gpio_init (PTE9 ,GPO,1);//MOSI
         gpio_init (PTE10, GPO,1);// CSN
         gpio_init (PTE11, GPO,0);//SCK
         
         CSN_H2;
         SCK_L2;
         MOSI_H2;
         
  
}
/****************************************************************************************************
* Function Name: uchar FLOAT_SPI_RW(uchar wdata)
* Description  : read and write of SPI.
* Arguments    : wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_SPI_RW2(uchar rwdata)
{  
    
	uchar spi_rw_i=0;	
        uchar temp=0;
        for(spi_rw_i=0;spi_rw_i<8;spi_rw_i++)   	// output 8-bit
   	{
   	        /*** prepare the write data of read before the coming of rising up******/
	          if(rwdata & 0x80)
                    MOSI_H2;
   		  else 
                    MOSI_L2;
   		  rwdata<<=1;           		// shift next bit to MSB
                  temp<<=1;
		SCK_H2;             //Set SCK high    Rising up 
               
   		if(MISO2) 
                  temp|=1;
   		SCK_L2;            //set  SCK low     Falling down
                
   	}
    return(temp);           		  		// return read byte
    
 
}
/****************************************************************************************************
* Function Name: uchar FLOAT_Singal_SPI(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_Singal_SPI_Read2(uchar reg)
{
	uchar rdata;
	
	CSN_L2;                        // CSN low, initialize SPI communication...
       
        FLOAT_delay_us(2);
         
         reg=reg|0x80;         //read com
	FLOAT_SPI_RW2(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         
	rdata = FLOAT_SPI_RW2(NULL);    // then read registervalue
       
        FLOAT_delay_us(1700);
	CSN_H2;                 // CSN high, terminate SPI communication
	
	return rdata;         // return register value
}
/****************************************************************************************************
* Function Name: void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
void FLOAT_Singal_SPI_Write2(uchar reg,uchar wdata)
{
	
	CSN_L2;                // CSN low, initialize SPI communication...
      
        FLOAT_delay_us(2);//2us
        reg=reg&~0x80;
	FLOAT_SPI_RW2(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
        
	FLOAT_SPI_RW2(wdata);    // ..then read registervalue
        FLOAT_delay_us(1700);//875us
	CSN_H2;              // CSN high, terminate SPI communication
       
	
}

/****************************************************************************************************
* Function Name: void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len)
* Description  : read muche registers
* Arguments    : reg,len
* Return Value : *pBuf
****************************************************************************************************/
void FLOAT_SPI_Read_Buf2(uchar reg, uchar *pBuf, uchar len)
{
	uchar spi_rw_i;
	
	CSN_L2;                   		// Set CSN low, init SPI tranaction
       
        reg=reg|0x80;                            //read
	FLOAT_SPI_RW2(reg);       		// Select register to write to and read status uchar
	
	for(spi_rw_i=0;spi_rw_i<len;spi_rw_i++)
        {  
	pBuf[spi_rw_i] = FLOAT_SPI_RW2(NULL);    
	 }
	CSN_H2;     
       

}



/**************
通道3采集信号
**************/
void FLOAT_LDC_init3()
{
      while(orgVal3[1]!=RPMAX3||orgVal3[2]!=RPMIN3||orgVal3[3]!=FREQ3)//一旦在此循环说明初始化不成功
      {  
         FLOAT_SPI_init3();   
         FLOAT_delay_us(3000);
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_RPMAX, RPMAX3);
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_RPMIN, RPMIN3);//0x14
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_SENSORFREQ, FREQ3);  //谐振频率计算方法见《浮点科技电轨传感器调试手册》
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_LDCCONFIG,   LDCCONFIG3);  /********/
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_CLKCONFIG,   0x00);  //L配置LDC1000的输出速率
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_THRESHILSB,  0x50);  /********/
	 FLOAT_Singal_SPI_Write3(LDC1000_CMD_THRESHIMSB,  0x14);
	 FLOAT_Singal_SPI_Write3(LDC1000_CMD_THRESLOLSB,  0xC0);
	 FLOAT_Singal_SPI_Write3(LDC1000_CMD_THRESLOMSB,  0x12);
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_INTCONFIG,   0x02);
         FLOAT_Singal_SPI_Write3(LDC1000_CMD_PWRCONFIG,   0x01);
         FLOAT_SPI_Read_Buf3(LDC1000_CMD_REVID,&orgVal3[0],12);//orgVal[]对应上面写入的值说明初始化正常
       
      }
} 

int ldc_read_avr3()
{

    char rpi=0;  //取rpi次平均值    
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf3(LDC1000_CMD_PROXLSB,&proximtyData3[0],2);  
      proximtyDataTEMP3 = ((unsigned char)proximtyData3[1]<<8) + proximtyData3 [0]; 
      proximtyDataSUM3 += proximtyDataTEMP3;
      if (proximtyDataTEMP3 < proximtyDataMIN3)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN3 = proximtyDataTEMP3;
      if (proximtyDataTEMP3 > proximtyDataMAX3)
        proximtyDataMAX3 = proximtyDataTEMP3;
    }
     proximtyDataAVE3 = proximtyDataSUM3 /rpi_max;
     proximtyDataSUM3=0;
     proximtyDataAVE_LAS3=proximtyDataAVE3;
  
    return   proximtyDataAVE3; 

}
long int filter3()
{
    int16  i,j,k,temp; 
     int16  ad_valu[5],ad_valu_avr,ad_sum; 
     int16  AD_sum,AD_V[7],AD_valu_avr;
     for(i=0;i<5;i++)//每个采5次
     {   
         ad_valu[i]=ldc_read_avr3()/10;     // ADC1 通道     PTB1       
               
         
     }
     //////////////////////冒泡排序////////////////////////////////
     
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
            
              if(ad_valu[k] > ad_valu[k+1])  //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[k+1];
                 ad_valu[k+1] = ad_valu[k];//从小到大排序
                 ad_valu[k] = temp;
              } 
           }
        }
    
          
        ad_sum = ad_valu[1] + ad_valu[2] + ad_valu[3];       
        ad_valu_avr = ad_sum / 3;
        
     ////////////////////////递推平均滤波/////////////////////////////      
     for(i = 0;i < NM-1;i ++)
     {      
         AD_V[i] = AD_V[i + 1];

     }

         AD_V[NM-1]=ad_valu_avr;
        
     for(i = 0;i < NM;i ++)
     {      
         AD_sum += AD_V[i];
       
     }   
     
            
         AD_valu_avr = (int16)(AD_sum/NM);         
         AD_sum = 0; 
      
     return AD_valu_avr;

}

void FLOAT_SPI_init3()
{  
       
         gpio_init (PTD6, GPI,1);//MISO
         gpio_init (PTD4 ,GPO,1);//MOSI
         gpio_init (PTD2, GPO,1);// CSN
         gpio_init (PTD0, GPO,0);//SCK
         
         CSN_H3;
         SCK_L3;
         MOSI_H3;
         
  
}
/****************************************************************************************************
* Function Name: uchar FLOAT_SPI_RW(uchar wdata)
* Description  : read and write of SPI.
* Arguments    : wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_SPI_RW3(uchar rwdata)
{  
    
	uchar spi_rw_i=0;	
        uchar temp=0;
        for(spi_rw_i=0;spi_rw_i<8;spi_rw_i++)   	// output 8-bit
   	{
   	        /*** prepare the write data of read before the coming of rising up******/
	          if(rwdata & 0x80)
                    MOSI_H3;
   		  else 
                    MOSI_L3;
   		  rwdata<<=1;           		// shift next bit to MSB
                  temp<<=1;
		SCK_H3;             //Set SCK high    Rising up 
               
   		if(MISO3) 
                  temp|=1;
   		SCK_L3;            //set  SCK low     Falling down
                
   	}
    return(temp);           		  		// return read byte
    
 
}
/****************************************************************************************************
* Function Name: uchar FLOAT_Singal_SPI(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_Singal_SPI_Read3(uchar reg)
{
	uchar rdata;
	
	CSN_L3;                        // CSN low, initialize SPI communication...
       
        FLOAT_delay_us(2);
         
         reg=reg|0x80;         //read com
	FLOAT_SPI_RW3(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         
	rdata = FLOAT_SPI_RW3(NULL);    // then read registervalue
       
        FLOAT_delay_us(1700);
	CSN_H3;                 // CSN high, terminate SPI communication
	
	return rdata;         // return register value
}
/****************************************************************************************************
* Function Name: void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
void FLOAT_Singal_SPI_Write3(uchar reg,uchar wdata)
{
	
	CSN_L3;                // CSN low, initialize SPI communication...
      
        FLOAT_delay_us(2);//2us
        reg=reg&~0x80;
	FLOAT_SPI_RW3(reg);            // Select register to read from..
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
         asm("nop");
        
	FLOAT_SPI_RW3(wdata);    // ..then read registervalue
        FLOAT_delay_us(1700);//875us
	CSN_H3;              // CSN high, terminate SPI communication
       
	
}

/****************************************************************************************************
* Function Name: void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len)
* Description  : read muche registers
* Arguments    : reg,len
* Return Value : *pBuf
****************************************************************************************************/
void FLOAT_SPI_Read_Buf3(uchar reg, uchar *pBuf, uchar len)
{
	uchar spi_rw_i;
	
	CSN_L3;                   		// Set CSN low, init SPI tranaction
       
        reg=reg|0x80;                            //read
	FLOAT_SPI_RW3(reg);       		// Select register to write to and read status uchar
	
	for(spi_rw_i=0;spi_rw_i<len;spi_rw_i++)
        {  
	pBuf[spi_rw_i] = FLOAT_SPI_RW3(NULL);    
	 }
	CSN_H3;     
       

}