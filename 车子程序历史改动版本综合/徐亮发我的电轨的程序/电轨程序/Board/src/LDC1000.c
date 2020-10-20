#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "LDC1000.H"
#include "MK60_spi.h"


#define uchar uint8 
#define NN  20

uint8 orgVal[12]={0};

uint8 RPMAX =0x07; 
uint8 RPMIN =0x2f;  
uint8 rpi_max=10;
uint8 proximtyData[2]={0};
unsigned long proximtyDataTEMP=0,proximtyDataMAX,proximtyDataMIN,proximtyDataSUM,proximtyDataAVE,proximtyDataAVE_LAS;

int LDC_val1=0;
int LDC_val2=0;
int LDC_val=0;

unsigned long value_buf[NN],new_value_buf[NN];

void FLOAT_delay_us(int ms)//为防止time_delay_ms();与lpt冲突编写的延时
{
  int j1,k_1;int i1;
  i1=ms;
  for(j1=0;j1<i1;j1++)   
    for(k_1=0;k_1<8;k_1++);
}
/*!
 *  @brief      初始化LDC电轨传感器模块
 *  @param      
 *  @param      
 *  @param      
 *  @since      
 *  @note       包含SPI初始化
 *  Sample usage:       
 */


/******************************************
*通道1采集信号
******************************************/
void FLOAT_LDC_init_1()
{
 
         FLOAT_SPI_init_1();   
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_RPMAX, RPMAX);
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_RPMIN, RPMIN);//0x14
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_SENSORFREQ,  0xA9);  //谐振频率计算方法见《浮点科技电轨传感器调试手册》
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_LDCCONFIG,   0x17);  //0x1B
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_CLKCONFIG,   0x00);  //0x01        
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_INTCONFIG,   0x02);
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_PWRCONFIG,   0x01);
         FLOAT_Singal_SPI_Write_1(LDC1000_CMD_THRESHILSB,  0x50);
	 FLOAT_Singal_SPI_Write_1(LDC1000_CMD_THRESHIMSB,  0x14);
	 FLOAT_Singal_SPI_Write_1(LDC1000_CMD_THRESLOLSB,  0xC0);
	 FLOAT_Singal_SPI_Write_1(LDC1000_CMD_THRESLOMSB,  0x12);
         FLOAT_SPI_Read_Buf_1(LDC1000_CMD_REVID,&orgVal[0],12);//orgVal[]对应上面写入的值说明初始化正常
       
  
} 

int ldc_read_avr_1()
{

    char rpi=0;  //取rpi次平均值    
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf_1(LDC1000_CMD_PROXLSB,&proximtyData[0],2);  
      proximtyDataTEMP = ((unsigned char)proximtyData[1]<<8) + proximtyData [0]; 
      proximtyDataSUM += proximtyDataTEMP;
      if (proximtyDataTEMP < proximtyDataMIN)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN = proximtyDataTEMP;
      if (proximtyDataTEMP > proximtyDataMAX)
        proximtyDataMAX = proximtyDataTEMP;
    }
     proximtyDataAVE = proximtyDataSUM /rpi_max;
     proximtyDataSUM=0;
     proximtyDataAVE_LAS=proximtyDataAVE;
  
    return   proximtyDataAVE; 

}
long int filter_1()
{
   char count,i,j,count1;
   char count2=0;
 
   long int temp;
   long int sum=0;
   for(count=0;count<NN;count++)
   {
      value_buf[count] = ldc_read_avr_1();
   }
   
   for(count1=0;count1<NN;count1++)
   {  
   if(value_buf[count1]<32768)
   {
   new_value_buf[count2]=value_buf[count1];
   count2++;
   }  
   }
   
   
   for (j=0;j<count2-1;j++)
   {
      for (i=0;i<count2-j;i++)
      {
        if ( new_value_buf[i]>new_value_buf[i+1] )
         {
            temp = new_value_buf[i];
            new_value_buf[i] = new_value_buf[i+1];
            new_value_buf[i+1] = temp;
         }
      }
   }

   for(count=1;count<count2-1;count++)
   {
      sum += new_value_buf[count];
   }
  
   sum=sum/10;  
   return (long int)(sum/(count2-2));


}

void FLOAT_SPI_init_1()
{  
       
         gpio_init (PTB0, GPI,1);//MISO
         gpio_init (PTB1 ,GPO,1);//MOSI
         gpio_init (PTB2, GPO,1);// CSN
         gpio_init (PTB3, GPO,0);//SCK
         
         CSN_H_1;
         SCK_L_1;
         MOSI_H_1;
         
  
}
/****************************************************************************************************
* Function Name: uchar FLOAT_SPI_RW(uchar wdata)
* Description  : read and write of SPI.
* Arguments    : wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_SPI_RW_1(uchar rwdata)
{  
    
	uchar spi_rw_i=0;	
        uchar temp=0;
        for(spi_rw_i=0;spi_rw_i<8;spi_rw_i++)   	// output 8-bit
   	{
   	        /*** prepare the write data of read before the coming of rising up******/
	          if(rwdata & 0x80)
                    MOSI_H_1;
   		  else 
                    MOSI_L_1;
   		  rwdata<<=1;           		// shift next bit to MSB
                  temp<<=1;
		SCK_L_1;             //Set SCK high    Rising up 
               
   		if(MISO_1) 
                  temp|=1;
   		SCK_H_1;            //set  SCK low     Falling down
                
   	}
    return(temp);           		  		// return read byte
    
 
}
/****************************************************************************************************
* Function Name: uchar FLOAT_Singal_SPI(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_Singal_SPI_Read_1(uchar reg)
{
	uchar rdata;
	
	CSN_L_1;                // CSN low, initialize SPI communication...
       
        FLOAT_delay_us(2);
         
         reg=reg|0x80;//read
	FLOAT_SPI_RW_1(reg);            // Select register to read from..
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
         
	rdata = FLOAT_SPI_RW_1(NULL);    // ..then read registervalue
       
        FLOAT_delay_us(1700);
	CSN_H_1;                // CSN high, terminate SPI communication
	
	return rdata;        // return register value
}
/****************************************************************************************************
* Function Name: void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
void FLOAT_Singal_SPI_Write_1(uchar reg,uchar wdata)
{
	
	CSN_L_1;                // CSN low, initialize SPI communication...
      
        FLOAT_delay_us(2);//2us
        reg=reg&~0x80;
	FLOAT_SPI_RW_1(reg);            // Select register to read from..
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
        
	FLOAT_SPI_RW_1(wdata);    // ..then read registervalue
        FLOAT_delay_us(1700);//875us
	CSN_H_1;              // CSN high, terminate SPI communication
       
	
}

/****************************************************************************************************
* Function Name: void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len)
* Description  : read muche registers
* Arguments    : reg,len
* Return Value : *pBuf
****************************************************************************************************/

void FLOAT_SPI_Read_Buf_1(uchar reg, uchar *pBuf, uchar len)
{
	uchar spi_rw_i;
	
	CSN_L_1;                   		// Set CSN low, init SPI tranaction
       
        reg=reg|0x80;//read
	FLOAT_SPI_RW_1(reg);       		// Select register to write to and read status uchar
	
	for(spi_rw_i=0;spi_rw_i<len;spi_rw_i++)
        {  
	pBuf[spi_rw_i] = FLOAT_SPI_RW_1(NULL);    // 
	 }
	CSN_H_1;     
       

}




/**************************************
*通道2采集信号
**************************************/

void FLOAT_LDC_init_2()
{
 
         FLOAT_SPI_init_2();   
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_RPMAX, RPMAX);
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_RPMIN, RPMIN);//0x14
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_SENSORFREQ,  0xA9);  //谐振频率计算方法见《浮点科技电轨传感器调试手册》
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_LDCCONFIG,   0x17);  //0x1B
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_CLKCONFIG,   0x00);  //0x01        
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_INTCONFIG,   0x02);
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_PWRCONFIG,   0x01);
         FLOAT_Singal_SPI_Write_2(LDC1000_CMD_THRESHILSB,  0x50);
	 FLOAT_Singal_SPI_Write_2(LDC1000_CMD_THRESHIMSB,  0x14);
	 FLOAT_Singal_SPI_Write_2(LDC1000_CMD_THRESLOLSB,  0xC0);
	 FLOAT_Singal_SPI_Write_2(LDC1000_CMD_THRESLOMSB,  0x12);
         FLOAT_SPI_Read_Buf_2(LDC1000_CMD_REVID,&orgVal[0],12);//orgVal[]对应上面写入的值说明初始化正常
       
  
} 

int ldc_read_avr_2()
{

    char rpi=0;  //取rpi次平均值    
    for (rpi=0;rpi<rpi_max;rpi++)
    {

      FLOAT_SPI_Read_Buf_2(LDC1000_CMD_PROXLSB,&proximtyData[0],2);  
      proximtyDataTEMP = ((unsigned char)proximtyData[1]<<8) + proximtyData [0]; 
      proximtyDataSUM += proximtyDataTEMP;
      if (proximtyDataTEMP < proximtyDataMIN)   //在100个proximtyDataTEMP中取最大，最小
        proximtyDataMIN = proximtyDataTEMP;
      if (proximtyDataTEMP > proximtyDataMAX)
        proximtyDataMAX = proximtyDataTEMP;
    }
     proximtyDataAVE = proximtyDataSUM /rpi_max;
     proximtyDataSUM=0;
     proximtyDataAVE_LAS=proximtyDataAVE;
  
    return   proximtyDataAVE; 

}
long int filter_2()
{
   char count,i,j,count1;
   char count2=0;
 
   long int temp;
   long int sum=0;
   for(count=0;count<NN;count++)
   {
      value_buf[count] = ldc_read_avr_2();
   }
   
   for(count1=0;count1<NN;count1++)
   {  
   if(value_buf[count1]<32768)
   {
   new_value_buf[count2]=value_buf[count1];
   count2++;
   }  
   }
   
   
   for (j=0;j<count2-1;j++)
   {
      for (i=0;i<count2-j;i++)
      {
        if ( new_value_buf[i]>new_value_buf[i+1] )
         {
            temp = new_value_buf[i];
            new_value_buf[i] = new_value_buf[i+1];
            new_value_buf[i+1] = temp;
         }
      }
   }

   for(count=1;count<count2-1;count++)
   {
      sum += new_value_buf[count];
   }
  
   sum=sum/10;  
   return (long int)(sum/(count2-2));


}

void FLOAT_SPI_init_2()
{  
       
         gpio_init (PTB4, GPI,1);//MISO
         gpio_init (PTB5 ,GPO,1);//MOSI
         gpio_init (PTB6, GPO,1);// CSN
         gpio_init (PTB7, GPO,0);//SCK
         
         CSN_H_2;
         SCK_L_2;
         MOSI_H_2;
         
  
}
/****************************************************************************************************
* Function Name: uchar FLOAT_SPI_RW(uchar wdata)
* Description  : read and write of SPI.
* Arguments    : wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_SPI_RW_2(uchar rwdata)
{  
    
	uchar spi_rw_i=0;	
        uchar temp=0;
        for(spi_rw_i=0;spi_rw_i<8;spi_rw_i++)   	// output 8-bit
   	{
   	        /*** prepare the write data of read before the coming of rising up******/
	          if(rwdata & 0x80)
                    MOSI_H_2;
   		  else 
                    MOSI_L_2;
   		  rwdata<<=1;           		// shift next bit to MSB
                  temp<<=1;
		SCK_L_2;             //Set SCK high    Rising up 
               
   		if(MISO_2) 
                  temp|=1;
   		SCK_H_2;            //set  SCK low     Falling down
                
   	}
    return(temp);           		  		// return read byte
    
 
}
/****************************************************************************************************
* Function Name: uchar FLOAT_Singal_SPI(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
uchar FLOAT_Singal_SPI_Read_2(uchar reg)
{
	uchar rdata;
	
	CSN_L_2;                // CSN low, initialize SPI communication...
       
        FLOAT_delay_us(2);
         
         reg=reg|0x80;//read
	FLOAT_SPI_RW_2(reg);            // Select register to read from..
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
         
	rdata = FLOAT_SPI_RW_2(NULL);    // ..then read registervalue
       
        FLOAT_delay_us(1700);
	CSN_H_2;                // CSN high, terminate SPI communication
	
	return rdata;        // return register value
}
/****************************************************************************************************
* Function Name: void FLOAT_Singal_SPI_Write(uchar reg,uchar wdata)
* Description  : registers read and write of device.
* Arguments    : commond,wdata
* Return Value : rdata
****************************************************************************************************/
void FLOAT_Singal_SPI_Write_2(uchar reg,uchar wdata)
{
	
	CSN_L_2;                // CSN low, initialize SPI communication...
      
        FLOAT_delay_us(2);//2us
        reg=reg&~0x80;
	FLOAT_SPI_RW_2(reg);            // Select register to read from..
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
        
	FLOAT_SPI_RW_2(wdata);    // ..then read registervalue
        FLOAT_delay_us(1700);//875us
	CSN_H_2;              // CSN high, terminate SPI communication
       
	
}

/****************************************************************************************************
* Function Name: void FLOAT_SPI_Read_Buf(uchar reg, uchar *pBuf, uchar len)
* Description  : read muche registers
* Arguments    : reg,len
* Return Value : *pBuf
****************************************************************************************************/

void FLOAT_SPI_Read_Buf_2(uchar reg, uchar *pBuf, uchar len)
{
	uchar spi_rw_i;
	
	CSN_L_2;                   		// Set CSN low, init SPI tranaction
       
        reg=reg|0x80;//read
	FLOAT_SPI_RW_2(reg);       		// Select register to write to and read status uchar
	
	for(spi_rw_i=0;spi_rw_i<len;spi_rw_i++)
        {  
	pBuf[spi_rw_i] = FLOAT_SPI_RW_2(NULL);    // 
	 }
	CSN_H_2;     
       

}