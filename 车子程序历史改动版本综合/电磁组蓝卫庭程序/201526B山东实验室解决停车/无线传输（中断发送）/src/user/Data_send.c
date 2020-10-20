#include "include.h"

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址 
//附加说明：用户无需直接操作此函数 
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回 
void Data_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
         switch (Channel)
		{
                  case 1:  Float2Byte(&Data,Data_OutPut_Buffer,1); break;
                  case 2:  Float2Byte(&Data,Data_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,Data_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,Data_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,Data_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,Data_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,Data_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,Data_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,Data_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,Data_OutPut_Buffer,37); break;
		}
    }	 
}

//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败 
unsigned char Data_Data_Generate(unsigned char Channel_Number)
{
     if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
     else
     {	
	        Data_OutPut_Buffer[0] = '$';  //帧头
		
                switch(Channel_Number)   
                { 
		 case 1:   Data_OutPut_Buffer[5]  =  5; return  6; break;   
		 case 2:   Data_OutPut_Buffer[9]  =  9; return 10; break;
		 case 3:   Data_OutPut_Buffer[13] = 13; return 14; break;
		 case 4:   Data_OutPut_Buffer[17] = 17; return 18; break;
		 case 5:   Data_OutPut_Buffer[21] = 21; return 22; break; 
		 case 6:   Data_OutPut_Buffer[25] = 25; return 26; break;
		 case 7:   Data_OutPut_Buffer[29] = 29; return 30; break;
		 case 8:   Data_OutPut_Buffer[33] = 33; return 34; break;
		 case 9:   Data_OutPut_Buffer[37] = 37; return 38; break;
                 case 10:  Data_OutPut_Buffer[41] = 41; return 42; break;
                 }	 
      }
      return 0;
}

/**************************************************************
  函数名称：sendNData()
  功能说明：上位机发送数据
  参数说明：数组地址，通道N
**************************************************************/
void sendNData(float *buff,unsigned char Num)
{
     unsigned char i;          //计数变量
     unsigned char Send_Count; //串口需要发送的数据个数		 	  		 
     for(i=0;i<Num;i++)	
     {
	    Data_Get_Channel_Data(buff[i], i+1);  //将数据buff[i]写入通道 i            
     }
	    Send_Count = Data_Data_Generate(Num); //生成Num个通道的 格式化帧数据，返回帧数据长度
            for(i=0;i<Send_Count;i++)
            {
                uart_putchar(UART1,Data_OutPut_Buffer[i]);
            }	      
            delayms(2); //20fps, 帧间隔时间。 不同电脑配置及 USB-TTL 设备的优劣均会影响此时间的长短，建议实测为准。  
}
/**************************************************************                        
  函数名称：sendmydata()  山外多功能上位机
  功能说明：上位机发送数据
 参数说明：数组地址，数组长度
**************************************************************/
void sendmydata(unsigned char *wareaddr,unsigned short int waresize)
{
   #define CMD_WARE 3
   unsigned char cmdf[2]={CMD_WARE,~CMD_WARE};
   unsigned char cmdr[2]={~CMD_WARE,CMD_WARE};
   uart_sendN(UART1 ,cmdf,sizeof(cmdf));
   uart_sendN(UART1 ,wareaddr,waresize);
   uart_sendN(UART1 ,cmdr,sizeof(cmdr));
}
/*****************************
 超级示波器
*****************************/
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
/**********************************************
最多四个通道

*/
void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
    uart_putchar(UART1, databuf[i]);
}



