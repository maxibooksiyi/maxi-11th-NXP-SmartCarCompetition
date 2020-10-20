#include "include.h"

//����˵�����������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ 
//����˵�����û�����ֱ�Ӳ����˺��� 
//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
//�����޷��� 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data��ͨ������
//Channel��ѡ��ͨ����1-10��
//�����޷��� 
void Data_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //ͨ����������10�����0��ֱ����������ִ�к���
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

//����˵�������� DataScopeV1.0 ����ȷʶ���֡��ʽ
//Channel_Number����Ҫ���͵�ͨ������
//���ط��ͻ��������ݸ���
//����0��ʾ֡��ʽ����ʧ�� 
unsigned char Data_Data_Generate(unsigned char Channel_Number)
{
     if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
     else
     {	
	        Data_OutPut_Buffer[0] = '$';  //֡ͷ
		
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
  �������ƣ�sendNData()
  ����˵������λ����������
  ����˵���������ַ��ͨ��N
**************************************************************/
void sendNData(float *buff,unsigned char Num)
{
     unsigned char i;          //��������
     unsigned char Send_Count; //������Ҫ���͵����ݸ���		 	  		 
     for(i=0;i<Num;i++)	
     {
	    Data_Get_Channel_Data(buff[i], i+1);  //������buff[i]д��ͨ�� i            
     }
	    Send_Count = Data_Data_Generate(Num); //����Num��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���
            for(i=0;i<Send_Count;i++)
            {
                uart_putchar(UART1,Data_OutPut_Buffer[i]);
            }	      
            delayms(2); //20fps, ֡���ʱ�䡣 ��ͬ�������ü� USB-TTL �豸�����Ӿ���Ӱ���ʱ��ĳ��̣�����ʵ��Ϊ׼��  
}
/**************************************************************                        
  �������ƣ�sendmydata()  ɽ��๦����λ��
  ����˵������λ����������
 ����˵���������ַ�����鳤��
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
 ����ʾ����
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
����ĸ�ͨ��

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



