/**************************************************************************************************************
    数组：     DataScope_OutPut_Buffer[42] ，待发送帧数据缓存区，里面存放经过功能函数格式化后的待发送帧数据。 

功能函数：
      1、DataScope_Get_Channel_Data(float Data,unsigned char Channel) ：
                  此函数无返回。
函数功能说明：将指定通道的待发送单精度浮点数据转换为字节数据存放至 DataScope_OutPut_Buffer (待发送帧数据缓存区)。
						              
参数说明:      1、Channel，单字节无符号整型，识别范围：1 - 10
		含义：指定本次数据对应哪个通道。
												 
               2、Data ，  单精度浮点型。
		含义：传递待转换的通道数据。
												 
      2、DataScope_Data_Generate(unsigned char Channel_Number) ：
                返回一个单字节无符号整型。						
                返回说明：返回串口需要发送的字节个数，返回 0 表示发生本次帧数据生成错误。
函数功能说明：产生指定通道个数的待发送帧数据。存放至 DataScope_OutPut_Buffer (待发送帧数据缓存区)，
						
参数说明;
                1、Channel_Number，单字节无符号整型，识别范围：1 - 10
		含义：设定需要发送的通道个数，最少1个通道，最多10个通道。
						
补充说明：与 DataScope 通讯的帧数据长度不固定，故调用 DataScope_Data_Generate 时
传递不同的Channel_Number，将在DataScope_OutPut_Buffer中生成不同长度的 格式化帧数据。
**************************************************************************************************************/
//头文件内容：

#ifndef __DATA_SEND_H
#define __DATA_SEND_H

extern unsigned char Data_OutPut_Buffer[42];	   //待发送帧数据缓存区

void Data_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

unsigned char Data_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void sendNData(float *buff,unsigned char Num);

void sendmydata(unsigned char *wareaddr,unsigned short int waresize);//多功能串口助手示波器

void OutData_choose(void);            //示波器
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);//超级示波器
void OutPut_Data(void);
#endif 