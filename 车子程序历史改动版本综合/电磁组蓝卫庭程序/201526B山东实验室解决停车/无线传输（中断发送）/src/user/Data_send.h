/**************************************************************************************************************
    ���飺     DataScope_OutPut_Buffer[42] ��������֡���ݻ������������ž������ܺ�����ʽ����Ĵ�����֡���ݡ� 

���ܺ�����
      1��DataScope_Get_Channel_Data(float Data,unsigned char Channel) ��
                  �˺����޷��ء�
��������˵������ָ��ͨ���Ĵ����͵����ȸ�������ת��Ϊ�ֽ����ݴ���� DataScope_OutPut_Buffer (������֡���ݻ�����)��
						              
����˵��:      1��Channel�����ֽ��޷������ͣ�ʶ��Χ��1 - 10
		���壺ָ���������ݶ�Ӧ�ĸ�ͨ����
												 
               2��Data ��  �����ȸ����͡�
		���壺���ݴ�ת����ͨ�����ݡ�
												 
      2��DataScope_Data_Generate(unsigned char Channel_Number) ��
                ����һ�����ֽ��޷������͡�						
                ����˵�������ش�����Ҫ���͵��ֽڸ��������� 0 ��ʾ��������֡�������ɴ���
��������˵��������ָ��ͨ�������Ĵ�����֡���ݡ������ DataScope_OutPut_Buffer (������֡���ݻ�����)��
						
����˵��;
                1��Channel_Number�����ֽ��޷������ͣ�ʶ��Χ��1 - 10
		���壺�趨��Ҫ���͵�ͨ������������1��ͨ�������10��ͨ����
						
����˵������ DataScope ͨѶ��֡���ݳ��Ȳ��̶����ʵ��� DataScope_Data_Generate ʱ
���ݲ�ͬ��Channel_Number������DataScope_OutPut_Buffer�����ɲ�ͬ���ȵ� ��ʽ��֡���ݡ�
**************************************************************************************************************/
//ͷ�ļ����ݣ�

#ifndef __DATA_SEND_H
#define __DATA_SEND_H

extern unsigned char Data_OutPut_Buffer[42];	   //������֡���ݻ�����

void Data_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

unsigned char Data_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void sendNData(float *buff,unsigned char Num);

void sendmydata(unsigned char *wareaddr,unsigned short int waresize);//�๦�ܴ�������ʾ����

void OutData_choose(void);            //ʾ����
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);//����ʾ����
void OutPut_Data(void);
#endif 