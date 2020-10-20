#include "include.h"
#include "CCD.h"


extern uint8 yuzhi_max1;     //CCD1��ֵ����
extern uint8 yuzhi_min1;     //CCD1��ֵ����
extern uint8 yz;
extern uint8 Pixel1[131];    //CCD1����(0~127�������ݡ�128min��129max��130��ֵ)
extern uint8 CCD_Flag;
uint8 WhiteNum=0;

/*************************************************************************
*                            ZJL
*
*  �������ƣ�CCD_init
*  ����˵����CCD��ʼ��
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2015-10-20
*  ��    ע��
*************************************************************************/
void CCD_init(void)
{
  gpio_init (PTE3, GPO,LOW);     //SI�ź�����    E3��
  gpio_init (PTE2, GPO,LOW);    //CLK�ź�����    E2��
  adc_init(ADC1_SE4a) ;         //AO�����       E0��
}

/*************************************************************************
*                           ZJL
*
*  �������ƣ�CCDDelay
*  ����˵����CCD������ʱ
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2015-10-20
*  ��    ע��
*************************************************************************/
void CCDDelay(void)
 {
   volatile uint8 i ;
   for(i=0;i<1;i++) 
   {
    asm("nop");
   }
}

/*************************************************************************
*                           ZJL
*
*  �������ƣ�CCD_RD
*  ����˵����CCD��������
*  ����˵����
*  �������أ���
*  �޸�ʱ�䣺2015-10-20
*  ��    ע��
*ImageData =  ad_once(ADC1_AD4a, ADC_8bit);
*************************************************************************/
void CCD_RD(unsigned char * ImageData)
{
  unsigned char i;
  CCD_SI_H;    //SI=1
  CCDDelay();
  CCD_CLK_H;   //CLK=1
  CCDDelay();
  CCD_SI_L;    //SI=0
  *ImageData =  adc_once(ADC1_SE4a, ADC_8bit);
  CCD_CLK_L;   //CLK=0
  for(i=0; i<127; i++)
  {
    CCDDelay();
    CCD_CLK_H; //CLK=1
    CCDDelay();
    ImageData ++ ;
    *ImageData =  adc_once(ADC1_SE4a, ADC_8bit);
    CCD_CLK_L; //CLK=0
  }
  CCDDelay();
  CCD_CLK_H; //CLK=1
  CCDDelay();
  CCD_CLK_L; //CLK=0
}
void Pixel_erzhihua(unsigned char * CCD)
{
    uint8 j;
    uint8 max;
    uint8 min;
    uint8 *CCD_0;           //CCD�洢����ָ��
    CCD_0=CCD;           //ָ�븳ֵ
    max=*CCD;
    min=*CCD;
    CCD++;
    for(j=1;j<128;j++)
    {
      max=(max<*CCD)?*CCD:max;
      min=(min>*CCD)?*CCD:min;	
      CCD++;
    }
    *CCD=min;            //���ڵ�129����ַ
    CCD++;
    *CCD=max;            //���ڵ�130����ַ
    CCD++;
    yz=8*(max+min)/20;
    if(CCD_0==Pixel1)
    {
      yz=(yz<yuzhi_min1)?yuzhi_min1:yz;
      yz=(yz>yuzhi_max1)?yuzhi_max1:yz;
    }
    *CCD=yz;             //���ڵ�131����ַ
  for(j=0;j<128;j++)
    {
     
       if(*CCD_0 >*CCD)
       { *CCD_0=1;
          WhiteNum++;
       }
       else
       {
        *CCD_0=0;
       
       }
      CCD_0++;
     
    }
    
}
void Get_Img(void)
{  int lv_num=1;
   CCD_RD(Pixel1);
   for(lv_num=1;lv_num<127;lv_num++)
   Pixel1[lv_num]=get_mid(Pixel1[lv_num-1],Pixel1[lv_num],Pixel1[lv_num+1]);
   Pixel_erzhihua(Pixel1);
}
void yuzhi(void)
{       
        int temp1;
        int i;
        int temp2;
        int min;
        int max;
        int pj1;
        for(i=0;i<5;i++)
        {
         while(CCD_Flag ==0);
         CCD_Flag = 0;
          CCD_RD(Pixel1);
          Pixel_erzhihua(Pixel1);
          temp1 = temp1 + Pixel1[129] ;
          temp2 = temp2 + Pixel1[128] ;
          
        }
        max = temp1/5;
        min = temp2/5;
        pj1 = 8*(max+min)/20;
        yuzhi_max1 = (8*(max + min)/20) * 1.1;
        yuzhi_min1 = (8*(max + min)/20) * 0.8;        
}