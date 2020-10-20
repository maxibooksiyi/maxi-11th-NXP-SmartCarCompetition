/******************** ************************* ********************
 * 文件名       ：adc.c
 * 描述         ：adc驱动函数
 *
**********************************************************************************/
#include "common.h"
#include "adc.h"
tADC_Config Master_Adc_Config;          //该结构体包含了需要的ADC/PGA配置

volatile struct ADC_MemMap *ADCx[2] = {ADC0_BASE_PTR, ADC1_BASE_PTR}; //定义两个指针数组保存 ADCx 的地址
/*************************************************************************
*
*
*  函数名称：adc_init
*  功能说明：AD初始化，使能时钟
*  参数说明：ADCn        模块号（ ADC0、 ADC1）
*  函数返回：无
*  修改时间：2012-2-10
*  备    注：参考苏州大学的例程
*************************************************************************/
void adc_init(ADCn adcn, ADC_Ch ch)
{
  switch(adcn)
    {
    case ADC0:       /*   ADC0  */
        SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK );        //开启ADC0时钟
        SIM_SOPT7 &= ~(SIM_SOPT7_ADC0ALTTRGEN_MASK  | SIM_SOPT7_ADC0PRETRGSEL_MASK);
        SIM_SOPT7 = SIM_SOPT7_ADC0TRGSEL(0);

        switch(ch)
        {
        case AD4b:   //ADC1_SE4b -- PTC2
            SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
            PORT_PCR_REG(PORTD_BASE_PTR,2) =  PORT_PCR_MUX(0);
        case AD5b:   //ADC1_SE5b -- PTD1
            SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
            PORT_PCR_REG(PORTD_BASE_PTR, 1) =  PORT_PCR_MUX(0);
            break;
        case AD6b:   //ADC1_SE6b -- PTD5
        case AD7b:   //ADC1_SE7b -- PTD6
            SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
            PORT_PCR_REG(PORTD_BASE_PTR, ch - 6 + 5) =  PORT_PCR_MUX(0);
            break;

        case AD8:   //ADC0_SE8 -- PTB0
        case AD9:   //ADC0_SE9 -- PTB1
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
            PORT_PCR_REG(PORTB_BASE_PTR, ch - AD8 + 0) =  PORT_PCR_MUX(0);
            break;
        case AD10:  //ADC0_SE10 -- PTA7
        case AD11:  //ADC0_SE11 -- PTA8
            SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
            PORT_PCR_REG(PORTA_BASE_PTR, ch - AD10 + 7) =  PORT_PCR_MUX(0);
            break;
        case AD12:  //ADC0_SE12 -- PTB2
        case AD13:  //ADC0_SE13 -- PTB3
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
            PORT_PCR_REG(PORTB_BASE_PTR, ch - AD12 + 2) =  PORT_PCR_MUX(0);
            break;
        case AD14:  //ADC0_SE14 -- PTC0
        case AD15:  //ADC0_SE15 -- PTC1
            SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
            PORT_PCR_REG(PORTC_BASE_PTR, ch - AD14 + 0) =  PORT_PCR_MUX(0);
            break;
        case AD17:   //ADC0_SE17 -- PTE24
        case AD18:   //ADC0_SE17 -- PTE25
            SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
            PORT_PCR_REG(PORTE_BASE_PTR, ch - AD17 + 24) =  PORT_PCR_MUX(0);
            break;
        default:
            return;
        }
        return;

    case ADC1:       /*   ADC1    */
        SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );
        SIM_SOPT7 &= ~(SIM_SOPT7_ADC1ALTTRGEN_MASK  | SIM_SOPT7_ADC1PRETRGSEL_MASK) ;
        SIM_SOPT7 = SIM_SOPT7_ADC1TRGSEL(0);

        switch(ch)
        {
        case AD4a:   //ADC1_SE4a -- PTE0
        case AD5a:   //ADC1_SE5a -- PTE1
        case AD6a:   //ADC1_SE6a -- PTE2
        case AD7a:   //ADC1_SE7a -- PTE3
            SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
            PORT_PCR_REG(PORTE_BASE_PTR, ch - AD4a + 0) =  PORT_PCR_MUX(0);
            break;
        case AD4b:   //ADC1_SE4b -- PTC8
        case AD5b:   //ADC1_SE5b -- PTC9
        case AD6b:   //ADC1_SE6b -- PTC10
        case AD7b:   //ADC1_SE7b -- PTC11
            SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
            PORT_PCR_REG(PORTC_BASE_PTR, ch - 4 + 8) =  PORT_PCR_MUX(0);
            break;
        case AD8:  //ADC1_SE8 -- PTB0
        case AD9:  //ADC1_SE9 -- PTB1
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
            PORT_PCR_REG(PORTB_BASE_PTR, ch - AD8 + 0) =  PORT_PCR_MUX(0);
            break;
        case AD10:  //ADC1_SE10 -- PTB4
        case AD11:  //ADC1_SE11 -- PTB5
        case AD12:  //ADC1_SE12 -- PTB6
        case AD13:  //ADC1_SE13 -- PTB7
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
            PORT_PCR_REG(PORTB_BASE_PTR, ch - 6) =  PORT_PCR_MUX(0);
            break;
        case AD14:  //ADC1_SE14 -- PTB10
        case AD15:  //ADC1_SE15 -- PTB11
            SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
            PORT_PCR_REG(PORTB_BASE_PTR, ch - AD10 + 4) =  PORT_PCR_MUX(0);
            break;
        case AD17:  //ADC1_SE17 -- PTA17
            SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
            PORT_PCR_REG(PORTA_BASE_PTR, ch) =  PORT_PCR_MUX(0);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

u16 getad(ADCn adcn, ADC_Ch ch, ADC_nbit bit)
  { uint16 result = 0;

    ADC_CFG1_REG(ADCx[adcn])  =  0
                                 //  ADC_CFG1_ADLPC_MASK     // 0正常，1低功耗
                                 | ADC_CFG1_ADIV(ADIV_4)     // 0,2,4,8时钟分频设置
                                 //| ADC_CFG1_ADLSMP_MASK    // 0正常，1加长采样时间
                                 | ADC_CFG1_MODE(bit)        // 8,10,12,16位采样
                                 | ADC_CFG1_ADICLK(ADICLK_BUS);// 时钟选择：总线，总线/2，交替，异步

    //ADC_CFG2_REG(ADCx[adcn])  = 0
                                 //| ADC_CFG2_MUXSEL_MASK      //0，A端口，1，B端口
                                 //| ADC_CFG2_ADACKEN_MASK    // 0异步时钟不使能，1异步时钟使能
                                 //| ADC_CFG2_ADHSC_MASK      // 0正常，1高速采样
                                 //| ADC_CFG2_ADLSTS(ADLSTS_20)// 在选择加长采样周期的情况下：加20,16,12,6个周期
                                    ;

    ADC_SC1_REG(ADCx[adcn], A) =
                                  0
                                  //|ADC_SC1_AIEN_MASK //0不产生中断，1产生中断
                                  //|ADC_SC1_DIFF_MASK //0单端，1差分
                                  |ch;

    //ADC_SC1_REG(ADCx[adcn], B) =   ch;

    //ADC_PGA_REG(ADCx[adcn])=0|0x80|0x00;

    while (( ADC_SC1_REG(ADCx[adcn], 0 ) & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK);//等待转换完成
    result = ADC_R_REG(ADCx[adcn], 0);
    ADC_SC1_REG(ADCx[adcn], 0) &= ~ADC_SC1_COCO_MASK;
    return result;
  }

/*u16 getadb(ADCn adcn, ADC_Ch ch, ADC_nbit bit)
  { uint16 result = 0;
    int c=ch-32;
    ADC_CFG1_REG(ADCx[adcn])  =  0
                                 //  ADC_CFG1_ADLPC_MASK     // 0正常，1低功耗
                                 | ADC_CFG1_ADIV(ADIV_4)     // 0,2,4,8时钟分频设置
                                 //| ADC_CFG1_ADLSMP_MASK    // 0正常，1加长采样时间
                                 | ADC_CFG1_MODE(bit)        // 8,12,10,16位采样   注意顺序
                                 | ADC_CFG1_ADICLK(ADICLK_BUS);// 时钟选择：总线，总线/2，交替，异步

    ADC_CFG2_REG(ADCx[adcn])  = 0
                                 | ADC_CFG2_MUXSEL_MASK      //0，A端口，1，B端口
                                 //| ADC_CFG2_ADACKEN_MASK    // 0异步时钟不使能，1异步时钟使能
                                 //| ADC_CFG2_ADHSC_MASK      // 0正常，1高速采样
                                 //| ADC_CFG2_ADLSTS(ADLSTS_20)// 在选择加长采样周期的情况下：加20,16,12,6个周期
                                    ;

    ADC_SC1_REG(ADCx[adcn], 0) =  0
                                  //|ADC_SC1_AIEN_MASK //0不产生中断，1产生中断
                                  //|ADC_SC1_DIFF_MASK //0单端，1差分
                                  |ch;

    ADC_SC1_REG(ADCx[adcn], 1) =  0
                                  //|ADC_SC1_AIEN_MASK //0不产生中断，1产生中断
                                  //|ADC_SC1_DIFF_MASK //0单端，1差分
                                  |c;

    //ADC_PGA_REG(ADCx[adcn])=0|0x80|0x00;

    while (( ADC_SC1_REG(ADCx[adcn],1)&ADC_SC1_COCO_MASK)!=ADC_SC1_COCO_MASK);//等待转换完成
    result=ADC_R_REG(ADCx[adcn],1);
    ADC_SC1_REG(ADCx[adcn],1)&=~ADC_SC1_COCO_MASK;
    return result;
  } */

u16 getadbiger(ADCn adcn, ADC_Ch ch, ADC_nbit bit,u8 biger)
  { uint16 result = 0;

    ADC_CFG1_REG(ADCx[adcn])  =  0
                                 //  ADC_CFG1_ADLPC_MASK     // 0正常，1低功耗
                                 | ADC_CFG1_ADIV(ADIV_4)     // 0,2,4,8时钟分频设置
                                 //| ADC_CFG1_ADLSMP_MASK    // 0正常，1加长采样时间
                                 | ADC_CFG1_MODE(bit)        // 8,10,12,16位采样
                                 | ADC_CFG1_ADICLK(ADICLK_BUS);// 时钟选择：总线，总线/2，交替，异步

    //ADC_CFG2_REG(ADCx[adcn])  = 0
                                 //| ADC_CFG2_MUXSEL_MASK      //0，A端口，1，B端口
                                 //| ADC_CFG2_ADACKEN_MASK    // 0异步时钟不使能，1异步时钟使能
                                 //| ADC_CFG2_ADHSC_MASK      // 0正常，1高速采样
                                 //| ADC_CFG2_ADLSTS(ADLSTS_20)// 在选择加长采样周期的情况下：加20,16,12,6个周期
                                    ;

    ADC_PGA_REG(ADCx[adcn])=(0|0x80|(biger&0x0f))<<16;//放大1,2,4,8,16,32,64倍

    ADC_SC1_REG(ADCx[adcn], A) =
                                  0
                                  //|ADC_SC1_AIEN_MASK //0不产生中断，1产生中断
                                  //|ADC_SC1_DIFF_MASK //0单端，1差分
                                  |ch;

    //ADC_SC1_REG(ADCx[adcn], B) =   ch;


    while (( ADC_SC1_REG(ADCx[adcn], 0 ) & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK);//等待转换完成
    result = ADC_R_REG(ADCx[adcn], 0);
    ADC_SC1_REG(ADCx[adcn], 0) &= ~ADC_SC1_COCO_MASK;
    return result;
  }
/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：adc_start
*  功能说明：启动adc软件采样，B通道不能用于软件触发！！！！
*  参数说明：ADCx        模块号（ ADC0、 ADC1）
*            ADC_Channel 通道号
*            ADC_nbit    精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*  函数返回：无
*  修改时间：2012-2-10
*  备    注：修改苏州大学的例程
*************************************************************************/
void adc_start(ADCn adcn, ADC_Ch ch, ADC_nbit bit)
{

    Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH( ch );

    //初始化ADC默认配置
    Master_Adc_Config.CONFIG1  = ADLPC_NORMAL
                                 | ADC_CFG1_ADIV(ADIV_4)
                                 | ADLSMP_LONG
                                 | ADC_CFG1_MODE(bit)
                                 | ADC_CFG1_ADICLK(ADICLK_BUS);
    Master_Adc_Config.CONFIG2  = MUXSEL_ADCA    //MUXSEL_ADCA
                                 | ADACKEN_DISABLED
                                 //| ADHSC_HISPEED
                                 | ADC_CFG2_ADLSTS(ADLSTS_20) ;

    Master_Adc_Config.COMPARE1 = 0x1234u ;                 //任意值
    Master_Adc_Config.COMPARE2 = 0x5678u ;                 //任意值

    adc_config_alt(ADCx[adcn], &Master_Adc_Config);       // 配置 ADCn
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：adc_stop
*  功能说明：停止ADC转换
*  参数说明：ADCx        模块号（ ADC0、 ADC1）
*            ADC_Channel 通道号
*  函数返回：无
*  修改时间：2012-2-10
*  备    注：修改苏州大学的例程
*************************************************************************/
void adc_stop(ADCn adcn)
{
    Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(Module_disabled);
    adc_config_alt(ADCx[adcn], &Master_Adc_Config);  // 配置ADC0
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：ad_once
*  功能说明：采集一次一路模拟量的AD值
*  参数说明：ADCn        模块号（ ADC0、 ADC1）
*            ADC_Channel 通道号
*            ADC_nbit    精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*  函数返回：无符号结果值
*  修改时间：2012-2-10
*  备    注：参考苏州大学的例程，B通道不能软件触发！！！！
*************************************************************************/
u16 ad_once(ADCn adcn, ADC_Ch ch, ADC_nbit bit) //采集某路模拟量的AD值
{
    u16 result = 0;
    ASSERT( ((adcn == ADC0) && (ch >= AD8 && ch <= AD18)) || ((adcn == ADC1) && (ch >= AD4a && ch <= AD17)) ) ; //使用断言检测ADCn_CHn是否正常

    adc_start(adcn, ch, bit);	  //启动ADC转换

    while (( ADC_SC1_REG(ADCx[adcn], 0 ) & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK);
    result = ADC_R_REG(ADCx[adcn], 0);
    ADC_SC1_REG(ADCx[adcn], 0) &= ~ADC_SC1_COCO_MASK;
    return result;
}
/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：ad_ave
*  功能说明：多次采样，取平均值
*  参数说明：ADCx        模块号（ ADC0、 ADC1）
*            ADC_Channel 通道号
*            ADC_nbit    精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*            N           均值滤波次数(范围:0~255)
*  函数返回：16位无符号结果值
*  修改时间：2012-2-10
*  备    注：修改苏州大学的例程
*************************************************************************/
u16 ad_ave(ADCn adcn, ADC_Ch ch, ADC_nbit bit, u8 N) //均值滤波
{
    u32 tmp = 0;
    u8  i;
    ASSERT( ((adcn == ADC0) && (ch >= AD8 && ch <= AD18)) || ((adcn == ADC1) && (ch >= AD4a && ch <= AD17)) ) ; //使用断言检测ADCn_CHn是否正常

    for(i = 0; i < N; i++)
        tmp += ad_once(adcn, ch, bit);
    tmp = tmp / N;
    return (u16)tmp;
}
/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：adc_config_alt
*  功能说明：将adc寄存器结构体配置进adc寄存器
*  参数说明：adcmap      adc基址寄存器地址（ADC0_BASE_PTR,ADC1_BASE_PTR）
*            ADC_CfgPtr  存放 寄存器值的结构体
*  函数返回：无
*  修改时间：2012-2-10
*  备    注：修改官方工程的例程
*************************************************************************/
void adc_config_alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
    ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
    ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
    ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1;
    ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
    ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
    ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
    ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;
    ADC_SC1_REG(adcmap, A) = ADC_CfgPtr->STATUS1A;
    ADC_SC1_REG(adcmap, B) = ADC_CfgPtr->STATUS1B;
}