/******************************************
 任务说明：FTM模块，输出PWM
           用于舵机，电机控制
           正交解码测速
 团队：首安六队
 时间:2014.01.15
******************************************/

#include "include.h"
#include  "FTM.h"
/***********************************************************************
 PWM输出通道说明：(以英文为准)
 FTM共3个模块,每个模块8个通道，X表示不可用：
        --FTM0--  --FTM1--  --FTM2--
CH0       PTC1      PTA8      PTA10
CH1       PTC2      PTA9      PTA11
CH2       PTC3       ×         ×
CH3       PTC4       ×         ×
CH4       PTD4       ×         ×
CH5       PTD5       ×         ×
CH6       PTD6       ×         ×
CH7       PTD7       ×         ×
**********************************************************************/

volatile struct FTM_MemMap *FTMx[3]={FTM0_BASE_PTR,FTM1_BASE_PTR,FTM2_BASE_PTR}; //定义三个指针数组保存 FTMn 的地址

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：FTM_PWM_init
*  功能说明：初始化FTM的PWM功能并设置频率、占空比。设置通道输出占空比。
*  参数说明：FTMn        模块号（FTM0、  FTM1、  FTM2）
*            CHn         通道号（CH0~CH7）
*            freq        频率（单位为Hz）
*            duty        占空比
*  函数返回：无
*  修改时间：2012-2-14
*  备    注：同一个FTM，各通道的PWM频率是一样的，共3个FTM，即可以输出3个不同频率PWM
*************************************************************************/
void FTM_PWM_init(FTMn ftmn, CHn ch, u32 freq, u32 duty)
{
    u32 clk_hz = (bus_clk_khz * 1000) >> 1;       //bus频率/2
    u16 mod;
    u8 sc_ps;
    u16 cv;

    ASSERT( (ftmn == FTM0) || ( (ftmn == FTM1 || ftmn == FTM2 ) && (ch <= CH1))   ); //检查传递进来的通道是否正确
    ASSERT( freq <= (clk_hz >> 1) );              //用断言检测 频率 是否正常 ,频率必须小于时钟二分之一
    /*       计算频率设置        */
    mod = (clk_hz >> 16 ) / freq ;
    for(sc_ps = 0; (mod >> sc_ps) >= 1; sc_ps++);
    if(freq < 1000)sc_ps++;
    mod = (clk_hz >> sc_ps) / freq;
    cv = (duty * (mod - 0 + 1)) / FTM_PRECISON;
    /******************* 开启时钟 和 复用IO口*******************/
    //注，这里代码虽然长，但编译时会删掉很多没用的部分，不影响速度
    switch(ftmn)
    {
    case FTM0:
        SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;                           //使能FTM0时钟
        switch(ch)
        {
        case CH0:
            if(FTM0_CH0 == PTC1)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 1) = PORT_PCR_MUX(4);  // PTC1
            }
            else if(FTM0_CH0 == PTA3)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 3) = PORT_PCR_MUX(3);  // PTA3
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH1:
            if(FTM0_CH1 == PTC2)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 2) = PORT_PCR_MUX(4);  // PTC2
            }
            else if(FTM0_CH1 == PTA4)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 4) = PORT_PCR_MUX(3);  // PTA4
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH2:
            if(FTM0_CH2 == PTC3)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 3) = PORT_PCR_MUX(4);  // PTC3
            }
            else if(FTM0_CH2 == PTA5)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 5) = PORT_PCR_MUX(3);  // PTA5
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH3:
            if(FTM0_CH3 == PTC4)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 4) = PORT_PCR_MUX(4);  // PTC4
            }
            else if(FTM0_CH3 == PTA6)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 6) = PORT_PCR_MUX(3);  // PTA6
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH4:
            if(FTM0_CH4 == PTD4)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 4) = PORT_PCR_MUX(4);  // PTD4
            }
            else if(FTM0_CH4 == PTA7)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 7) = PORT_PCR_MUX(3);  // PTA7
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH5:
            if(FTM0_CH5 == PTD5)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 5) = PORT_PCR_MUX(4);  // PTD5
            }
            else if(FTM0_CH5 == PTA0)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 0) = PORT_PCR_MUX(3);  // PTA0
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH6:
            if(FTM0_CH6 == PTD6)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 6) = PORT_PCR_MUX(4);  // PTD6
            }
            else if(FTM0_CH6 == PTA1)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 1) = PORT_PCR_MUX(3);  // PTA1
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH7:
            if(FTM0_CH7 == PTD7)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 7) = PORT_PCR_MUX(4);  // PTD7
            }
            else if(FTM0_CH7 == PTA2)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 2) = PORT_PCR_MUX(3);  // PTA2
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;
        default:
            return;
        }
        break;

    case FTM1:
        SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;                           //使能FTM1时钟
        switch(ch)
        {
        case CH0:
            if(FTM1_CH0 == PTA8)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 8) = PORT_PCR_MUX(3);  // PTA8
            }
            else if(FTM1_CH0 == PTA12)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 12) = PORT_PCR_MUX(3);  // PTA12
            }
            else if(FTM1_CH0 == PTB0)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTB_BASE_PTR, 0) = PORT_PCR_MUX(3);  // PTB0
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;


        case CH1:
            if(FTM1_CH1 == PTA9)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 9) = PORT_PCR_MUX(3);  // PTA9
            }
            else if(FTM1_CH1 == PTA13)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 13) = PORT_PCR_MUX(3);  // PTA13
            }
            else if(FTM1_CH1 == PTB1)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTB_BASE_PTR, 1) = PORT_PCR_MUX(3);  // PTB1
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        default:
            return;
        }
        break;
    case FTM2:
        SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //使能FTM2时钟
        switch(ch)
        {
        case CH0:
            if(FTM2_CH0 == PTA10)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 10) = PORT_PCR_MUX(3);  // PTA10
            }
            else if(FTM2_CH0 == PTB18)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 18) = PORT_PCR_MUX(3);  // PTB18
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        case CH1:
            if(FTM2_CH1 == PTA11)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 11) = PORT_PCR_MUX(3);  // PTA11
            }
            else if(FTM2_CH1 == PTB19)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 19) = PORT_PCR_MUX(3);  // PTB19
            }
            else
            {
                assert_failed(__FILE__, __LINE__);                   //设置管脚有误？
            }
            break;

        default:
            return;
        }
        break;
    default:
        break;
    }
    /******************** 选择输出模式为 边沿对齐PWM *******************/
    //通道状态控制，根据模式来选择 边沿或电平
    FTM_CnSC_REG(FTMx[ftmn], ch) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMx[ftmn], ch) = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    //MSnB:MSnA = 1x
    // ELSnB:ELSnA = 10   High-true pulses  (clear Output on match)
    // ELSnB:ELSnA = 11   Low-true pulses   (set Output on match)
    // Edge-aligned PWM  边沿对齐PWM波   《k16 reference manual.pdf》  P944  或者  《K60P144M100SF2RM.pdf》P1001


    /******************** 配置时钟和分频 ********************/
    FTM_SC_REG(FTMx[ftmn])    = FTM_SC_CPWMS_MASK | FTM_SC_PS(sc_ps) | FTM_SC_CLKS(1);  //没有使能溢出中断
    FTM_CNTIN_REG(FTMx[ftmn]) = 0;                                                      // Channel (n) Value  。设置脉冲宽度：(CnV - CNTIN).
    FTM_MOD_REG(FTMx[ftmn])   = mod;                                                    //Modulo value模数, EPWM的周期为 ：MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMx[ftmn]) = 0;                                                      //Counter Initial Value 计数器初始化值
    FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    FTM_CNT_REG(FTMx[ftmn])   = 0;                                                      //计数器。只有低16位可用

}

/**************************************************************************************
 FTM_PWM_init函数
 初始化FTM的PWM功能并设置频率、占空比。设置通道输出占空比。
 FTMn：模块号（FTM0、  FTM1、  FTM2）
 CHn ：通道号（CH0~CH7）
 freq：频率（单位为Hz）
 duty：占空比
 说明：同一个FTM，各通道的PWM频率是一样的，共3个FTM，即可以输出3个不同频率PWM
***************************************************************************************/
void FTM1_PWM_init_STEER(uint32 freq,uint32 duty)
{      	
    uint32 clk_hz = (bus_clk_khz*1000)>>1;           //bus频率/2=25.
    uint16 mod;
    uint8 sc_ps;
    uint16 cv;
    /*       计算频率设置        */
    mod= (clk_hz>>16 )/freq ;
    for(sc_ps=0;(mod>>sc_ps)>=1;sc_ps++);         //求sc_ps的最小值
    mod=(clk_hz>>sc_ps)/freq;
    cv = (duty*(mod-0+1))/FTM_PRECISON_STEER;           //EPWM的周期 ：MOD - CNTIN + 0x0001   (CNTIN 设为0)
                                                  //脉冲宽度：CnV - CNTIN
                                                  //FTM_PRECISON 是精度
    /******************* 开启时钟 和 复用IO口*******************/
    //注，这里代码虽然长，但编译时会删掉很多没用的部分，不影响速度
     SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;                             //使能FTM1时钟               
     SIM_SCGC5|= SIM_SCGC5_PORTA_MASK;
     PORT_PCR_REG(PORTA_BASE_PTR, 8) = PORT_PCR_MUX(3);  // PTA8
    /******************** 选择输出模式为 边沿对齐PWM *******************/
    //通道状态控制，根据模式来选择 边沿或电平
    FTM_CnSC_REG(FTMx[FTM1],CH0) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMx[FTM1],CH0) = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    /******************** 配置时钟和分频 ********************/
    FTM_SC_REG(FTMx[FTM1])    = FTM_SC_CPWMS_MASK | FTM_SC_PS(sc_ps) | FTM_SC_CLKS(1);  //没有使能溢出中断
    FTM_CNTIN_REG(FTMx[FTM1]) = 0;                                                      // Channel (n) Value  。设置脉冲宽度：(CnV - CNTIN).
    FTM_MOD_REG(FTMx[FTM1])   = mod;                                                    //Modulo value模数, EPWM的周期为 ：MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMx[FTM1]) =0;                                                       //Counter Initial Value 计数器初始化值
    FTM_CnV_REG(FTMx[FTM1],CH0)=cv;
    FTM_CNT_REG(FTMx[FTM1])   =0;                                                       //计数器。只有低16位可用
}
void  FTM1_PWM_init()//初始化FTM1-PTA8-CH0
{
     SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //开启A端口时钟
     PORTA_PCR8&= PORT_PCR_MUX(0);  // PTA8
     PORTA_PCR8|= PORT_PCR_MUX(3);  // PTA8
     PORTA_PCR8|=0x40;
     SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;//开启FTM0模块时钟
     FTM1_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  //FTM1_CH0
     FTM1_CNT = 0; //设置计数初值为0
     FTM1_CNTIN = 0; //设置初始化计数值

      //设置时钟和分频
     FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(3);//总线时钟32分频
     FTM1_MOD = 33700; //设置PWM频率为333hz
     FTM1_C0V = 16850;
}
void FTM0_PWM_init_MOTOR(uint32 freq,uint32 duty1,uint32 duty2)
{      	
    uint32 clk_hz = (bus_clk_khz*1000)>>1;           //bus频率/2
    uint16 mod;
    uint8 sc_ps;
    uint16 cv1,cv2;
    /*       计算频率设置        */
    mod= (clk_hz>>16 )/freq ;
    for(sc_ps=0;(mod>>sc_ps)>=1;sc_ps++);         //求sc_ps的最小值
    mod=(clk_hz>>sc_ps)/freq;

    cv1 = (duty1*(mod-0+1))/FTM_PRECISON_MOTOR;           //EPWM的周期 ：MOD - CNTIN + 0x0001   (CNTIN 设为0)
    cv2 = (duty2*(mod-0+1))/FTM_PRECISON_MOTOR;                                              //脉冲宽度：CnV - CNTIN
                                                  //FTM_PRECISON 是精度
    /******************* 开启时钟 和 复用IO口*******************/
    //注，这里代码虽然长，但编译时会删掉很多没用的部分，不影响速度
        SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;                             //使能FTM0时钟
        SIM_SCGC5|= SIM_SCGC5_PORTC_MASK;
        PORT_PCR_REG(PORTC_BASE_PTR, 1) = PORT_PCR_MUX(4);  // PTC1      
        PORT_PCR_REG(PORTC_BASE_PTR, 2) = PORT_PCR_MUX(4);  // PTC2
    /******************** 选择输出模式为 边沿对齐PWM *******************/
    //通道状态控制，根据模式来选择 边沿或电平
    FTM_CnSC_REG(FTMx[FTM0],CH0) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMx[FTM0],CH1) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMx[FTM0],CH0) = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    FTM_CnSC_REG(FTMx[FTM0],CH1) = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    /******************** 配置时钟和分频 ********************/
    FTM_SC_REG(FTMx[FTM0])    = FTM_SC_CPWMS_MASK | FTM_SC_PS(sc_ps) | FTM_SC_CLKS(1);  //没有使能溢出中断
    FTM_CNTIN_REG(FTMx[FTM0]) = 0;                                                      // Channel (n) Value  。设置脉冲宽度：(CnV - CNTIN). 
    FTM_MOD_REG(FTMx[FTM0])   = mod;                                               //Modulo value模数, EPWM的周期为 ：MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMx[FTM0]) =0;                                               //Counter Initial Value 计数器初始化值
    FTM_CnV_REG(FTMx[FTM0],CH0)=cv1;
    FTM_CnV_REG(FTMx[FTM0],CH1)=cv2;
    FTM_CNT_REG(FTMx[FTM0])   =0;                                                       //计数器。只有低16位可用
  
   
}
/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：FTM_PWM_Duty
*  功能说明：设置通道占空比
*  参数说明：FTMn        模块号（FTM0、  FTM1、  FTM2）
*            CHn         通道号（CH0~CH7）
*            duty        占空比
*  函数返回：无
*  修改时间：2012-2-14
*  备    注：
*************************************************************************/
void FTM_PWM_Duty(FTMn ftmn, CHn ch, u32 duty)
{
    u32 cv;
    u32 mod;
    ASSERT( (ftmn == FTM0) || ( (ftmn == FTM1 || ftmn == FTM2 ) && (ch <= CH1)) ); //检查传递进来的通道是否正确
    ASSERT(duty <= FTM_PRECISON);     //用断言检测 占空比是否合理
    //占空比 = (CnV-CNTIN)/(MOD-CNTIN+1)
    mod = FTM_MOD_REG(FTMx[ftmn]);        //读取 MOD 的值
    cv = (duty * (mod - 0 + 1)) / FTM_PRECISON;
    // 配置FTM通道值
    FTM_CnV_REG(FTMx[ftmn], ch) = cv;

}
/**************************************
 FTM_PWM_Duty函数
 设置通道占空比
**************************************/
void FTM1_PWM_Duty_STEER(uint32 duty)
{
    FTM1_C0V = duty;
}
void FTM0_PWM_Duty_MOTOR(uint32 duty1,uint32 duty2)
{
    uint32 cv1,cv2;
    uint32 mod;  
    mod = FTM_MOD_REG(FTMx[FTM0]);        //读取 MOD 的值
    cv1 = (duty1*(mod-0+1))/FTM_PRECISON_MOTOR;
    cv2 = (duty2*(mod-0+1))/FTM_PRECISON_MOTOR;
    // 配置FTM通道值
    FTM_CnV_REG(FTMx[FTM0],CH0) = cv1;
    FTM_CnV_REG(FTMx[FTM0],CH1) = cv2;
}


/**************************************
 FTM_PWM_freq函数
 设置FTM的频率
**************************************/
void FTM_PWM_freq(FTMn ftmn,uint32 freq)              //设置FTM的频率
{
    uint32 clk_hz = (bus_clk_khz*1000)>>1;             //bus频率/2
    uint32 mod;
    uint8 sc_ps; 

    /*       计算频率设置        */
    mod= (clk_hz>>16 )/freq ;
    for(sc_ps=0;(mod>>sc_ps)>=1;sc_ps++);             //求sc_ps的最小值
    mod=(clk_hz>>sc_ps)/freq;


    /******************** 配置时钟和分频 ********************/
    FTM_SC_REG(FTMx[ftmn])    = FTM_SC_CPWMS_MASK | FTM_SC_PS(sc_ps) | FTM_SC_CLKS(1);  //没有使能溢出中断
    FTM_CNTIN_REG(FTMx[ftmn]) = 0;                                                      // Channel (n) Value  。设置脉冲宽度：(CnV - CNTIN).
    FTM_MOD_REG(FTMx[ftmn])   = mod;                                                    //Modulo value模数, EPWM的周期为 ：MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMx[ftmn]) =0;                                                       //Counter Initial Value 计数器初始化值
    FTM_CNT_REG(FTMx[ftmn])   =0;                                                       //计数器。只有低16位可用
}

/////////////////////////////////以上为PWM输出/////////////////////////////


///////////////////////////////////以下为正交解码/////////////////////////


/***********************************************************************************************************

  函数名称：FTM_Input_init
  功能说明：输入捕捉初始化函数
  参数说明：FTMn        模块号（FTM0、  FTM1、  FTM2）
            CHn         通道号（CH0~CH7）
           Input_cfg   输入捕捉配置（Rising、Falling、Rising_or_Falling）上升沿捕捉、下降沿捕捉、跳变沿捕捉

  备    注：CH0~CH3可以使用过滤器，未添加这功能
**********************************************************************************************************/
void FTM_Input_init(FTMn ftmn,CHn ch,Input_cfg cfg)
{
    

    /******************* 开启时钟 和 复用IO口*******************/
    //注，这里代码虽然长，但真正执行的就几条语句
    switch(ftmn)
    {
    case FTM0:
        SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;                             //使能FTM0时钟
        switch(ch)
        {
        case CH0:
            if(FTM0_CH0==PTC1)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 1) = PORT_PCR_MUX(4);  // PTC1
            }
            else if(FTM0_CH0==PTA3)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 3) = PORT_PCR_MUX(3);  // PTA3
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                    //设置管脚有误？
            }
            break;

        case CH1:
            if(FTM0_CH1==PTC2)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 2) = PORT_PCR_MUX(4);  // PTC2
            }
            else if(FTM0_CH1==PTA4)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 4) = PORT_PCR_MUX(3);  // PTA4
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                   //设置管脚有误？
            }
            break;

        case CH2:
            if(FTM0_CH2==PTC3)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 3) = PORT_PCR_MUX(4);  // PTC3
            }
            else if(FTM0_CH2==PTA5)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 5) = PORT_PCR_MUX(3);  // PTA5
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                    //设置管脚有误？
            }
            break;

        case CH3:
            if(FTM0_CH3==PTC4)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
                PORT_PCR_REG(PORTC_BASE_PTR, 4) = PORT_PCR_MUX(4);  // PTC4
            }
            else if(FTM0_CH3==PTA6)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 6) = PORT_PCR_MUX(3);  // PTA6
            }
            else
            {
                 GPIO_SET(PORTA,17,0) ;                   //设置管脚有误？
            }
            break;

        case CH4:
            if(FTM0_CH4==PTD4)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 4) = PORT_PCR_MUX(4);  // PTD4
            }
            else if(FTM0_CH4==PTA7)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 7) = PORT_PCR_MUX(3);  // PTA7
            }
            else
            {
                 GPIO_SET(PORTA,17,0) ;                   //设置管脚有误？
            }
            break;

        case CH5:
            if(FTM0_CH5==PTD5)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 5) = PORT_PCR_MUX(4);  // PTD5
            }
            else if(FTM0_CH5==PTA0)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 0) = PORT_PCR_MUX(3);  // PTA0
            }
            else
            {
                 GPIO_SET(PORTA,17,0) ;                   //设置管脚有误？
            }
            break;

        case CH6:
            if(FTM0_CH6==PTD6)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 6) = PORT_PCR_MUX(4);  // PTD6
            }
            else if(FTM0_CH6==PTA1)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 1) = PORT_PCR_MUX(3);  // PTA1
            }
            else
            {
                 GPIO_SET(PORTA,17,0) ;                    //设置管脚有误？
            }
            break;

        case CH7:
            if(FTM0_CH7==PTD7)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
                PORT_PCR_REG(PORTD_BASE_PTR, 7) = PORT_PCR_MUX(4);  // PTD7
            }
            else if(FTM0_CH7==PTA2)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 2) = PORT_PCR_MUX(3);  // PTA2
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                   //设置管脚有误？
            }
            break;
        default:
            return;
        }
        break;

  case FTM1:
        SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;                             //使能FTM1时钟
        switch(ch)
        {
        case CH0:
            if(FTM1_CH0==PTA8)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 8) = PORT_PCR_MUX(3);  // PTA8
            }
            else if(FTM1_CH0==PTA12)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 12) = PORT_PCR_MUX(3);  // PTA12
            }
            else if(FTM1_CH0==PTB0)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTB_BASE_PTR, 0) = PORT_PCR_MUX(3);  // PTB0
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                   //设置管脚有误？
            }
            break;


        case CH1:
            if(FTM1_CH1==PTA9)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 9) = PORT_PCR_MUX(3);  // PTA9
            }
            else if(FTM1_CH1==PTA13)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 13) = PORT_PCR_MUX(3);  // PTA13
            }
            else if(FTM1_CH1==PTB1)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTB_BASE_PTR, 1) = PORT_PCR_MUX(3);  // PTB1
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                    //设置管脚有误？
            }
            break;

        default:
            return;
        }
        break;
  case FTM2:
        SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //使能FTM2时钟
        switch(ch)
        {
        case CH0:
            if(FTM2_CH0==PTA10)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 10) = PORT_PCR_MUX(3);  // PTA10
            }
            else if(FTM2_CH0==PTB18)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 18) = PORT_PCR_MUX(3);  // PTB18
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                    //设置管脚有误？
            }
            break;

        case CH1:
            if(FTM2_CH1==PTA11)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 11) = PORT_PCR_MUX(3);  // PTA11
            }
            else if(FTM2_CH1==PTB19)
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
                PORT_PCR_REG(PORTA_BASE_PTR, 19) = PORT_PCR_MUX(3);  // PTB19
            }
            else
            {
                  GPIO_SET(PORTA,17,0) ;                    //设置管脚有误？
            }
            break;

        default:
            return;
        }
        break;
  default:
        break;
    }


    /******************* 设置为输入捕捉功能 *******************/
    switch(cfg)
    {
    case Rising:    //上升沿触发
        FTM_CnSC_REG(FTMx[ftmn],ch) |=  ( FTM_CnSC_ELSA_MASK  | FTM_CnSC_CHIE_MASK );                    //置1
        FTM_CnSC_REG(FTMx[ftmn],ch) &= ~( FTM_CnSC_ELSB_MASK  | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK);  //清0
        break;

    case Falling:   //下降沿触发
        FTM_CnSC_REG(FTMx[ftmn],ch) |= (FTM_CnSC_ELSB_MASK  | FTM_CnSC_CHIE_MASK );                     //置1
        FTM_CnSC_REG(FTMx[ftmn],ch) &= ~( FTM_CnSC_ELSA_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK);  //清0
        break;

    case Rising_or_Falling: //上升沿、下降沿都触发
        FTM_CnSC_REG(FTMx[ftmn],ch) |=  ( FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK  | FTM_CnSC_CHIE_MASK ); //置1
        FTM_CnSC_REG(FTMx[ftmn],ch) &= ~( FTM_CnSC_MSB_MASK  | FTM_CnSC_MSA_MASK);                         //清0
        break;
    }

    FTM_SC_REG(FTMx[ftmn]) = FTM_SC_CLKS(0x1);       //System clock

    FTM_MODE_REG(FTMx[ftmn]) |= FTM_MODE_WPDIS_MASK;
    FTM_COMBINE_REG(FTMx[ftmn])=0;
    FTM_MODE_REG(FTMx[ftmn]) &= ~FTM_MODE_FTMEN_MASK;
    FTM_CNTIN_REG(FTMx[ftmn])=0;

    FTM_STATUS_REG(FTMx[ftmn])=0x00;                 //清中断标志位

    //开启输入捕捉中断
    enable_irq(78-16+ftmn);
}

/*****************************
FTM2模块双路正交脉冲计数；
PTA10、PTA11
初始化FTM2的正交解码功能；
****************************/
void FTM_QUAD_init()
{
    /*开启端口时钟*/
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    /*选择管脚复用功能*/
    PORTA_PCR10 = PORT_PCR_MUX(6);
    PORTA_PCR11 = PORT_PCR_MUX(6);
    /*使能FTM2时钟*/
    SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;
    FTM2_MOD = 65535;                       //最大溢出值    
    FTM2_CNTIN = 0;                         //FTM0计数器初始值为0
    FTM2_QDCTRL|=FTM_QDCTRL_PHBFLTREN_MASK; //打开B的滤波器
    FTM2_QDCTRL|=FTM_QDCTRL_PHAFLTREN_MASK; //打开A的滤波器
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;       //禁止写保护
    FTM2_MODE |= FTM_MODE_FTMEN_MASK;       //FTMEN=1,关闭TPM兼容模式，开启FTM所有功能  
    FTM2_QDCTRL |= FTM_QDCTRL_QUADMODE_MASK; //选定编码模式为A相与B相编码模式~ //改为a计数b方向  
    FTM2_QDCTRL |= FTM_QDCTRL_QUADEN_MASK;   //使能正交解码模式
    FTM2_SC |= FTM_SC_CLKS(3);
    FTM2_CNT=0;
//   FTM2_CONF |=FTM_CONF_BDMMODE(3);

}

void FTM2_QUAD_init(void)
{
    PORTA_PCR10= PORT_PCR_MUX(6); // 设置引脚A10引脚为FTM2_PHA功能
    PORTA_PCR11= PORT_PCR_MUX(6); // 设置引脚A11引脚为FTM2_PHB功能

    PORTA_PCR10|=PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;               //上拉电阻
    PORTA_PCR11|=PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;               //上拉电阻

    SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;//使能FTM2时钟
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;//写保护禁止
    FTM2_MODE |= FTM_MODE_FTMEN_MASK; //FTMEN=1,关闭TPM兼容模式，开启FTM所有功能
    FTM2_CNTIN=0;//FTM0计数器初始值为0
    FTM2_MOD=0xffff;//结束值

    FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(0);
    FTM2_CNT=0;

    FTM2_QDCTRL|=FTM_QDCTRL_QUADEN_MASK;//启用FTM2正交解码模式
    //FTM2_QDCTRL|=FTM_QDCTRL_QUADMODE_MASK;//AB相同时确定方向和计数值
    //FTM2_QDCTRL|=FTM_QDCTRL_PHAPOL_MASK;//A反相
    FTM2_QDCTRL|=FTM_QDCTRL_PHBPOL_MASK;//B反相

    FTM2_MODE |= FTM_MODE_FTMEN_MASK;//FTM2EN=1
}

 
