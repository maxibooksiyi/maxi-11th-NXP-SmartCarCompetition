#ifndef LDC1000_CMD_H_
#define LDC1000_CMD_H_
#define uchar uint8
#define NM 7



//FLOAT LDC COMMANDS
#define LDC1000_CMD_REVID               0x00
#define LDC1000_CMD_RPMAX 	        0x01
#define LDC1000_CMD_RPMIN 	        0x02
#define LDC1000_CMD_SENSORFREQ 	0x03               //谐振频率
#define LDC1000_CMD_LDCCONFIG 	0x04
#define LDC1000_CMD_CLKCONFIG 	0x05
#define LDC1000_CMD_THRESHILSB 	0x06
#define LDC1000_CMD_THRESHIMSB 	0x07
#define LDC1000_CMD_THRESLOLSB 	0x08
#define LDC1000_CMD_THRESLOMSB 	0x09
#define LDC1000_CMD_INTCONFIG 	0x0A
#define LDC1000_CMD_PWRCONFIG 	0x0B
#define LDC1000_CMD_STATUS	0x20
#define LDC1000_CMD_PROXLSB 	0x21
#define LDC1000_CMD_PROXMSB 	0x22
#define LDC1000_CMD_FREQCTRLSB	0x23
#define LDC1000_CMD_FREQCTRMID	0x24
#define LDC1000_CMD_FREQCTRMSB	0x25

//FLOAT LDC BITMASKS
#define LDC1000_BIT_AMPLITUDE    0x18
#define LDC1000_BIT_RESPTIME     0x07
#define LDC1000_BIT_CLKSEL       0x02
#define LDC1000_BIT_CLKPD        0x01
#define LDC1000_BIT_INTMODE      0x07
#define LDC1000_BIT_PWRMODE      0x01
#define LDC1000_BIT_STATUSOSC    0x80
#define LDC1000_BIT_STATUSDRDYB  0x40
#define LDC1000_BIT_STATUSWAKEUP 0x20
#define LDC1000_BIT_STATUSCOMP   0x10


/**********************************************************SPI 管脚定义*******************************************************/
/***************经测试各家核心板IO驱动能力有区别建议大家不要使用下面注释掉的方式，使用gpio_get（）方式数据会更稳定************/



#define MISO1   gpio_get(PTE1)//(GPIO_PDIR_REG(GPIOX_BASE(PTD0)) >> PTn(PTD0 )) & 0x01  
#define MOSI_H1  gpio_set(PTE3,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD1))  |= (1 << PTn(PTD1))
#define MOSI_L1  gpio_set(PTE3,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD1)) &= ~(1 << PTn(PTD1))
#define CSN_H1   gpio_set(PTE5,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD2))  |= (1 << PTn(PTD2))
#define CSN_L1   gpio_set(PTE5,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD2)) &= ~(1 << PTn(PTD2))
#define SCK_H1   gpio_set(PTE7,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD3))  |= (1 << PTn(PTD3))
#define SCK_L1   gpio_set(PTE7,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD3)) &= ~(1 << PTn(PTD3))

#define MISO2   gpio_get(PTE8)//(GPIO_PDIR_REG(GPIOX_BASE(PTD0)) >> PTn(PTD0 )) & 0x01  
#define MOSI_H2  gpio_set(PTE9,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD1))  |= (1 << PTn(PTD1))
#define MOSI_L2  gpio_set(PTE9,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD1)) &= ~(1 << PTn(PTD1))
#define CSN_H2   gpio_set(PTE10,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD2))  |= (1 << PTn(PTD2))
#define CSN_L2   gpio_set(PTE10,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD2)) &= ~(1 << PTn(PTD2))
#define SCK_H2   gpio_set(PTE11,1)//GPIO_PDOR_REG(GPIOX_BASE(PTD3))  |= (1 << PTn(PTD3))
#define SCK_L2   gpio_set(PTE11,0)//GPIO_PDOR_REG(GPIOX_BASE(PTD3)) &= ~(1 << PTn(PTD3))


void FLOAT_LDC_init1();
int ldc_read_avr1();
long int filter1();
void FLOAT_SPI_init1();
uchar FLOAT_SPI_RW1(uchar rwdata);
uchar FLOAT_Singal_SPI_Read1(uchar reg);
void FLOAT_Singal_SPI_Write1(uchar reg,uchar wdata);
void FLOAT_SPI_Read_Buf1(uchar reg, uchar *pBuf, uchar len);

void FLOAT_LDC_init2();
int ldc_read_avr2();
long int filter2();
void FLOAT_SPI_init2();
uchar FLOAT_SPI_RW2(uchar rwdata);
uchar FLOAT_Singal_SPI_Read2(uchar reg);
void FLOAT_Singal_SPI_Write2(uchar reg,uchar wdata);
void FLOAT_SPI_Read_Buf2(uchar reg, uchar *pBuf, uchar len);




#endif
