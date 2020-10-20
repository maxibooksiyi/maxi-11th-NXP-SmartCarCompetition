#ifndef LDC1000_CMD_H_
#define LDC1000_CMD_H_
#define uchar uint8

//FLOAT LDC COMMANDS
#define LDC1000_CMD_REVID               0x00
#define LDC1000_CMD_RPMAX 	        0x01
#define LDC1000_CMD_RPMIN 	        0x02
#define LDC1000_CMD_SENSORFREQ 	0x03               //Ð³ÕñÆµÂÊ
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

#define MISO_1   (GPIO_PDIR_REG(GPIOX_BASE(PTB0)) >> PTn(PTB0 )) & 0x01
#define MOSI_H_1  GPIO_PDOR_REG(GPIOX_BASE(PTB1))  |= (1 << PTn(PTB1))
#define MOSI_L_1  GPIO_PDOR_REG(GPIOX_BASE(PTB1)) &= ~(1 << PTn(PTB1))
#define CSN_H_1  GPIO_PDOR_REG(GPIOX_BASE(PTB2))  |= (1 << PTn(PTB2))
#define CSN_L_1  GPIO_PDOR_REG(GPIOX_BASE(PTB2)) &= ~(1 << PTn(PTB2))
#define SCK_H_1  GPIO_PDOR_REG(GPIOX_BASE(PTB3))  |= (1 << PTn(PTB3))
#define SCK_L_1  GPIO_PDOR_REG(GPIOX_BASE(PTB3)) &= ~(1 << PTn(PTB3))


#define MISO_2   (GPIO_PDIR_REG(GPIOX_BASE(PTB4)) >> PTn(PTB4 )) & 0x01
#define MOSI_H_2  GPIO_PDOR_REG(GPIOX_BASE(PTB5))  |= (1 << PTn(PTB5))
#define MOSI_L_2  GPIO_PDOR_REG(GPIOX_BASE(PTB5)) &= ~(1 << PTn(PTB5))
#define CSN_H_2  GPIO_PDOR_REG(GPIOX_BASE(PTB6))  |= (1 << PTn(PTB6))
#define CSN_L_2  GPIO_PDOR_REG(GPIOX_BASE(PTB6)) &= ~(1 << PTn(PTB6))
#define SCK_H_2  GPIO_PDOR_REG(GPIOX_BASE(PTB7))  |= (1 << PTn(PTB7))
#define SCK_L_2  GPIO_PDOR_REG(GPIOX_BASE(PTB7)) &= ~(1 << PTn(PTB7))


void FLOAT_LDC_init_1();
int ldc_read_avr_1();
long int filter_1();
void FLOAT_SPI_init_1();
uchar FLOAT_SPI_RW_1(uchar rwdata);
uchar FLOAT_Singal_SPI_Read_1(uchar reg);
void FLOAT_Singal_SPI_Write_1(uchar reg,uchar wdata);
void FLOAT_SPI_Read_Buf_1(uchar reg, uchar *pBuf, uchar len);


void FLOAT_LDC_init_2();
int ldc_read_avr_2();
long int filter_2();
void FLOAT_SPI_init_2();
uchar FLOAT_SPI_RW_2(uchar rwdata);
uchar FLOAT_Singal_SPI_Read_2(uchar reg);
void FLOAT_Singal_SPI_Write_2(uchar reg,uchar wdata);
void FLOAT_SPI_Read_Buf_2(uchar reg, uchar *pBuf, uchar len);

#endif
