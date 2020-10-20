/******************** 
文件名       ：OLED.h
描述         ：OLED头文件
*********************************/	

#ifndef _OLED_H_
#define _OLED_H_


#define byte uint8
#define word uint16
#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1

void LCD_WrDat(byte data);
void LCD_WrCmd(byte cmd);
void LCD_Set_Pos(byte x, byte y);//画点
void LCD_Init(void);
void LCD_CLS(void);
void LCD_clear_L(unsigned char x,unsigned char y);
void LCD_Print(byte x, byte y, byte ch[]);
void LCD_P6x8Str(byte x,byte y,byte ch[]);
void LCD_P8x16Str(byte x,byte y,byte ch[]);
void LCD_write_char(uint8 X, uint8 Y,uint16 c);
void LCD_Show_Number (uint8 X,uint8 Y,int16 number);
void LCD_Show_float (uint8 X,uint8 Y,float number);
void LCD_8x16_number (uint8 X,uint8 Y,float number);
void LCD_6x8_number  (uint8 X,uint8 Y,float number);
void write_6_8_char(byte x,byte y,byte ch);
void write_6_8_string(byte x,byte y,byte ch[]);
void write_6_8_number(unsigned char x,unsigned char y, float number);
void write_8_16_char(byte x,byte y,byte ch);
void write_8_16_string(byte x,byte y,byte ch[]);
void write_8_16_number(unsigned char x,unsigned char y, float number);
void LCD_Draw(byte x1,byte y1,byte m);
void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]);
void MAIN_ConvertData(uint16 *pInputImageData, uint8 *pOutputImageBuf);
void LED_PrintImage(uint8 *pucTable, uint16 usRowNum, uint16 usColumnNum);
void LCD_PrintImage(uint8 *pucTable, uint16 usRowNum, uint16 usColumnNum);

#endif

