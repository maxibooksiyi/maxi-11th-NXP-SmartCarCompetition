#ifndef _OLED_H_
#define _OLED_H_


#define byte uint8
#define word uint16

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1

byte longqiu96x64[768];
void LCD_Init(void);
void LCD_CLS(void);
void LCD_P6x8Str(byte x,byte y,byte ch[]);
void LCD_P8x16Str(byte x,byte y,byte ch[]);
void LCD_P14x16Str(byte x,byte y,byte ch[]);
void LCD_Print(byte x, byte y, byte ch[]);
void LCD_PutPixel(byte x,byte y);
void Draw_LQLogo(void);
void Draw_LibLogo(void);
void LCD_BL(uint8 x, uint8 y, uint16 z);
void blprint(uint8 x,uint8 y,uint8 ax);   //写入字模
void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
void LCD_Fill(byte dat);
void LCD_CCD_SHOW(uint8 a,unsigned char * CCD); 

void write_6_8_char(byte x,byte y,byte ucData) ;
void write_6_8_string(byte x,byte y,byte ch[]);
void write_6_8_number(unsigned char x,unsigned char y, float number);
#endif 

 

