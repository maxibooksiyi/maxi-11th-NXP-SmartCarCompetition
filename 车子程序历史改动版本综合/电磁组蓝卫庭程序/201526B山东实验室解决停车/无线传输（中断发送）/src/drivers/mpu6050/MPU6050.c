#include "MPU6050.h"
#include "OLED.h"
#include  "i2c.h"
volatile int16 MPU6050_ACCEL_XOUT_DATA=0;
volatile int16 MPU6050_ACCEL_YOUT_DATA=0;
volatile int16 MPU6050_ACCEL_ZOUT_DATA=0;
volatile int16 MPU6050_TEMP_OUT_DATA=0;
volatile int16 MPU6050_GYRO_XOUT_DATA=0;
volatile int16 MPU6050_GYRO_YOUT_DATA=0;
volatile int16 MPU6050_GYRO_ZOUT_DATA=0;

void MPU6050_Init(void)
{
  //i2c_init(MPU6050_I2C_Moudle,0,0,4);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_PWR_MGMT_1, 0x00);
  Delay(150);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_SMPLRT_DIV, 0x07);
  Delay(150);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_CONFIG, 0x06);//��ͨ�˲�������5Hz
  Delay(150);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_GYRO_CONFIG, 0x18);//���Լ�   ��500 ��/s
  Delay(150);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG, 0x01);//���Լ�  ��4g
  Delay(150);
}

int16 MPU6050_GetDoubleData(uint8 Addr)
{
  uint16 data=0x0000;
  data=i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, Addr);
  data=(uint16)((data<<8)&0xff00);
  data+=i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, Addr+1);
  return (int16)data;//�ϳ����ݣ�Ϊ�з���������
}
void MPU6050_GetData(void)
{
  MPU6050_ACCEL_XOUT_DATA=((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_ACCEL_XOUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_ACCEL_XOUT+1);
  MPU6050_ACCEL_YOUT_DATA=((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_ACCEL_YOUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_ACCEL_YOUT+1);
  MPU6050_ACCEL_ZOUT_DATA=((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT+1);
  MPU6050_TEMP_OUT_DATA  =((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_TEMP_OUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_TEMP_OUT+1);
  MPU6050_GYRO_XOUT_DATA =((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_GYRO_XOUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_GYRO_XOUT+1);
  MPU6050_GYRO_YOUT_DATA =((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_GYRO_YOUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_GYRO_YOUT+1);
  MPU6050_GYRO_ZOUT_DATA =((int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_GYRO_ZOUT)<<8)
                          |(int16)i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, MPU6050_GYRO_ZOUT+1);
    write_6_8_string(0,0,"Ax:");
    LCD_Show_Number(24,0,MPU6050_ACCEL_XOUT_DATA);
    write_6_8_string(0,1,"Ay:");
    LCD_Show_Number(24,1,MPU6050_ACCEL_YOUT_DATA);
}
void Delay(uint16 i)
{
  uint8 j;
  for(;i>1;i--){ for(j=0;j<100;j++); }
}  