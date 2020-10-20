#ifndef _KALMAN_H
#define _KALMAN_H

extern int16   Angle_X,Angle_Y,Angle_Z;
extern int32   Accel_X,Accel_Y,Accel_Z;
extern int32   Gyro_X,Gyro_Y,Gyro_Z;
extern int16   Gyro_Z_Offest,Gyro_Y_Offest,Gyro_X_Offest;
extern int16   jiaodu_x,jiaodu_y,jiaodu_z;

void CarVoltageGet(void);
void Offest_Init(void);
void jiaodu_get(void);
 
#endif 
