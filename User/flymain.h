#ifndef _FLYMAIN_H
#define _FLYMAIN_H

#include "stm32f4xx.h"

void FlyMain(void);

#define CALI_MODE 0x10
#define NORMAL    0x01




struct hmc5883l_correct_info
{
	float Mag_X_k;
	float Mag_X_b;
	
	float Mag_Y_k;
	float Mag_Y_b;
	
	float Mag_Z_k;
	float Mag_Z_b;
};

struct hmc5883l_info
{
	float X;
	float Y;
	float Z;
	
  float AngPitch;
	float AngRoll;
};


struct mpu6050_info
{
	float Accel_X;
	float Accel_X_Offset;
	float Accel_Y;
	float Accel_Y_Offset;
	float Accel_Z;
	float Accel_Z_Offset;
	float GYRO_X;
	float GYRO_X_Offset;
	float GYRO_Y;
	float GYRO_Y_Offset;
	float GYRO_Z;
	float GYRO_Z_Offset;
};



#endif
