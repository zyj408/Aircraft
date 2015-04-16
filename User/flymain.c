#include  <includes.h>
#include  "IMU.h"

float ypr[3]; // yaw pitch roll
uint16_t Math_hz = 0;




void FlyMain(void)
{
	MPU6050_DataDeal();  //处理MPU6050获取的数据得到：MPU6050_H结构体
	HMC5883L_DataDeal();	//处理HMC5883L获取的数得到：HMC5883L_H结构体
	
	IMU_getYawPitchRoll(ypr);
	Math_hz++;
	
	control(ypr[0], ypr[1], ypr[2]);
}
