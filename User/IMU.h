#ifndef __IMU_H
#define __IMU_H


extern float angleAx_temp, angleAy_temp, angleAz_temp;

void IMU_init(void);
void IMU_getYawPitchRoll(float* angles);
void IMU_getQ(float * q);
float invSqrt(float x);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_getValues(float * values);



#endif
