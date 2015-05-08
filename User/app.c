#include  <includes.h>
#include "flymain.h"
#include "IMU.h"


extern enum WIFI_STATUS WifiStatus;
uint8_t WIFI_Period;
uint32_t CaliTime = 0;
/* PWM输出任务 */
uint16_t SetPwmValue[4] = {50, 50, 50, 50};
uint16_t SetGeneralReinforce = 0;
uint8_t  SetPwmDirection[4] = {0,0,0,0};

uint16_t CurrentPwmValue[4] = {0,0,0,0};
uint16_t CurrentGeneralReinforce = 0;
uint8_t  CurrentPwmDirection[4] = {0,0,0,0};


uint8_t index;
uint32_t PwmTemp;


char send_data_buf[200];
extern float Angle_X_Final; //X最终倾斜角度
extern float Angle_Y_Final; //Y最终倾斜角度
void AppSampleTask(void *p_arg)
{
	OS_ERR err;
	
	(void)p_arg;
/* 监测MPU6050信息 */
	if(i2c_CheckDevice(MPU6050_SLAVE_ADDRESS) == 0)
	{
		//printf("MPU-6050 Ok (0x%02X)\r\n", MPU6050_SLAVE_ADDRESS);
		MPU6050Flag |= NORMAL;
	}
	else
	{
		//printf("MPU-6050 Err (0x%02X)\r\n", MPU6050_SLAVE_ADDRESS);
		MPU6050Flag &= (~NORMAL);
	}
/* 初始化MPU6050 */	
	bsp_InitMPU6050();  //初始化MPU6050
	
	
/* 监测HMC5883L信息 */	
	if (i2c_CheckDevice(HMC5883L_SLAVE_ADDRESS) == 0)
	{
		//printf("HMC5883L Ok (0x%02X)\r\n", HMC5883L_SLAVE_ADDRESS);
		HMC5883LFlag |= NORMAL;
	}
	else
	{
		//printf("HMC5883L Err (0x%02X)\r\n",HMC5883L_SLAVE_ADDRESS);
		HMC5883LFlag &= (~NORMAL);
	}
/* 初始化HMC5883L */		
	bsp_InitHMC5883L();

	
	while(1)
	{
		/* 传感器采集数据 */
		MPU6050_ReadData();
		HMC5883L_ReadData();
		MPU6050_DataDeal();  //处理MPU6050获取的数据得到：MPU6050_H结构体
		HMC5883L_DataDeal();	//处理HMC5883L获取的数得到：HMC5883L_H结构体
		
		if(HMC5883LFlag & CALI_MODE)
		{
			
		}
		if(MPU6050Flag & CALI_MODE)
		{
			if(!(MPU6050FlagOld & CALI_MODE))  //进入校验模式
			{
				CaliTime = 200;
				
				MPU6050_H.Accel_X_Offset = g_tMPU6050.Accel_X;
				MPU6050_H.Accel_Y_Offset = g_tMPU6050.Accel_Y;
				MPU6050_H.Accel_Z_Offset = g_tMPU6050.Accel_Z  - 65536 / 4;
			
				MPU6050_H.GYRO_X_Offset = g_tMPU6050.GYRO_X;
				MPU6050_H.GYRO_Y_Offset = g_tMPU6050.GYRO_Y;
				MPU6050_H.GYRO_Z_Offset = g_tMPU6050.GYRO_Z;
				
			}
			if(CaliTime == 0)
			{			
				MPU6050Flag &= ~(CALI_MODE);
			}
			
			MPU6050_H.Accel_X_Offset = (float)(g_tMPU6050.Accel_X + MPU6050_H.Accel_X_Offset) / 2;
			MPU6050_H.Accel_Y_Offset = (float)(g_tMPU6050.Accel_Y + MPU6050_H.Accel_Y_Offset) / 2;
			MPU6050_H.Accel_Z_Offset = (float)(g_tMPU6050.Accel_Z + MPU6050_H.Accel_Z_Offset - 65536 / 4) / 2;
			
			MPU6050_H.GYRO_X_Offset = (float)(g_tMPU6050.GYRO_X + MPU6050_H.GYRO_X_Offset) / 2;
			MPU6050_H.GYRO_Y_Offset = (float)(g_tMPU6050.GYRO_Y + MPU6050_H.GYRO_Y_Offset) / 2;
			MPU6050_H.GYRO_Z_Offset = (float)(g_tMPU6050.GYRO_Z + MPU6050_H.GYRO_Z_Offset) / 2;
			
			CaliTime--;
			
			if(CaliTime > 200)
				CaliTime = 200;
		}
		
		if((HMC5883LFlag & NORMAL) && (MPU6050Flag & NORMAL))
		{
			FlyMain();
		}
		
	if(WifiStatus == CONNECTING)	
	{
		if((WIFI_Period++) % 1000000 == 0)
		{
			Mem_Clr(send_data_buf, sizeof(send_data_buf));
			sprintf(send_data_buf, "sy:s1:d:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%fd:%f",angleAx_temp, angleAy_temp, angleAz_temp,MPU6050_H.Accel_X,MPU6050_H.Accel_Y,MPU6050_H.Accel_Z, \
			MPU6050_H.GYRO_X,MPU6050_H.GYRO_Y,MPU6050_H.GYRO_Z,HMC5883L_H.X,HMC5883L_H.Y,HMC5883L_H.Z,MPU6050_H.Accel_X_Offset,MPU6050_H.Accel_Y_Offset,MPU6050_H.Accel_Z_Offset, \
			MPU6050_H.GYRO_X_Offset,MPU6050_H.GYRO_Y_Offset,MPU6050_H.GYRO_Z_Offset);
			ESP8266_send_data(send_data_buf);
		}
			
			if(CurrentGeneralReinforce != SetGeneralReinforce)
			{
				if(SetGeneralReinforce > 100)    //防止增益超出范围    
					SetGeneralReinforce = 100;             
			
				for(index=0; index<4; index++)
				{
					PwmTemp = (uint32_t)(CurrentPwmValue[index] * SetGeneralReinforce / 100);    //计算出当前的PWM实际值
					if(PwmTemp > 100)    //防止PWM占空比超出范围
						PwmTemp = 100;
					
					bsp_SetPWMDutyCycle(PwmTemp, index+1);				
				}
				
				CurrentGeneralReinforce = SetGeneralReinforce;  //增益赋值
			}
		
			for(index=0; index<4; index++)
			{
				if(CurrentPwmValue[index] != SetPwmValue[index])
				{
					if(SetPwmValue[index] > 100)    //防止增益超出范围    
						SetPwmValue[index] = 100;   				
					
					PwmTemp = (uint32_t)(SetPwmValue[index] * CurrentGeneralReinforce / 100);    //计算出当前的PWM实际值
					if(PwmTemp > 100)    //防止PWM占空比超出范围
						PwmTemp = 100;
					
					bsp_SetPWMDutyCycle(PwmTemp, index+1);
					CurrentPwmValue[index] = SetPwmValue[index];
				}
				
				if(SetPwmDirection[index] != CurrentPwmDirection[index])
				{
					
					
					SetPwmDirection[index] = CurrentPwmDirection[index];
				}
			}
	}

	MPU6050FlagOld = MPU6050Flag;
	
	
	
	OSTimeDlyHMSM((CPU_INT16U) 0u,
                (CPU_INT16U) 0u,
                (CPU_INT16U) 0u,
                (CPU_INT32U) 2u,
                (OS_OPT    ) OS_OPT_TIME_HMSM_STRICT,
                (OS_ERR   *)&err);		
	//bsp_LedToggle(1);
	
	
	}
}


