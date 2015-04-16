#include  <includes.h>
#include "flymain.h"
#include "IMU.h"




void AppSampleTask(void *p_arg)
{
	OS_ERR err;
	
	(void)p_arg;
/* 监测MPU6050信息 */
	if(i2c_CheckDevice(MPU6050_SLAVE_ADDRESS) == 0)
	{
		printf("MPU-6050 Ok (0x%02X)\r\n", MPU6050_SLAVE_ADDRESS);
		MPU6050Flag |= NORMAL;
	}
	else
	{
		printf("MPU-6050 Err (0x%02X)\r\n", MPU6050_SLAVE_ADDRESS);
		MPU6050Flag &= (~NORMAL);
	}
/* 初始化MPU6050 */	
	bsp_InitMPU6050();  //初始化MPU6050
	
	
/* 监测HMC5883L信息 */	
	if (i2c_CheckDevice(HMC5883L_SLAVE_ADDRESS) == 0)
	{
		printf("HMC5883L Ok (0x%02X)\r\n", HMC5883L_SLAVE_ADDRESS);
		HMC5883LFlag |= NORMAL;
	}
	else
	{
		printf("HMC5883L Err (0x%02X)\r\n",HMC5883L_SLAVE_ADDRESS);
		HMC5883LFlag &= (~NORMAL);
	}
/* 初始化HMC5883L */		
	bsp_InitHMC5883L();
	
	IMU_init();
	
	OSTimeDlyHMSM((CPU_INT16U) 0u,
                (CPU_INT16U) 0u,
                (CPU_INT16U) 10u,
                (CPU_INT32U) 0u,
                (OS_OPT    ) OS_OPT_TIME_HMSM_STRICT,
                (OS_ERR   *)&err);
	
	while(1)
	{
		/* 传感器采集数据 */
		MPU6050_ReadData();
		HMC5883L_ReadData();
		
		
		if(HMC5883LFlag & CALI_MODE)
		{
			UsartSendData(0x25);	  //HMC5883校准
		}
		if(MPU6050Flag & CALI_MODE)
		{
			UsartSendData(0x45);
		}
		
		if((HMC5883LFlag & NORMAL) && (MPU6050Flag & NORMAL))
		{
			FlyMain();
		}
		
		
	OSTimeDlyHMSM((CPU_INT16U) 0u,
                (CPU_INT16U) 0u,
                (CPU_INT16U) 0u,
                (CPU_INT32U) 3u,
                (OS_OPT    ) OS_OPT_TIME_HMSM_STRICT,
                (OS_ERR   *)&err);		
	}
}

/* PWM输出任务 */
uint16_t SetPwmValue[4] = {0,0,0,0};
uint16_t CurrentPwmValue[4];
uint16_t SetGeneralReinforce = 0;
uint16_t CurrentGeneralReinforce;
uint8_t  SetPwmDirection[4] = {0,0,0,0};
uint8_t  CurrentPwmDirection[4];
void AppOutPutTask(void *p_arg)
{
	uint8_t index;
	uint32_t PwmTemp;
	(void)p_arg;
	
	Mem_Copy(CurrentPwmValue, SetPwmValue, sizeof(SetPwmValue));
	Mem_Copy(CurrentPwmDirection, SetPwmDirection, sizeof(SetPwmDirection));
	CurrentGeneralReinforce = SetGeneralReinforce;
	
	while(1)
	{
		if(CurrentGeneralReinforce != SetGeneralReinforce)
		{
			if(SetGeneralReinforce > 100)    //防止增益超出范围    
				SetGeneralReinforce = 100;             
		
			for(index=0; index<4; index++)
			{
				PwmTemp = (uint32_t)(CurrentPwmValue[index] * SetGeneralReinforce / 100);    //计算出当前的PWM实际值
				if(PwmTemp > 100)    //防止PWM占空比超出范围
					PwmTemp = 100;
				
				bsp_SetPWMDutyCycle(index+1, PwmTemp);				
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
				
				bsp_SetPWMDutyCycle(index+1, PwmTemp);
				CurrentPwmValue[index] = SetPwmValue[index];
			}
			
			if(SetPwmDirection[index] != CurrentPwmDirection[index])
			{
				
				
				SetPwmDirection[index] = CurrentPwmDirection[index];
			}
		}
		
		BSP_OS_TimeDlyMs(5);
	}
}

