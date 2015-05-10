#include <includes.h>




void AppCameraTask(void *p_arg)
{
	uint16_t id = 0;
	(void)p_arg;
	if(i2c_CheckDevice2(OV7670_SLAVE_ADDRESS) == 0)
	{
		printf("OV7670 Ok (0x%02X)\r\n", OV7670_SLAVE_ADDRESS);
		id = OV_ReadID();
		printf("OV7670 ID = (0x%04X)\r\n", id);
	}
	else
	{
		id = OV_ReadID();
		printf("OV7670 Err (0x%02X)\r\n",OV7670_SLAVE_ADDRESS);
	}
	
	
	while(1)
	{
		
	}
}
