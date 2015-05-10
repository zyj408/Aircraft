#include <includes.h>

#define WIFI_TIMEOUT 20
#define COM_ESP8266	COM1		/* 选择串口 */
#define ESP8266_TMR_ID 3
#define AT_CR		'\r'
#define AT_LF		'\n'

enum WIFI_STATUS WifiStatus;  //WIFI连接状态

char ESP8266_rx_data[256] = {0};
char ESP8266_tx_data[256] = {0};
char ESP8266_current_mode = 0;
char *ptr_temp = NULL;

uint8_t ESP8266_connect_flag = 0;
struct ip_infomation ESP8266_connect[4];
extern BSP_OS_SEM wifi_send_sem;

OS_ERR err;
uint8_t ESP8266_WaitResponse(char *_pAckStr, uint16_t _usTimeOut)
{
	uint8_t ucData;
	uint8_t ucRxBuf[256];
	uint16_t pos = 0;
	uint32_t len;
	uint8_t ret;

	len = strlen(_pAckStr);
	if (len > 255)
	{
		return 0;
	}

	/* _usTimeOut == 0 表示无限等待 */
	if (_usTimeOut > 0)
	{
		bsp_StartTimer(ESP8266_TMR_ID, _usTimeOut);		/* 使用软件定时器3，作为超时控制 */
	}
	while (1)
	{
		bsp_Idle();				/* CPU空闲执行的操作， 见 bsp.c 和 bsp.h 文件 */

		if (_usTimeOut > 0)
		{
			if (bsp_CheckTimer(ESP8266_TMR_ID))
			{
				ret = 0;	/* 超时 */
				break;
			}
		}

		if (comGetChar(COM_ESP8266, &ucData))
		{
		//	ESP8266_PrintRxData(ucData);		/* 将接收到数据打印到调试串口1 */

			if (ucData == '\n')
			{
				if (pos > 0)	/* 第2次收到回车换行 */
				{
					if (memcmp(ucRxBuf, _pAckStr,  len) == 0)
					{
						ret = 1;	/* 收到指定的应答数据，返回成功 */
						break;
					}
					else
					{
						pos = 0;
					}
				}
				else
				{
					pos = 0;
				}
			}
			else
			{
				if (pos < sizeof(ucRxBuf))
				{
					/* 只保存可见字符 */
					if (ucData >= ' ')
					{
						ucRxBuf[pos++] = ucData;
					}
				}
			}
		}
	}
	return ret;
}



uint16_t ESP8266_ReadResponse(char *_pBuf, uint16_t _usBufSize, uint16_t _usTimeOut)
{
	uint8_t ucData;
	uint16_t pos = 0;
	uint8_t ret;
	uint8_t status = 0;		/* 接收状态 */

	/* _usTimeOut == 0 表示无限等待 */
	if (_usTimeOut > 0)
	{
		bsp_StartTimer(ESP8266_TMR_ID, _usTimeOut);		/* 使用软件定时器作为超时控制 */
	}
	while (1)
	{
		bsp_Idle();				/* CPU空闲执行的操作， 见 bsp.c 和 bsp.h 文件 */

		if (status == 2)		/* 正在接收有效应答阶段，通过字符间超时判断数据接收完毕 */
		{
			if (bsp_CheckTimer(ESP8266_TMR_ID))
			{
				_pBuf[pos]	 = 0;	/* 结尾加0， 便于函数调用者识别字符串结束 */
				ret = pos;		/* 成功。 返回数据长度 */
				break;
			}
		}
		else
		{
			if (_usTimeOut > 0)
			{
				if (bsp_CheckTimer(ESP8266_TMR_ID))
				{
					ret = 0;	/* 超时 */
					break;
				}
			}
		}

		if (comGetChar(COM_ESP8266, &ucData))
		{
			switch (status)
			{
				case 0:			/* 首字符 */
					if (ucData == AT_CR)		/* 如果首字符是回车，表示 AT命令不会显 */
					{
						_pBuf[pos++] = ucData;		/* 保存接收到的数据 */
						status = 2;	 /* 认为收到模块应答结果 */
					}
					else	/* 首字符是 A 表示 AT命令回显 */
					{
						status = 1;	 /* 这是主机发送的AT命令字符串，不保存应答数据，直到遇到 CR字符 */
					}
					break;

				case 1:			/* AT命令回显阶段, 不保存数据. 继续等待 */
					if (ucData == AT_CR)
					{
						status = 2;
					}
					break;

				case 2:			/* 开始接收模块应答结果 */
					/* 只要收到模块的应答字符，则采用字符间超时判断结束，此时命令总超时不起作用 */
					bsp_StartTimer(ESP8266_TMR_ID, 5);
					if (pos < _usBufSize - 1)
					{
						_pBuf[pos++] = ucData;		/* 保存接收到的数据 */
					}
					break;
			}
		}
	}
	return ret;
}



void ESP8266_SendAT(char *_Cmd)
{
	comSendBuf(COM1, (uint8_t *)_Cmd, strlen(_Cmd));
	comSendBuf(COM1, "\r\n", 2);
}

void ESP8266_Reset(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	Mem_Set(&GPIO_InitStructure, 0x00, sizeof(GPIO_InitStructure));
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 配置GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	BSP_OS_TimeDlyMs(100);
	GPIO_SetBits(GPIOA, GPIO_Pin_11);
	BSP_OS_TimeDlyMs(100);

	
	
	
	comClearRxFifo(COM_ESP8266);
}



uint32_t wifi_rx_cnt = 0;
char data_temp[10] = {0};
extern uint16_t SetPwmValue[4];
extern uint8_t  SetPwmDirection[4];

void AppCommTask(void *p_arg)
{
	uint8_t i;
	WifiStatus = IDLE;

	(void)p_arg;
	
	while(1)
	{
		switch (WifiStatus)
		{
			case IDLE:
				ESP8266_SendAT("AT+RST"); //发送“AT+RST”
				if(ESP8266_WaitResponse("OK", 2000))
					WifiStatus = AP_BEGIN;
					
			break;
			case AP_BEGIN:
				if(ESP8266_connect_flag == 0)
				{
					while(1)  //配置WIFI至AP模式
					{
						Mem_Clr(ESP8266_rx_data, sizeof(ESP8266_rx_data));
						ESP8266_SendAT("AT+CWMODE=2"); //发送“AT+RST?
						ESP8266_ReadResponse(ESP8266_rx_data, sizeof(ESP8266_rx_data), 2000);
						if(strstr(ESP8266_rx_data, "no change") != NULL) //查询当前WIFI模式是否为AP模式
							break; 
						else if(strstr(ESP8266_rx_data, "OK") != NULL)  //转换完成
							break;
					}
					while(1)  //重新初始化WIFI模块
					{
						ESP8266_SendAT("AT+RST"); //发送“AT+RST”
						if(ESP8266_WaitResponse("OK", 2000))
						{
							ESP8266_current_mode = 2;
							break;
						}
					}
					while(1)  //配置AP节点
					{
						ESP8266_SendAT("AT+CWSAP=\"XiaoSiZhou\",\"123456789\",1,3"); //发送“AT+RST”
						if(ESP8266_WaitResponse("OK", 2000))
						{
							break;
						}
					}					
					while(1)  //查询多节点连接
					{
						Mem_Clr(ESP8266_rx_data, sizeof(ESP8266_rx_data));
						ESP8266_SendAT("AT+CIPMUX=1"); //发送“AT+CIPMUX=1”
						if(ESP8266_WaitResponse("OK", 2000))
						{
							ESP8266_SendAT("AT+CIPMUX?");
							if(ESP8266_WaitResponse("+CIPMUX:1", 2000))
							{
								break;
							}
						}
					}	
					while(1) //开启服务器
					{
						ESP8266_SendAT("AT+CIPSERVER=1,8080"); //
						if(ESP8266_WaitResponse("OK", 2000))
						{
							break;
						}
					}
					while(1) //设置服务器超时时间
					{
						ESP8266_SendAT("AT+CIPSTO=2880"); //发送“AT+CIPSTO=2880”
						if(ESP8266_WaitResponse("OK", 2000))
						{
							ESP8266_connect_flag = 1;
							break;
						}		
					}
				}
				else if(ESP8266_connect_flag == 1)
				{
					
					Mem_Clr(ESP8266_rx_data, sizeof(ESP8266_rx_data));
					ESP8266_SendAT("AT+CIPSTATUS"); //ESP8266查询当前连接状态
					ESP8266_ReadResponse(ESP8266_rx_data, sizeof(ESP8266_rx_data), 2000);
					Mem_Clr(&ESP8266_connect[0], sizeof(struct ip_infomation) * 4);
					
					if(strstr(ESP8266_rx_data, "+CIPSTATUS:") != NULL)
					{
						WifiStatus = CONNECTING;
					}
				
				}
			break;
			case CONNECTING:
				BSP_OS_SemWait(&wifi_send_sem, 0);
				Mem_Clr(ESP8266_rx_data, sizeof(ESP8266_rx_data));
				ESP8266_ReadResponse(ESP8266_rx_data, sizeof(ESP8266_rx_data), 1000);
				ptr_temp = NULL;
				ptr_temp = strstr(ESP8266_rx_data, "sync"); //接收到同步字
				if(ptr_temp != NULL)
				{
					wifi_rx_cnt++;
					switch(*(ptr_temp+4))
					{
						case 'c':
							MPU6050Flag |= CALI_MODE;  //MPU6050校准模式
						break;
						
						case 'm':
							for(i=0; i<4; i++)
							{
								strncpy(data_temp, (ptr_temp+5+ 3*i), 3);
								data_temp[3] = '\0';
								SetPwmValue[i] = (uint16_t)atoi(data_temp);
							}
						break;
					}
					comClearRxFifo(COM_ESP8266);
				}
			
				BSP_OS_SemPost(&wifi_send_sem);
			break;			
			
		}
		
		BSP_OS_TimeDlyMs(500);
	}
}

void ESP8266_send_data(char* str)
{
	if(WifiStatus == CONNECTING)
	{
		BSP_OS_SemWait(&wifi_send_sem, 0);
		
		Mem_Clr(ESP8266_tx_data, sizeof(ESP8266_tx_data));
		sprintf(ESP8266_tx_data, "AT+CIPSEND=0,%d", strlen(str));
		ESP8266_SendAT(ESP8266_tx_data);
		
		Mem_Clr(ESP8266_rx_data, sizeof(ESP8266_rx_data));
		ESP8266_ReadResponse(ESP8266_rx_data, sizeof(ESP8266_rx_data), 2000);
		
		if(strchr(ESP8266_rx_data, '>') != NULL)
		{
			ESP8266_SendAT(str);
		}
		
		Mem_Clr(ESP8266_rx_data, sizeof(ESP8266_rx_data));
		BSP_OS_SemPost(&wifi_send_sem);
		
	}
		
}
