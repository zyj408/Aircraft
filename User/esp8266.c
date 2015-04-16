#include <includes.h>

#define WIFI_TIMEOUT 20
#define BUFF_SIZE 200

enum WIFI_STATUS WifiStatus;  //WIFI连接状态
char ReceiveBuff[BUFF_SIZE];  //指令接收堆栈
uint8_t ReceivePtr;  //指令接收指针
char ReceiveFlag;  //指令接收标志位

OS_TMR COM_OT_TIMER;
OS_ERR err;

uint8_t bsp_ESP8266_WaitResponse(void);


void ComOT_CallBack (OS_TMR *p_tmr, void *p_arg)
{
	if(ReceivePtr != 0)
	{
		ReceivePtr = 0;
		ReceiveFlag = 0;
		
		OSTmrStop ((OS_TMR            *) &COM_OT_TIMER,
               (OS_OPT             ) OS_OPT_TMR_NONE,
               (OS_TMR_CALLBACK_PTR) ComOT_CallBack,
               (OS_ERR            *) &err);
	}
}
void bsp_ESP8266SetupAP(void)
{
	ReceiveFlag = 0;
  printf("AT+RST\r\n");
	bsp_ESP8266_WaitResponse();

}

uint8_t bsp_ESP8266_WaitResponse(void)
{
	int timeout = 0;
	
	while(timeout++ < WIFI_TIMEOUT)
	{
		if(ReceiveFlag == 1)
			return 1;
		
		OSTimeDlyHMSM((CPU_INT16U) 0u,
                  (CPU_INT16U) 0u,
                  (CPU_INT16U) 1u,
                  (CPU_INT32U) 0u,
                  (OS_OPT    ) OS_OPT_TIME_HMSM_STRICT,
                  (OS_ERR   *)&err);
	}
	
	return 0;
}
void AppCommTask(void *p_arg)
{
	WifiStatus = IDLE;
	Mem_Set(ReceiveBuff, 0x00, sizeof(char));  //初始化指令接收堆栈
	ReceivePtr = 0;  //初始化指令接收指针
	ReceiveFlag = 0;  //初始化指令接收标志位
	(void)p_arg;
			
	OSTmrCreate((OS_TMR            *) &COM_OT_TIMER,
							(CPU_CHAR          *) "COM Over Timer",
							(OS_TICK            ) 0,
							(OS_TICK            ) 2*10,
							(OS_OPT             ) OS_OPT_TMR_ONE_SHOT,
							(OS_TMR_CALLBACK_PTR) ComOT_CallBack,
							(void              *) 0,
							(OS_ERR            *) &err);
	
	while(1)
	{
		switch (WifiStatus)
		{
			case IDLE:
				bsp_ESP8266SetupAP();
			break;
			case AP_BEGIN:
				//bsp_ESP8266SetupAP();
			break;
			case CONNECTING:
				//bsp_ESP8266SetupAP();
			break;			
			
		}
		
		
		BSP_OS_TimeDlyMs(10);  //延迟10ms
	}
}


void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		USART_SendData(USART1, U1TxBuffer[U1TxCounter++]);                    

    /* Clear the USART1 transmit interrupt */
    USART_ClearITPendingBit(USART1, USART_IT_TXE); 

    if(U1TxCounter == U1count)
    {
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }    
	}
	
	else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	
		if(ReceivePtr == 0)
		{
			ReceiveBuff[ReceivePtr++] = USART_ReceiveData(USART1);
			OSTmrStart(&COM_OT_TIMER, &err);
		}
		else if(ReceivePtr>0 && ReceivePtr<BUFF_SIZE)
		{
			
			ReceiveBuff[ReceivePtr++] = USART_ReceiveData(USART1);
			
			if(ReceiveBuff[ReceivePtr] == '\n')
				ReceiveFlag = 1;
			OSTmrStart(&COM_OT_TIMER, &err);
		}
		else
		{
			ReceivePtr = 0;
			OSTmrStop ((OS_TMR            *) &COM_OT_TIMER,
                 (OS_OPT             ) OS_OPT_TMR_NONE,
                 (OS_TMR_CALLBACK_PTR) ComOT_CallBack,
                 (OS_ERR            *) &err);
		}
	}

}
