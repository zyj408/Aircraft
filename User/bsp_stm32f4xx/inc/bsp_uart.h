/*
*********************************************************************************************************
*	                                  
*	模块名称 : 串口驱动模块    
*	文件名称 : bsp_uart.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2012-2013, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_UART_H
#define __BSP_UART_H

extern uint8_t U1TxBuffer[258];
extern uint8_t U1TxCounter;
extern uint8_t U1count; 



void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put_String(unsigned char *Str);
void bsp_InitUart(void);
void UsartSendData(uint8_t mode);
#endif


