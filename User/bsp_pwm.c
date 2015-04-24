#include <includes.h>

/*********** PWM模块配置参数 *******************/
uint32_t usPrescaler = 83;   /* PWM时钟预分频系数 */
uint32_t usPeriod = 999;     /* PWM计时周期 */
uint32_t usPWMOffset = 0;    /* PWM占空比补偿 */
/***********************************************/


void bsp_PWMInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	Mem_Set(&GPIO_InitStructure, 0x00, sizeof(GPIO_InitStructure));
	Mem_Set(&TIM_TimeBaseStructure, 0x00, sizeof(TIM_TimeBaseStructure));
	Mem_Set(&TIM_OCInitStructure, 0x00, sizeof(TIM_OCInitStructure));
	
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	/* 连接到AF功能 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	/* 配置GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*-----------------------------------------------------------------------
		system_stm32f4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

		HCLK = SYSCLK / 1	  (AHB1Periph)	168MHz
		PCLK2 = HCLK / 2	  (APB2Periph)	84MHz
		PCLK1 = HCLK / 4	  (APB1Periph)	42MHz

		因为APB1 prescaler != 1, 所以 APB1上的TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
		因为APB2 prescaler != 1, 所以 APB2上的TIMxCLK = PCLK2 x 2 = SystemCoreClock;

		APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
		APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11

	----------------------------------------------------------------------- */
	
	/* 配置TIM5 */
	TIM_TimeBaseStructure.TIM_Period = usPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_CtrlPWMOutputs(TIM5, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	
	/* 方向使能初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}


uint32_t PWM_CCR;
void bsp_SetPWMDutyCycle(uint16_t PWMValue, unsigned char PWMChannel)
{	

	PWM_CCR =(usPeriod + 1) * (uint32_t)PWMValue / 100 - 1 - usPWMOffset;  /* 计算PWM反向寄存器的值 */

	switch(PWMChannel)
	{
		case 1:  /* PWM通道1配置 */
		{
			TIM_SetCompare1(TIM5, PWM_CCR);
		}break;
	
		case 2:  /* PWM通道2配置 */
		{
			TIM_SetCompare2(TIM5, PWM_CCR);
		}break;
		
		case 3:  /* PWM通道3配置 */
		{
			TIM_SetCompare3(TIM5, PWM_CCR);
		}break;

		case 4:  /* PWM通道3配置 */
		{
			TIM_SetCompare4(TIM5, PWM_CCR);
		}break;
		
		default: 
			break;
	}
	
}
