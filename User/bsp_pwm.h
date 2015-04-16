#ifndef __BSP_PWM_H
#define __BSP_PWM_H


void bsp_PWMInit(void);
void bsp_SetPWMDutyCycle(uint16_t PWMValue, unsigned char PWMChannel);

#endif
