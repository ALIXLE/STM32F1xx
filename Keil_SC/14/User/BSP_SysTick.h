#ifndef __BSP_SYSTICK_H__
#define __BSP_SYSTICK_H__

#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void Delay_us(unsigned long nCount);

#endif
