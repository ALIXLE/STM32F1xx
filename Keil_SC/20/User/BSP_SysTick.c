#include "BSP_SysTick.h"

unsigned int TimingDelay;

void SysTick_Configuration()
{
	while(SysTick_Config(72));
	
	SysTick->CTRL &= ~(1<<0);
}

void Delay_us(unsigned int nCount)
{
	TimingDelay = nCount;
	
	SysTick->CTRL |= (1<<0);
	while(TimingDelay);
	SysTick->CTRL &= ~(1<<0);
}

