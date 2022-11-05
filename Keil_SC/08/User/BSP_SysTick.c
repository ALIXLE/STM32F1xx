#include "BSP_SysTick.h"

unsigned long TimingDelay;

void SysTick_Configuration(void) {
	
	while(SysTick_Config(72));	// 初始化使能定时器, 1us 进一次中断
	SysTick->CTRL &= ~(1<<0);	// 未使用时关闭定时器--寄存器控制CTRL第 0 位清零
								// 1<<0 -> 1
								// ~(1<<0) -> 0
}

void Delay_us(unsigned long nCount) {
	
	TimingDelay = nCount;
	
	SysTick->CTRL |= (1<<0);
	while(TimingDelay);
	SysTick->CTRL &= ~(1<<0);
	
}
