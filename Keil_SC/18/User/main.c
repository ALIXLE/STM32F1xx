#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_TIM2.h"

int main(void) {

	SysTick_Configuration();
	GPIOC_Configuration();
	TIM2_BaseConfiguration();
	
	// 初始化时 TIM2 时钟已关闭, 使用前开启
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
	
	while(1) {
		//
	}
}
