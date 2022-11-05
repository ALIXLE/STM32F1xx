#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"


int main(void) {
	
	GPIOC_Configuration();
	SysTick_Configuration();
	
	//
	
	while(1) {
		//
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(100000);	// 1s = 1000ms = 1000000us
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(100000);
	}
}
