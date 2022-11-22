#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_GPIOA_EXTI0.h"

int main(void) {
	
	unsigned int nCount;
	
	SysTick_Configuration();
	GPIOC_Configuration();
	PA0_EXTI0_Configuration();
	
	
	while(1) {
		//
		for(nCount = 0; nCount < 5; nCount++) {
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(1000000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(1000000);
		}
		
		// WFI	睡眠模式	任一中断唤醒
		__WFI();
		
		for(nCount = 0; nCount < 5; nCount++) {
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(200000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(200000);
		}
	}
}
