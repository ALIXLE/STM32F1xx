#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_USART.h"

int main(void) {
	
	SysTick_Configuration();
	GPIOC_Configuration();
	USART_Configuration();
	
	while(1) {
		//
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(100000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(100000);
	}
}
