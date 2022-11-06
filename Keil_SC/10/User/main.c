#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_USART1.h"

int main(void) {
	
	SysTick_Configuration();
	GPIOC_Configuration();
	GPIOA_USATR1_Configuration();
	
	while(1) {
		//
		USART_SendData(USART1, 0x41);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(1000000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(1000000);
	}
}
