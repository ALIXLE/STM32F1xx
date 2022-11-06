#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_USART1.h"


int main(void) {
	
	unsigned int i = 0;
	
	SysTick_Configuration();
	GPIOC_Configuration();
	GPIOA_USATR1_Configuration();
	
	USART_SendString("Hello World!");
	
	while(1) {
		//
		USART_SendData(USART1, 0x41);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(1000000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(1000000);
		printf("i = %d -- ", i++);
		printf("Hello World!\n");
	}
}
