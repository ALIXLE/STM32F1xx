#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_USART1.h"


int main(void) {
	
	SysTick_Configuration();
	GPIOA_USATR1_Configuration();
	USART1_NVIC_Configuration();
	
	while(1) {
		
	}
}
