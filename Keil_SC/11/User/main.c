#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_USART1.h"


int main(void) {
	
	SysTick_Configuration();
	GPIOC_Configuration();
	GPIOA_USATR1_Configuration();
	
	while(1) {
		// 方式一
		USART_SendString("USART_SendString: Hello Wolrd\n");
		// 方式二
		printf("printf: Hello World!\n");
	}
}
