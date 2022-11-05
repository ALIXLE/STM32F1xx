#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_GPIOA_EXTI0.h"

int main(void) {
	
	GPIOC_Configuration();
	PA0_EXTI0_Configuration();
	
	while(1) {
		//
	}
	
}
