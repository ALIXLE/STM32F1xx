#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "Delay.h"

#define TIME 3000

int main(void) {
	
	unsigned long i;
	
	GPIOC_Configuration();
		
	while(1) {
		
		for(i = 0; i <= TIME; i++) {	// GPIO 由灭变亮, 低电平越来越长
			
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay(i);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay(TIME - i);
			
		}
		for(i = 0; i <= TIME; i++) { // GPIO 由亮变灭, 高电平越来越长
			
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay(i);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay(TIME - i);
			
		}
	}
	
}
