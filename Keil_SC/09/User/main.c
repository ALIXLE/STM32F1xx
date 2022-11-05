#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_WWDG.h"


int main(void) {
	
	SysTick_Configuration();
	GPIOC_Configuration();
	
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
	Delay_us(1000000);
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
	Delay_us(1000000);
		
	WWDG_Configuration(0x7F, 0x5F, WWDG_Prescaler_4);
	
	while(1) {
		//
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(100000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(100000);
	}
}
