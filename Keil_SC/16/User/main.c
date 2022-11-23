#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_USART1.h"
#include "BSP_DMA1.h"


int main(void) {
	
	int i;
	
	SysTick_Configuration();
	GPIOC_Configuration();
	GPIOA_USATR1_Configuration();
	DMA1_Configuration();
	
	for(i = 0; i < NCOUNT; i++) {
		arr[i] = 'A';
	}
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	
	while(1) {
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(1000000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(1000000);
	}
}
