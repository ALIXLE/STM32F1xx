#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_ADC.h"
#include "BSP_USART1.h"


int main(void) {

	float Temperature;
	
	SysTick_Configuration();
	GPIOC_Configuration();
	ADC1_Configuration();
	GPIOA_USATR1_Configuration();
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	
	while(1) {
		
		Temperature = ((V25 - ADC_ConvertVal) / AVG_SLOPE) + 25;
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(1000000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(1000000);
		printf("V25 = %d\n", V25);
		printf("ADC_ConvertVal = %d\n", ADC_ConvertVal);
		printf("AVG_SLOPE = %d\n", AVG_SLOPE);	
		
		printf("Temperature = %.6f\n", Temperature);
	}
}
