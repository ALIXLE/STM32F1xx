#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_RTC.h"
#include "BSP_USART1.h"


int main(void) {

	unsigned int flag;
	
	SysTick_Configuration();
	GPIOC_Configuration();
	flag = RTC_Configuration();
	GPIOA_USATR1_Configuration();
	USART1_NVIC_Configuration();
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
	
	if(flag == 0)	// 未配置过 RTC, 需要设置当前时间
		Set_Time();
	
	while(1) {
		//
		Time_Display(RTC_GetCounter());
		Delay_us(1000000);
	}
}
