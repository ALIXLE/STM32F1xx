#include "main.h"
#include "BSP_GPIO_Configuration.h"
#include "BSP_SysTick.h"
#include "BSP_GPIOA_EXTI0.h"

void STOP_Configuration(void);

int main(void) {
	
	unsigned int nCount;
	
	SysTick_Configuration();
	GPIOC_Configuration();
	PA0_EXTI0_Configuration();
	
	
	while(1) {
		//
		for(nCount = 0; nCount < 5; nCount++) {
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(200000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(200000);
		}
		
		// 停止模式		唤醒后使用低速内部时钟 HSI
		STOP_Configuration();
		
		// 退出停止模式后需初始化 HSI / HSE 时钟, 此工作可在中断中进行
		
	}
}

void STOP_Configuration() {
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	
}
