// 配置 GPIOC_13 端口为通用推挽输出模式

#include "BSP_GPIO_Configuration.h"


void GPIOC_Configuration(void) {
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// RCC 时钟使能 GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}
