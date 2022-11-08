// 检测 GPIOA_0 端口电平状态

#include "Key_Scan.h"
#include "Delay.h"


// GPIO_Pin_0 内部上拉输入模式
void GPIOA_Configuration(void) {
	
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	// RCC 时钟使能 GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

int Key_Scan(void) {
	
	//读取 GPIOA_0 状态
	if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
		
		Delay(0xFF);	// 延时	消除抖动
		if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
			
			return KEY_ON;
			
		}
		
		return KEY_OFF;
		
	}
	
	return KEY_OFF;
	
}
