#include "BSP_USART.h"


void USART_Configuration(void) {
	
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART1, &USART_InitStructure);
	
}

void GPIOA_USATR_Configuration() {
	
	GPIO_InitTypeDef GPIOA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	
	GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIOA_InitStructure);
	
}