#include "BSP_USART1.h"


// PA9 TX / PA10 RX
void GPIOA_USATR1_Configuration() {
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	// TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	// RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
}

void USART1_NVIC_Configuration() {
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}

void USART_SendString(const unsigned char *pt) {
	//
	while(*pt)
	{
		// 确保发送缓冲区为空
		while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TXE));
		USART_SendData(USART1, *pt);
		// 等待发送完成
		while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TC));
		pt++;
	}
}

int fputc(int c, FILE *fp) {
	//
	while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	USART_SendData(USART1, c);
	while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TC));
	
	return 0;
}

int fgetc(FILE *stream) {
	
	while(SET != USART_GetFlagStatus(USART1, USART_FLAG_RXNE));
	return (int)USART_ReceiveData(USART1);
}
