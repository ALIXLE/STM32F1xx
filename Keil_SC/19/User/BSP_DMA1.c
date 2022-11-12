#include "BSP_DMA1.h"

unsigned char arr[1];

void DMA1_Configuration() {
	
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40013804;	// 外设地址USART1_DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)arr;	// 发送内容的地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	// 传输方向: 内存 -> 外设
	DMA_InitStructure.DMA_BufferSize = 1; // 传输数据大小, 暂无单位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// 外设即数据[发送到的地址USART_DR]是否可变
	DMA_InitStructure.DMA_MemoryInc = DMA_PeripheralInc_Enable;	// 需发送的[数据地址arr]是否可变
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;	//发送数据单位大小
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	// 接收数据单位大小
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	// 发送数据是否循环发送
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;	// 优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	// 使能内存到内存传输
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}
