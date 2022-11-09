#include "BSP_TIM2.h"

unsigned int Timer = 0;

void TIM2_BaseConfiguration(void) {
	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	
	TIM_TimeBaseStructure.TIM_Period = 0x3E7;	// 计数 999 + 1 = 1000 个
	TIM_TimeBaseStructure.TIM_Prescaler = 0x47;	// 72MHz 再进行分频	预分频值 71 + 1 = 72
												// 产生中断时间: t = T * cCount = (1 / (72MHz / 72)) * 1000 = 1 ms
												// 内部会自 +1 被除数不能为 0
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );
	
	TIM_Cmd(TIM2, ENABLE);
	
	// 暂时关闭 TIM2 时钟, 使用时开启
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}
