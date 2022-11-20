#include "main.h"
#include "BSP_SysTick.h"
#include "BSP_Matrix_Key.h"

int main()
{
	unsigned int Key_num;
	SysTick_Configuration();
	
	USART1_Configuration();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	while(1)
	{
		Key_num = Matrix_Key();
		if(Key_num != 0)
		{
			//
			printf("Key %d.\n", Key_num);
		}
	}
}
