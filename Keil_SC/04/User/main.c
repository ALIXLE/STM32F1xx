#include "main.h"
#include "Delay.h"
#include "BSP_GPIO_Configuration.h"
#include "Key_Scan.h"


int main() {
	
	// GPIOC 初始化
	GPIOC_Configuration();
	
	// GPIOA 初始化
	GPIOA_Configuration();
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);	// 置0 亮	
	
	while(1) {
		
		if(KEY_ON == Key_Scan()) {	// 检测按键是否按下
			
			while(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));	// 循环检测, 当按键抬起时改变状态
			GPIOC->ODR ^= GPIO_Pin_13;	// 异或操作 相同为0不同为1
			
		}
		
	}
	
}
