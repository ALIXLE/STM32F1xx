#include "main.h"

// 注意 '()' 的使用
// volatile 从寄存器取值
#define RCC_APB2ENR_GPIOC (*((volatile unsigned long*)(0x40021000 + 0x18)))

void Delay(unsigned long nCount);

int main(void) {
	
	// RCC_APB2ENR 时钟使能 GPIOC
	//*((unsigned long*)(0x40021000 + 0x18)) |= 0x00000010;	// RCC_APB2ENR 开启 GPIOC 时钟
	RCC_APB2ENR_GPIOC |= 0x00000010;
	
	// GPIOC_13 端口配置高寄存器 通用推挽输出 50MHz
	// 0x40011000 数值, 需强制转化为地址
	// (unsigned long*)0x40011000 地址, 需取出值进行重新赋值
	*((unsigned long*)(0x40011000 + 0x04)) &= 0xFF0FFFFF;	// GPIOC_CRH 13 位清零
	*((unsigned long*)(0x40011000 + 0x04)) |= 0x00300000;	// GPIOC_CRH 13 位通用推挽输出 50MHz
	
	*((unsigned long*)(0x40011000 + 0x0c)) &= 0xFFFF0FFF;	// GPIOC_ODR 13置0
	
	while(1) {
		*((unsigned long*)(0x40011000 + 0x0c)) |= 0x00002000;	// GPIOC_ODR 13位 置1 LED13 亮
		Delay(0xFFFFF);
		*((unsigned long*)(0x40011000 + 0x0c)) &= 0xFFFF0FFF;	// GPIOC_ODR 13位 置0 LED13 灭
		Delay(0xFFFFF);
	}
}

void Delay(unsigned long nCount) {
	while(nCount--);
}
