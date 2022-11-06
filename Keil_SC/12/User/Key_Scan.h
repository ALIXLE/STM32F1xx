#ifndef __KEY_SCAN_H__
#define __KEY_SCAN_H__

#include <stm32f10x.h>

#define KEY_ON 0	// 按键按下 低电平
#define KEY_OFF 1	// 按键未按下 高电平


void GPIOA_Configuration(void);
int Key_Scan(void);

#endif
