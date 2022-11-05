#ifndef __BSP_WWDG_H__
#define __BSP_WWDG_H__

#include <stm32f10x.h>

extern unsigned char WWDG_CNT;

void WWDG_Configuration(unsigned char tr, unsigned char wr, unsigned int pre);
void WWDG_NVIC_Configuration(void);

#endif
