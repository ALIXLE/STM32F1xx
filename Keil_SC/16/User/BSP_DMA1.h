#ifndef __BSP_DMA1_H__
#define __BSP_DMA1_H__
#define NCOUNT 1

#include <stm32f10x.h>

void DMA1_Configuration(void);

extern unsigned char arr[NCOUNT];

#endif
