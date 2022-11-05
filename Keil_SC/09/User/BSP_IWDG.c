#include "BSP_IWDG.h"

void IWDG_Configuration(void) {
	
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(625);	// 1s 不进行重装载则产生复位
	IWDG_ReloadCounter();
	IWDG_Enable();
	
}
