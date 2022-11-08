## STM32基础教程

### 低功耗模式 -- 停止模式

开启 PWR 时钟:
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

调用库函数进入停止模式:
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

停止模式唤醒方式: 任一中断


本例说明:
	
	进入停止模式会关闭 HSI / HSE, 当唤醒后 HSE 不会被唤醒, 需要重新初始化, 此工作可在中断函数中进行.
	
	参考代码来源: system_stm32f10x.c -> SetSysClockTo72()


注:
	调用中断后 [清除] 中断标识位！！！
