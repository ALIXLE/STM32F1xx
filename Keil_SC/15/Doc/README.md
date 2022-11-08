## STM32基础教程

### 低功耗模式 -- 待机模式


说明:
通过按键 PA1 进入 待机STANDBY 模式, PA0 上升沿唤醒


效果等同于硬件复位

```c
void EXTI1_IRQHandler(void) {
	
	if(RESET != EXTI_GetFlagStatus(EXTI_Line1)) {
		
		PWR_ClearFlag(PWR_FLAG_WU);
		// 进入待机模式
		PWR_EnterSTANDBYMode();
		
		// -[此处代码是否执行??]-
		//EXTI_ClearITPendingBit(EXTI_Line1);
	}
	
}
```

注:
	调用中断后 [清除] 中断标识位！！！
