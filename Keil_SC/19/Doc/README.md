## STM32基础教程

### RTC 实时时钟


RTC 初始化:

1. 设置寄存器 RCC_APB1ENR 的 PWREN 和 BKPEN 位, 使能电源和后备接口时钟;
	
2. 设置寄存器 PWR_CR 的 DBP 位, 使能对后备寄存器和 RTC 的访问.

3. 复位 RTC_PRL / RTC_ALR / RTC_CNT / RTC_DIV 寄存器

4. 使能 LSE 外部振荡时钟

5. 等待 LSE 起振就绪

6. 设置 RTC 时钟 -> LSE

7. 使能 RTC 时钟, 开始工作

8. 等待 RTC 寄存器写操作完成

9. 等待寄存器同步标志位置 1

10. 设置 32768 分频 

11. 等待 RTC 寄存器写操作完成

