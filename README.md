# STM32F1xx 基础教程 - 源码

数据参考来源:



### 01 开发环境搭建

Windows 11 + C51V960A.exe + MDK534.exe



一. 基于Keil μVision 5开发环境搭建

(1) STM32F10x标准外设库

STM32标准外设软件库：https://www.st.com/zh/embedded-software/stm32-standard-peripheral-libraries.html

STM32F10x标准外设库下载（en.stsw-stm32054_v3-6-0_v3.6.0.zip）：https://www.st.com/zh/embedded-software/stsw-stm32054.html

创建项目文件夹$STM32F103C8T6, 在项目文件夹下创建$Doc（README.md）、$MDK-ARM、$User（main.c / main.h）文件夹, $en.stsw-stm32054_v3-6-0_v3.6.0.zip文件解压，复制$STM32F10x_StdPeriph_Lib_V3.6.0文件夹下$Libraries到项目文件夹, 复制$stm32f10x_it.h、$stm32f10x_it.c、$stm32f10x_conf.h (例程文件夹中: ./STM32F10x_StdPeriph_Lib_V3.6.0/Project\STM32F10x_StdPeriph_Template/stm32f10x_conf.h)到$User文件夹



**Keil μVision 5 创建项目**：

1. Project >>

    New μVision Project... >>

    ​	Path: .\STM32F103C8T6\MDK-ARM\STM32F103C8T6.uvprojx

    ​	Device: STM32F103C8

2. Manage Project Items >> Project Targets / Groups / Files

    Project Targets: STM32F103C8T6

    Groups: Startup / CMSIS / StdPeriph_Driver / User / Doc

    Files: 

    ​	Startup >>

    ​		.\STM32F103C8T6\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\startup\arm\startup_stm32f10x_md.s

    ​	CMSIS >>

    ​		.\STM32F103C8T6\Libraries\CMSIS\CM3\CoreSupport\core_cm3.c

    ​		.\STM32F103C8T6\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\system_stm32f10x.c

    ​	StdPeriph_Driver >>

    ​		.\STM32F103C8T6\Libraries\STM32F10x_StdPeriph_Driver\src\\**.c

    ​	User >>

    ​		main.c

    ​		stm32f10x_it.c

    ​	Doc >>

    ​		.\STM32F103C8T6\Doc\README.md

3. Options for Target '$STM32F103C8T6' >>

    Target >>

    ​	Xtal(MHz): 72.0(STM32F103)

    ​	ARM Compiler: **Use default compiler version 5**

    Output >>

    ​	■ Create HEX File

    C/C++ >>

    ​	Define：STM32F10X_MD, USE_STDPERIPH_DRIVER

    ​	Include Paths:

    ​		.\STM32F103C8T6\Libraries\CMSIS\CM3\CoreSupport

    ​		.\STM32F103C8T6\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x

    ​		.\STM32F103C8T6\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\startup\arm

    ​		.\STM32F103C8T6\Libraries\STM32F10x_StdPeriph_Driver\inc

    ​		.\User

    编译环境测试:

    ```c
    // main.h
    #ifndef __MAIN_H__
    #define __MAIN_H__
    
    //
    
    #edndif
    
    --------------------------------文件分割线--------------------------------
    // main.c
    #include "main.h"
        
    int main(void)
    {
        while(1)
        {
            //
        }
    }
    
    ```

    PS：

    编译报错：

    ​	Target > Code Generation：**Use default compiler version 5**

    字符编码：

    ​	Edit > Configuration > Encoding：**UTF-8**

    文件编辑：

    ​	Libraries文件夹 - `只读`选项



### 02 利用寄存器点亮 LED 灯

例程说明:

​	通过配置寄存器控制板载 LED 闪烁

注：

寄存器地址 = ①基地址 + ②偏移地址

GPIOC 时钟使能(RCC 寄存器):

​	根据 STM32F103Cx_en.pdf -> Figure 1. STM32F103xx performance line block diagram 确认 GPIOC 时钟挂载在 APB2 总线上, 然后根据 STM32中文参考手册_V10.pdf -> 7.3.7 APB2 外设时钟使能寄存器(RCC_APB2ENR) 对 GPIOC 时钟进行使能(RCC_APB2ENR 寄存器第4位置1开启GPIOC时钟)

​	① APB2 基地址: 0x40021000 (STM32F103Cx_en.pdf -> 4 Memory mapping -> Figure 11. Memory map）	

​	② APB2 外设时钟使能寄存器(RCC_APB2ENR)偏移地址: 0x18 (STM32中文参考手册_V10.pdf -> 7.3.7 APB2 外设时钟使能寄存器(RCC_APB2ENR))

​	③ APB2 外设时钟使能寄存器地址: 0x40021000 + 0x18 = 0x40021018

​	④ RCC_APB2ENR 第4位置1 (| 操作)

GPIOC 引脚模式配置:

​	根据 STM32F103C8T6原理图.pdf 确认 LED 接在 GPIOC_13 引脚, 根据 STM32中文参考手册_V10.pdf -> 8.2.2 端口配置高寄存器(GPIOx_CRH) (x=A..E) 配置引脚模式为`通用推挽输出模式`.

​	0\~7 低位 8\~15 高位	GPIOC_13 位于高位寄存器

​	端口配置高寄存器(GPIOx_CRH) CNF13 / MODE13 配置为 `0011` -- 通用推挽输出模式, 最大速度50MHz

​	① GPIOC 基地址: 0x40011000 （STM32F103Cx_en.pdf -> 4 Memory mapping -> Figure 11. Memory map）	

​	② GPIOC 端口配置高寄存器偏移地址: 0x04 (STM32中文参考手册_V10.pdf -> 8.2.2 端口配置高寄存器(GPIOx_CRH) (x=A..E))

​	③ GPIOC 端口配置高寄存器地址: 0x40011000 + 0x04 = 0x40011004

​	④ GPIOx_CRH 第13组[CNF / MODE]配置为 0011

GPIOC 引脚写数据(输出高低电平):

​	根据 STM32F103C8T6原理图.pdf 确认当 GPIOC_13 输出低电平时, LED 点亮, 输出高电平时, LED 熄灭.

​	① GPIOC端口输出数据寄存器偏移地址：0x0c

​	② GPIOC端口输出数据寄存器地址：0x40011000 + 0x0c = 0x4001100c

​	③ GPIOx_IDR 第13位置位(| 置1 / & 置0)

流程:

1） 相应引脚时钟使能(GPIOC时钟使能)

```c
*((unsigned long*)(0x40021000 + 0x18)) |= 0x00000010;
```

2） 寄存器初始化

```c
*((unsigned long*)(0x40011000 + 0x04)) &= 0xFF0FFFFF;
*((unsigned long*)(0x40011000 + 0x04)) |= 0x00300000;
```

3） 寄存器赋值

```c
*((unsigned long*)(0x40011000 + 0x0c)) |= 0x00002000;	// GPIOC_ODR_13 置1
*((unsigned long*)(0x40011000 + 0x0c)) &= 0xFFFF0FFF;	// GPIOC_ODR_13 置0
```

代码：

```c
// main.h
#ifndef __MAIN_H__
#define __MAIN_H__

//

#endif

--------------------------------文件分割线--------------------------------
// main.c
#include "main.h"

// 注意 '()' 的使用
// volatile 从寄存器取值
#define RCC_APB2ENR_GPIOC (*((volatile unsigned long*)(0x40021000 + 0x18)))

void Delay(unsigned int nCount);

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

void Delay(unsigned int nCount) {
	while(nCount--);
}

```



### 03 利用库函数点亮 LED 灯

例程说明:

​	利用库函数控制板载 LED 闪烁

流程:

1） GPIOC 时钟使能

```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
```

2） GPIOC_13 引脚配置

```C
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC, &GPIO_InitStructure);
```

3） 引脚赋值

```c
GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
```

代码:

```c
// main.h
#ifndef __MAIN_H__
#define __MAIN_H__

//

#endif

--------------------------------文件分割线--------------------------------
// main.c
#include "main.h"
#include <stm32f10x.h>

void Delay_us(unsigned int nCount);
void GPIOC_Configuration(void);

int main(void) {
    
    GPIOC_Configuration();	// GPIOC 初始化
    
    while(1) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        Delay_us(1000000);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        Delay_us(1000000);
    }
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;	// 定义结构体
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	// 使能 GPIOC 时钟
    // 结构体变量赋值
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	// 13 引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// 输出速度 50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);	// 完成 GPIOC 端口初始化配置
}

void Delay_us(unsigned int nCount) {
    while(cCount--);
}

```



### 04 利用按键控制 LED 状态

例程说明:

​	利用按键改变 LED 状态

​	GPIOC_13 引脚接 LED 灯, GPIOA_0 引脚接按键开关, 通过按下按键开关改变 LED 状态(亮变灭, 灭变亮)

原理:

​	//

流程:

​	1） GPIOC 配置(通用推挽输出模式)

```c
void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
```

​	2） GPIOA 配置(内部上拉输入模式)

```c
void GPIOA_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
```

​	3） 检测 GPIOA_0 引脚状态, 当为 0 时(即按键被按下), GPIOC_13 引脚状态反转

```c
if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
    GPIOC->ODR ^= GPIO_Pin_13;
}
```

代码:

```c
// main.h
#ifndef __MAIN_H__
#define __MAIN_H__

//

#endif

--------------------------------文件分割线--------------------------------
// main.c
#include "main.h"
#include "BSP_Delay.h"
#include "BSP_GPIOC_Configuration.h"
#include "BSP_Key_Scan.h"
    
void GPIOC_Configurstion();	// LED 引脚配置
void GPIOA_Configurstion();	// 按键引脚配置

int main(void) {
    
    // LED 引脚初始化
    GPIOC_Configuration();
    // 按键引脚初始化
    GPIOA_Configuration();
    
    GPIO_RestBits(GPIOC, GPIO_Pin_13);	// 初始状态 LED 点亮
    
    while(1) {
        if(KEY_ON == Key_Scan()) {	// 检测按键是否按下
            while(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));	// 当按键抬起时改变 LED 状态
            GPIOC->ODR ^= GPIO_Pin_13;	// 异或操作, 相同为零不同为一
        }
    }
}

--------------------------------文件分割线--------------------------------
// BSP_GPIOC_Configuration.h
#ifndef __BSP_GPIOC_CONFIGURATION_H__
#define __BSP_GPIOC_CONFIGURATION_H__

#include <stm32f10x.h>

void GPIOC_Configuration(void);

#endif

--------------------------------文件分割线--------------------------------
// BSP_GPIOC_Configuration.c
#include "BSP_GPIOC_Configuration.h"
// LED 引脚模式初始化
void GPIOC_Configuration() {
    // 定义结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    // GPIOC 时钟使能, 挂载在 APB2 总线上
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    // 成员变量赋值
    // 引脚 13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    // 通用推挽输出模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // 输出速率 50MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// 完成初始化
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

--------------------------------文件分割线--------------------------------
// BSP_Key_Scan.h
#ifndef __BSP_KEY_SCAN_H__
#define __BSP_KEY_SCAN_H__
// 宏定义
#define KEY_ON 1
#defien KEY_OFF 0

#endif

--------------------------------文件分割线--------------------------------
// BSP_Key_Scan.c
#include "BSP_Key_Scan.h"
// 按键引脚模式初始化
void GPIOA_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    // GPIOA 时钟使能, 挂载在 APB2 总线上
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 引脚 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    // 速率 50MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // 内部上拉输入模式, 当按键未按下时, 此引脚状态为高电平
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
int Key_Scan() {	// 检测按键状态
    if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {	// 读取 GPIOA_0 引脚电平状态
        Delay_us(0xFF);	// 延时, 软件消抖, 此时间不可太长
        if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {	// 以上延时后再次判断 GPIOA_0 引脚电平状态
            return KEY_ON;	// 经过以上两个 if 进行判断后, 按键确实按下, 返回状态(按键已按下)
        }
        return KEY_OFF;	// 经过第一次 if 判断后, 未进入第二次 if 判断, 可以认为是一次偶然的低电平(返回状态按键未按下)
    }
    return KEY_OFF;	// 未进入 if 判断, 按键未按下, 返回状态(按键未按下)
}

--------------------------------文件分割线--------------------------------
// BSP_Delay.h
#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

void Delay_us(unsigned int nCount);

#endif

--------------------------------文件分割线--------------------------------
// BSP_Delay.c
void Delay_us(unsigned int nCount) {
    while(nCount--);
}

```



### 05 呼吸灯

例程说明:

​	实现 LED 呼吸灯效果(涉及知识点: 占空比)

原理:

​	单位时间内流经 LED 高电平的时间

代码:

```c
#include <stm32f10x.h>

#define TIME 3000	// 周期

void GPIOC_Configuration(void);
void Delay(unsigned int nCount);

int main(void) {
    unsigned int i;
    GPIOC_Configuration();	// 初始化配置 GPIOC_13 引脚, 通用推挽输出
    while(1) {
        for(i = 0; i < TIME; i++) {	// PC13 由暗变亮, 低电平时间越来越长
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
            Delay(i);
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
            Delay(TIME-i);
        }
        for(i = 0; i < TIME; i++) {	// PC13 由亮变暗, 高电平越来越长
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
            Delay(i);
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
            Delay(TIME-i);
        }
    }
}

void Delay(unsigned int nCount) {
    while(nCount--);
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

```



### 06 利用中断控制 LED 状态

例程说明:

​	利用`外部中断`功能改变 LED 状态(亮变灭, 灭变亮)

​	当检测到 GPIOA_0 引脚电平发生改变时, 产生 EXTI0 中断 

原理:



代码:

```c
#include <stm32f10x.h>

void GPIOC_Configuration(void);
void PA0_EXTI0_Configurstion(void);

int main(void) {
    
    GPIOC_Configuration();
    PA0_EXTI0_Configuration();	// 外部中断初始化配置
    
    while(1) {
        //
    }
}

void PA0_EXTI0_Configuration() {
    NVIC_InitTypeDef NVIC_InitStructure;	// ->-向量中断控制器-<-
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;	// ->-外部中断/事件控制器-<-
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	// 需要开启功能复用 IO 时钟(外部中断功能)
    
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	// 使能指定的 IRQ 通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// 抢占优先级, 响应优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	// 从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// 使能 IRQ 通道
	NVIC_Init(&NVIC_InitStructure);	// 完成初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// 内部上拉输入, 默认高电平状态
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);	// 开启引脚的外部中断模式
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;	// 使能外部中断线 0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	// 中断模式为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	// 下降沿触发, 当 GPIOA_0 引脚检测到高电平变为低电平时, 触发中断
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;	// 使能中断
	EXTI_Init(&EXTI_InitStructure);	// 完成 EXTI 寄存器初始化
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// EXTI0 的中断处理函数, 当发生 EXTI0 中断时, 必定跳转至此函数
void EXTI0_IRQHandler() {
    if(SET == EXTI_GetITStatus(EXTI_Line0)) {	// 判断是否真正触发了中断请求
		GPIOC->ODR ^= GPIO_Pin_13;	// 异或操作(不同为 1, 相同为 0), 翻转当前 LED 状态
		EXTI_ClearITPendingBit(EXTI_Line0);	// 清除中断标志位!!! 重要, 否则会持续进中断
	}
}

```



### 07 SysTick 滴答定时器

例程说明:

​	利用 SysTick 实现精确延时

原理:

​	SysTick timer(STK) 24-bit(计数范围: 0~((2^24)-1))

​	相关寄存器:

​		STK_CTRL(控制寄存器)

​			关于计数周期的计算:

​				总时间T = 减数的个数nCount * 减一个数所用时间t

​		STK_LOAD(重装载寄存器)

​			重装载值取值说明:

​				工作频率取值 AHB 时钟频率: 72MHz, 则周期 T=1/f = 1/72000000(此为减一个数所用时间), 当装载值取值 72 时, 则装载值从 72 减至 0 用时 （1/72000000)*72=1us, 此时会进入一次 SysTick_Handler() 中断(装载值减至 0 就会进入一次此中断), 综上, 进几次 SysTick_Handler() 中断就会用时几微秒, 程序中取值 1000000, 即为延时 1s.

​		STK_VAL(当前计数值)

​		STK_CALIB(校准)

参考来源:

​		STM32F10xxx Cortex-M3编程手册.pdf -> 4.5 SysTick timer(STK)

代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void GPIOC_Configuration(void);
void SysTick_Configuration(void);
void Delay_us(unsigned long nCount);

int main(void) {
    
    unsigned long TimingDelay;
    
    GPIOC_Configuration();
    SysTick_Configuration();	// 滴答定时器初始化
    
    while(1) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		Delay_us(100000);	// 延时函数 1s = 1000ms = 1000000us
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		Delay_us(100000);
    }
}

void SysTick_Configuration() {	// STK 初始化函数
    while(SysTick_Config(72));	// 库函数完成 STK 使能(STK_CTRL低3位取值 111), 并装载值 72(取值依据: AHB时钟频率)
    SysTick->CTRL &= ~(1<<0);	// STK_CTRL 0位置0, 关闭计数, 待使用前开启
}
void Delay_us(unsigned long nCount) {	// 延时函数
    TimingDelay = nCount;
	SysTick->CTRL |= (1<<0);	// STK_CTRL 0位置1, 开启计数
	while(TimingDelay);	// 等待数值减完, 用时 TiningDelay 微秒
	SysTick->CTRL &= ~(1<<0);	// STK_CTRL 0位置0, 关闭计数
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SysTick_Handler() {	// STK 中断函数, 当 STK 装载值减至 0 时会跳转至此函数, 根据频率 72MHz 以及 装载值 72, 计算进一次此中断用时 1us
    if(0 != TimingDelay) {	// 判断延时值是否减完, 即是否延时时间到
        TimingDelay--;
    }
}

```



### 08 IWDG 独立看门狗

例程说明:

​	验证 IWDG 工作流程

原理:

​	IWDG: 计数值达到给定的超时值时, 会产生系统复位

​	时钟源: LSI (频率按照 40kHz(非准确值: 30~60之间) 计算)

​	相关寄存器:

​		IWDG_KR(键寄存器)

​			作用: ->-使 IWDG 可以使用-<-

​			程序运行时以一定时间间隔写入 0xAAAA, 否则计数值为 0 时, 会产生看门狗复位

​		IWDG_PR(预分频寄存器)

​			工作频率, 对 LSI 进行分频设置 IWDG_PR = 4*2^PR(PR取值 000~111), 按照 LSI 频率 40kHz， 此处 IWDG_PR 取值 64

​			最终工作频率 f = 40kHz / IWDG_PR = 40kHz / 64

​			寄存器减一个数需要用时:

​				t = 1 / f = IWDG_PR / 40kHz (此处预分频因子取值 100, 即IWDG_PR = 64) = 1.6 us

​		IWDG_RLR(重装载寄存器)

​			设置计数器的初始值, 此处取值 625.

​			产生 IWDG 复位所需总时间T:

​				T(寄存器减一个数用时t * 减数的个数nCount) = t * nCount = 1.6 * 625 = 1000 us

​		IWDG_SR(状态寄存器)

代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void GPIOC_Configuration(void);
void IWDG_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    GPIOC_Configuration();
    // 开始 IWDG 初始化配置
    IWDG_Configuration();
    // 完成 IWDG 初始化配置
    // 开始计数
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);	// LED 亮	<依据-原理图->
	Delay_us(500000);	// 延时 500ms
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);	// LED 灭
	Delay_us(400000);	// 延时 400ms
    // 结束计数, 此时间约 500 + 400 = 900 ms < 1000 ms (1000 依据: 以上说明中 ->-产生IWDG复位所需总时间T-<-) 
	// 以上 MCU 启动后从开始到完成 IWDG 初始化配置约 900ms 进入 while() 循环体, 此时间 < 1000 ms, 不会产生 IWDG 复位
    // 验证: 更改以上总延时时间 > 1000 ms, 程序不会进入到 while() 循环体, 现象: LED 闪烁
    
    while(1) {	// 当以上总延时时间 < 1000 ms 时, 进入此循环体
        IWDG_ReloadCounter();	// IWDG 重装载(重置计数值)
		Delay_us(900000);	// 延时 900ms(< 1000 ms) 后重装载计数值, 不会产生 IWDG 复位
        // 验证: 更改此处延时时间 > 1000 ms, 产生 IWDG 复位, 现象: LED 闪烁(此现象会在 LED 一次亮灭后有-停滞感-)
    }
}

void IWDG_Configuration() {	// IWDG 初始化
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);	// 使能对寄存器 IWDG_PR 和 IWDG_RLR 的写操作
	IWDG_SetPrescaler(IWDG_Prescaler_64);	// 设置 IWDG 的预分频值
	IWDG_SetReload(625);	// 设置 IWDG 的重装载值 (T = t * nCount = 64 / 40kHz * 625 = 1s 不进行重装载则产生复位)
	IWDG_ReloadCounter();	// 将以上重装载值装载至 IWDG 计数器
	IWDG_Enable();	// 使能 IWDG
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 延时, 代码固定
void SysTick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
	SysTick->CTRL |= (1<<0);
	while(TimingDelay);
	SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay) {
        TimingDelay--;
    }
}

```



### 09 WWDG 窗口看门狗

例程说明:

​	验证 WWDG 工作流程

原理:

​	WWDG: 配置有 递减计数器值, 上窗口值, 当计数器值从设置值向下递减至上窗口值之前时, 不能进行重置计数器值(喂狗)操作, 当计数器值减至上窗口值与 0x40 之间时, 可以(必须)进行重置计数器值(喂狗)操作, 当计数器值减至 0x40 时, 若使能了 WWDG_CFR 寄存器的 EWI 位, 则会产生一个中断跳转至 WWDG_IRQHandler() 函数, 可以在此中断函数内进行重置计数器值(喂狗)操作(不建议).

​	相关寄存器:

​		WWDG_CR: 控制寄存器, 使能 WWDG 以及存储计数器值

​		WWDG_CFR: 配置寄存器, 设置上窗口值, 预分频值以及 EWI 中断使能

​			工作频率: f = ((PCLK1 / 4096) / 2^WDGTB)

​			减一个数用时:

​				t = 1 / f = (4096 * 2^WDGTB) / PCLK1

​			超时值计算:

​				最小 <-> 最大计数个数: T[5:0] + 1 = 1 <-> 64

​				(最小: 0x40 -> 0x3F) (最大: 0x7F -> 0x3F)

​				在 PCLK1 = 36 MHz 时的最小-最大超时值(T = t * nCount):

​					WDGTB	min			max

​					0		113us		7.28ms

​					1		227us		14.56ms

​					2		455us		29.12ms

​					3		910us		58.25ms

​		WWDG_SR: 状态寄存器, EWIF 软件写 '0' 清除中断标志

代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void GPIOC_Configuration(void);
void WWDG_Configuration(void);
void WWDG_NVIC_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    GPIOC_Configuration();
    
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
	Delay_us(1000000);
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
	Delay_us(1000000);
    
    WWDG_Configuration();	// WWDG 初始化配置
    // 现象: 上电后, LED 亮一次后一直熄灭.
    // 验证: 修改 WWDG_IRQHandler() 函数, 取消 WWDG_SetCounter() 操作, 现象: LED 闪烁
    
    while(1) {
        
    }
}

void WWDG_Configuration(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);	// WWDG 时钟使能
    
    WWDG_SetPrescaler(WWDG_Prescaler_4);	// 设置预分频系数
    // WWDG_SetCounter(0x7F);	// 设置计数值, 从哪个数开始减
    WWDG_SetWindowValue(0x5F);	// 设置上窗口值
    WWDG_Enable(0x7F);	// 使能 WWDG 并装载计数值,	此处 WWDG_Enable 具有装载初始值的功能, 可以省略上面的 WWDG_SetCounter 函数
    
    WWDG_NVIC_Configuration();	// 涉及到 WWDG_IRQHandler() 中断, 需要对 NVIC 进行配置
    
    WWDG_ClearFlag();	// ! 清除早期的中断控制位
    WWDG_EnableIT();	// 开启中断
}
void WWDG_NVIC_Configuration() {
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;	// 使能 WWDG 的中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void WWDG_IRQHandler() {	// 当 WWDG 计数值减至 0x40 时会跳转至此函数
	WWDG_SetCounter(0x7F);	// 设置计数器值(喂狗)
    WWDG_ClearFlag();	// 清除中断标志位, 否则一直在此中断
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void SysTick_Configuration() {
    while(Systick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay)
        TimingDelay--;
}

```



### 10 USART 配置及字符发送

例程说明:

​	通过 USART 发送字符

代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void USART1_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    USART1_Configuration();	// USART1 初始化配置
    
    while(1) {
        USART_SendData(USART1, 0x41);	// 发送 0x41 对应的字符 'A' (ASCII码表)
        Delay_us(1000000);
    }
}

void USART1_Configuration() {	// USART1 初始化
    
    GPIO_InitTypeDef GPIO_InitStructure;	// 需要使用 GPIO 引脚
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);	// 对应时钟使能
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	// TX -> RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// 复用推挽输出, 输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	// RX -> TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入, 接收
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;	// 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;	// 一个帧中传输的数据位 8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;	// 停止位数目
    USART_InitStructure.USART_Parity = USART_Parity_No;	// 奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	// 发送 | 接收 使能
    USART_Init(USART1, &USART_InitStructure);	// 完成初始化配置
    USART_Cmd(USART1, ENABLE);	// 使能 USART1 外设
}

void SysTick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay)
        TimingDelay--;
}

```



### 11 USART 发送字符串

例程说明:

​	通过 USART 发送字符串



代码:

```c
#include <stm32f10x.h>
#include <stdio.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void USART1_Configuration(void);
void USART1_SendString(const unsigned char *pt);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    USART1_Configuration();	// UASRT1 初始化配置
    USART1_SendString();	// 通过 USART1 发送字符串
    
    while(1) {
        // 方式一
        USART1_SendString("Hello World!\n");
        // 方式二
        printf("Hello World!\n");	// 通过改写 fputc() 发送字符串
    }
}

int fputc(int c, FILE *fp) {	// 改写 fputc(), 通过 USART1 '显示'
    while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TXE));
    USART_SendData(USART1, c);
    while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TC));
    
    return 0;
}

void USART1_SendString(const unsigned char *pt) {	// 字符串发送
    while(*pt) {	// 两处 while() 循环, 当发送数据寄存器为空或数据发送完成时返回 SET 值, 即当判断获取的值!=SET时, 跳出循环
        while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TXE));	// 发送数据寄存器空标志位, 当发送缓冲区为空时, 进行下面的数据发送
        USART_SendData(USART1, *pt);	// 发送字符
        while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TC));	// 发送完成标志位, 循环等待 USART_DR 寄存器内数据发送完毕
        pt++;	// 指向下一位数据
    }
}

// USART1: PA9 TX / PA10 RX
void USART1_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	// TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	// RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);	// 使能 USART1 外设
}

// 延时函数
void SysTick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay)
        TimingDelay--;
}

```



### 12 USART 接收数据

例程说明:

​	利用 UASRT 中断实现字符接收并输出

代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void USART1_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    USART1_Configuration();
    
    while(1) {
        //
    }
}

// USART1: PA9 TX / PA10 RX
void USART1_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	// TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	// RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	// 使能 USART 的接收中断
    USART_Cmd(USART1, ENABLE);	// 使能 USART1 外设
        
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	// 使能 USART1 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
// USART1 中断
void USART1_IRQHandler() {
    while(SET == USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {	// 接收数据寄存器非空返回 SET
        printf("%c", USART_ReceiveData(USART1));
    }
    printf("\n");
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}
int fputc(int c, FILE *fp) {	// 改写 fputc(), 通过 USART1 '显示'
    while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TXE));
    USART_SendData(USART1, c);
    while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TC));
    
    return 0;
}

//
void SysTick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    Systick->CTRL &= ~(1<<0);
}
void SysTick_Handler(){
    if(0 != TimingDelay)
        TimingDelay--;
}

```



### 13 STM32低功耗模式 - 睡眠模式

例程说明:

​	CPU上电启动后 ->

​		1. PC13位 LED 闪烁 5 次后程序调用 __WFI() 进入睡眠模式, 此时现象为: LED 间隔 1000000 us 闪烁 5 次后不再闪烁.

​		2. 当外部 PA0 按键按下后(EXTI0 中断), CPU 被唤醒, 继续执行 while() 语句其余部分, PC13位 LED 闪烁 5 次后进入下一周期的 while() 循环, 此时现象为: LED 间隔 200000 us 闪烁 5 次, 间隔 1000000 us 闪烁 5 次后不再闪烁(进入睡眠模式)

​		3. 重复 1. ~ 2.



代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void GPIOC_Configuration(void);
void PA0_EXTI0_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    unsigned int nCount;
    
    SysTick_Configuration();
    GPIOC_Configuration();
    
    while(1) {
        for(nCount = 0; nCount < 5; nCount++) {	// 延时间隔 1s
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(1000000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(1000000);
        }
        // __WFI() 进入睡眠模式, 通过任一中断唤醒(PA0 EXTI0中断)
        __WFI();
        for(nCount = 0; nCount < 5; nCount++) {	// 延时间隔 0.2s
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(200000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(200000);
        }
    }
}

void PA0_EXTI0_Configuration() {	// PA0_EXTI0 中断
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	// PA0 用作外部中断功能, 需开启复用功能时钟
    
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	// 使能外部中断线 0 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;	// 使能外部中断线 0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	// 中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	// 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;	// 使能
	EXTI_Init(&EXTI_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_EXTILineConfig(GPIO_PortSource_GPIOA, GPIO_PinSource0);	// 使用外部中断线路
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 延时
void SysTick_Configuration() {
    while(SysTick_Config(72));
    StsTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay)
        TimingDelay--;
}

void EXTI0_IRQHandler() {
    EXTI_ClearITPendingBit(EXTI_Line0);	// 清除 EXTI0 线路挂起位
}

```



### 14 低功耗模式 - 停止模式

例程说明:

​	进入停止模式会关闭 HSI / HSE, 当唤醒后 HSE 不会被唤醒, 需要重新初始化, 此工作可在中断函数中进行.

​		参考代码来源: system_stm32f10x.c -> SetSysClockTo72()

​	进入停止模式:

​		①开启 PWR 时钟:
​			RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

​		②调用库函数进入停止模式:
​			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

​	停止模式唤醒方式: 任一中断

​	注:
​		调用中断后需 [清除] 中断标识位！！！

代码:

```c
#include <stdio.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void Delay_us(unsigned long nCount);
void GPIOC_Configuration(void);
void PA0_EXTI0_Configuration(void);
void STOP_Configuration(void);
void SetSysClockTo72MHz(void);

int main(void) {
    
    unsigned long TimingDelay;
    unsigned int nCount;
    
    SysTick_Configuration();
    GPIOC_Configuration();
	PA0_EXTI0_Configuration();
    
    while(1) {
        
        for(nCount = 0; nCount < 5; nCount++) {	// 延时间隔 0.2s
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(200000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(200000);
        }
        // 进入停止模式
        STOP_Configuration();
        
        for(nCount = 0; nCount < 5; nCount++) {	// 延时间隔 0.2s
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
			Delay_us(200000);
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
			Delay_us(200000);
        }
    }
}

void PA0_EXTI0_Configuration() {	// PA0_EXTI0 中断, 下降沿触发
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	// PA0 用作外部中断功能, 需开启复用功能时钟
    
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	// 使能外部中断线 0 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;	// 使能外部中断线 0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	// 中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	// 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;	// 使能
	EXTI_Init(&EXTI_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_EXTILineConfig(GPIO_PortSource_GPIOA, GPIO_PinSource0);	// 使用外部中断线路
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void STOP_Configuration() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}
void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 延时
void SysTick_Configuration() {
    while(SysTick_Config(72));
    StsTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay)
        TimingDelay--;
}
// HSE TO 72MHz, 来源: system_stm32f10x.c
void SetSysClockTo72MHz() {
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
    /* Enable HSE */
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    
    /* Wait till HSE is ready and if Time out is reached exit */
    do {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
    
    if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
        HSEStatus = (uint32_t)0x01;
    }
    else {
        HSEStatus = (uint32_t)0x00;
    }
    
    if (HSEStatus == (uint32_t)0x01) {
        /* Enable Prefetch Buffer */
        FLASH->ACR |= FLASH_ACR_PRFTBE;
        
        /* Flash 2 wait state */
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

		/* HCLK = SYSCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
		/* PCLK2 = HCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

		/* PCLK1 = HCLK */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
        /* Configure PLLs ------------------------------------------------------*/
        /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
        /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */

        RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                                    RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
        RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                                    RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);

        /* Enable PLL2 */
        RCC->CR |= RCC_CR_PLL2ON;
        /* Wait till PLL2 is ready */
        while((RCC->CR & RCC_CR_PLL2RDY) == 0) {
        }


        /* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */
        RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                                RCC_CFGR_PLLMULL9); 
    #else
        /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                            RCC_CFGR_PLLMULL));
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
    #endif /* STM32F10X_CL */

        /* Enable PLL */
        RCC->CR |= RCC_CR_PLLON;

        /* Wait till PLL is ready */
        while((RCC->CR & RCC_CR_PLLRDY) == 0) {
        }

        /* Select PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

        /* Wait till PLL is used as system clock source */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08) {
        }
    }
    else {
		/* If HSE fails to start-up, the application will have wrong clock 
		configuration. User can add here some code to deal with this error */
    }
}

void EXTI0_IRQHandler() {
    // 进入停止模式后, HSE / HSI 时钟振荡器均关闭, 需要重新初始化
    SetSysClockTo72MHz();
    // 清除 EXTI 线路挂起位
    EXTI_ClearITPendingBit(EXTI_Line0);
}

```



### 15 低功耗模式 - 待机模式

例程说明:

​	通过按键 PA1(低电平) 进入 待机STANDBY 模式, 通过 PA0(高电平) 上升沿唤醒

​	CPU 上电启动后 ->

​		LED 间隔 0.2s 闪烁, 当 PA1 引脚检测到低电平时, 进入 EXTI1 中断并进入待机模式(STANDBY), LED 不再闪烁;

​		当 PA0 检测到高电平时, CPU 被唤醒(需使能 PA0 的 WKUP 功能), LED 灯开始闪烁.



代码:

```c
#include <stm32f10x.h>

extern unsigned long TimingDelay;

void SysTick_Configuration(void);
void Delay_us(unsigned long nCount);
void GPIOC_Configuration(void);
void PA1_EXTI1_Configuration(void);
void PA0_WKUP_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    GPIOC_Configuration();
    PA1_EXTI1_Configuration();
    
    PA0_WKUP_Configuration();	// 使能引脚(PA0)唤醒功能
    
    while(1) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        Delay_us(200000);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        Delay_us(200000);
    }
}

void EXTI1_IRQHandler() {
    if(SET == EXTI_GetITStatus(EXTI_Line1)) {	// 检查中断是否发生
        PWR_ClearFlag(PWR_FLAG_WU);	// 清除唤醒标志位
        PWR_EnterSTANDBYMode();	//  进入待机(STANDBY)模式
        // ??
        //EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void PA0_WKUP_Configuration() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	// 使能 PWR 时钟
    PWR_WakeUpPinCmd(ENABLE);	// 使能引脚(PA0)唤醒功能
}

void PA1_EXTI1_Configuration() {
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);	// 外部中断 使能复用时钟
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// 内部上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_EXTILineConfig(GPIO_PortSource_GPIOA, GPIO_PinSource1);	// 使能外部中断模式
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SysTick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    
    SysTick->CTRL |= (1<<0);
    while(TimingDelay--);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimingDelay)
        TimingDelay--;
}

```



### 16 DMA

例程说明:

​	利用 DMA 将内存数据通过 USART 串口发送



​	注(待解决):

​		// 数据位 NCOUNT 值过大时(>=5), 串口助手不显示

代码:

```c
#include <stm32f10x.h>
#define NCOUNT 1

extern unsigned long TimingDelay;
extern unsigned char arr[NCOUNT];

void SysTick_Configuration(void);
void Delay_us(unsigned long nCount);
void GPIOC_Configuration(void);
void USART1_Configuration(void);
void DMA1_Configuration(void);


int main(void) {
    
    unsigned long TimingDelay;
    unsigned char arr[NCOUNT];
    unsigned int i;
    
    SysTick_Configuration();
    GPIOC_Configuration();
    USART1_Configuration();
    DMA1_Configuration();
    
    for(i = 0; i < NCOUNT; i++)
        arr[i] = 'A';
    
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	// 使能 USART 的 DMA 请求
    
    while(1) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        Delay_us(1000000);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        Delay_us(1000000);
    }
}

void DMA1_Configuration() {	// DMA 初始化配置	内存数据传输至外设
    DMA_InitTypeDef DMA_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	// 使能 DMA1 时钟
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (0x40013800 + 0x04);	// DMA 外设基地址: USART_DR
    DMA_InitStructure.DMA_MemoryBaseAddr = (unit32_t)arr;	// DMA 内存基地址: unit32_t, 需要强制类型转换
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	// 外设作为数据传输的目的地
    DMA_InitStructure.DMA_BufferSize = NCOUNT;	// DMA 缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// 外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	// 内存地址寄存器增加 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	// 外设数据宽度
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	// [内存]数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	// 工作模式: Circular 循环缓存模式/ Normal 正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	// DMA 通道的优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	// 是否使能内存到内存的传输
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);	// 完成 DMA1 配置, 通道值参阅 《STM32中文参考手册_V10》-> 10.3.7 表59
    
    DMA_Cmd(DMA1_Channel4, ENABLE);	// 使能通道
}
void USART1_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
	//USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	// 所有初始化完成后, 使能 USART 的 DMA 请求
    USART_Cmd(USART1, ENABLE);
}

void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
// 延时
void SyStick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimningDelay)
        TimingDelay--;
}

```



### 17 ADC 采集 CPU 温度

例程说明:

​	温度传感器

代码:

```c
#include <stm32f10x.h>
#defint V25 0x5
#define AVG_SLOPE 0x6EF

extern unsigned long TimingDelay;
extern uint16_t ADC_ConvertVal;

void SysTick_Configuration(void);
void Delay_us(unsigned long nCount);
void GPIOC_Configuration(void);
void USART1_Configuration(void);
void ADC1_Configuration(void);


int main(void) {
    
    unsigned long TimingDelay;
    uint16_t ADC_ConvertVal;
    float Temperature;
    
    SysTick_Configuration();
    GPIOC_Configuration();
    USART1_Configuration();
    ADC1_Configuration();
    
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    while(1) {
        Temperature = (((V25 - ADC_ConvertVal) / AVG_SLOPE) + 25);
        /*
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        Delay_us(1000000);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        Delay_us(1000000);
        */
        printf("Temperature = %.2f\n", Temperature);        
    }
}
void ADC1_Configuration() {
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	// 使能 DMA1 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (0x40012400 + 0x4C);	// DMA 外设基地址: ADC_DR
    DMA_InitStructure.DMA_MemoryBaseAddr = (unit32_t)&ADC_ConvertVal;	// DMA 内存数据地址: unit32_t, 需要强制类型转换
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	// 外设作为数据传输的来源
    DMA_InitStructure.DMA_BufferSize = 1;	// DMA 缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// 外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	// 内存地址寄存器不变 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_HalfWord;	// 外设数据宽度
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	// [内存]数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	// 工作模式: Circular 循环缓存模式/ Normal 正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	// DMA 通道的优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	// 是否使能内存到内存的传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);	// 完成 DMA1 配置, 通道值参阅 《STM32中文参考手册_V10》-> 10.3.7 表59
    DMA_Cmd(DMA1_Channel1, ENABLE);	// 使能通道
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	// ADC1 / ADC2 工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	// 扫描(多通道)模式 / 单次(单通道)模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// 连续 / 单次模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	// 由软件触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	// 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);	// 设置 ADC 时钟, 由 PCLK 分频
    // 设置 ADC 通道, 采样周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);	// 转换一次用时: 周期 12.5 + 55.5 = 68
																					// 频率 72 / 8 = 9 MHz
																					// 时间 t = 68 * (1 / 9 MHz) = 7.556 us
	ADC_TempSensorVrefintCmd(ENABLE);	// 使能温度传感器和内部参考电压的通道
	ADC_DMACmd(ADC1, ENABLE);	// 使能 ADC 的 DMA 请求
	ADC_Cmd(ADC1, ENABLE);	// 使能指定的 ADC
	
    // ADC 校准
	ADC_ResetCalibration(ADC1);	// 重置校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));	// ?? 获取重置校准寄存器的状态, 重置成功为 RESET
	ADC_StartCalibration(ADC1);	// 开始指定 ADC 的校准状态
	while(ADC_GetCalibrationStatus(ADC1));	// ?? 获取校准程序, RESET
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// 使能 ADC 的软件转换启动功能
}

void USART1_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}
int fputc(int c, FILE *fp) {
	while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	USART_SendData(USART1, c);
	while(SET != USART_GetFlagStatus(USART1, USART_FLAG_TC));
	
	return 0;
}
void GPIOC_Configuration() {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 通用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
// 延时
void SyStick_Configuration() {
    while(SysTick_Config(72));
    SysTick->CTRL &= ~(1<<0);
}
void Delay_us(unsigned long nCount) {
    TimingDelay = nCount;
    
    SysTick->CTRL |= (1<<0);
    while(TimingDelay);
    SysTick->CTRL &= ~(1<<0);
}
void SysTick_Handler() {
    if(0 != TimningDelay)
        TimingDelay--;
}

```



### 18 RTC 定时器

例程说明:

代码:

```c
#include <stm32f10x.h>

int main(void) {
    
    while(1) {
        //
    }
}
```



### 19

数据参考来源:

### 20 矩阵(Matrix)按键

### 21 PWM

### 22 无源蜂鸣器

### 23 超声波传感器

### 24



