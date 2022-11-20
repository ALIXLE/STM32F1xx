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

​	0~7 低位 8~15 高位	GPIOC_13 位于高位寄存器

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
    while(SysTick_Config(72));	// 库函数完成 STK 使能(STK_CTRL低3位取值 111), 并装载值 72
    SysTick->CTRL &= ~(1<<0);	// STK_CTRL 0位置0, 关闭计数, 使用时开启
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

​			产生 IWDG 复位所需总时间:

​				T(寄存器减一个数用时t * 减数的个数nCount) = t * nCount = 1.6 * 625 = 1000 us

​		IWDG_SR(状态寄存器)

代码:

```c
#include <stm32f10x.h>

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



原理:

代码:

```c
#include <stm32f10x.h>

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
    
    WWDG_Configuration();
    
    while(1) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        Delay_us(1000000);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        Delay_us(1000000);
    }
}

void WWDG_Configuration(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    
    WWDG_SetPrescaler(WWDG_Prescaler_4);
    WWDG_SetWindowValue(0x5F);
    WWDG_Enable(0x7F);
    WWDG_NVIC_Configuration();
    WWDG_ClearFlag();
    WWDG_EnableIT();
}
void WWDG_NVIC_Configuration() {
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void WWDG_IRQHandler() {
	WWDG_SetCounter(0x7F);
    WWDG_ClearFlag();
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

代码:

```c
#include <stm32f10x.h>

void SysTick_Configuration(void);
void USART1_Configuration(void);

int main(void) {
    
    unsigned long TimingDelay;
    
    SysTick_Configuration();
    USART1_Configuration();	// USART1 初始化配置
    
    while(1) {
        USART_SendData(USART1, 0x41);	// 发送 0x41 对应的字符
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

### 11

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



### 12

### 13

### 14

### 15

### 16

### 17

### 18

### 19

数据参考来源:

