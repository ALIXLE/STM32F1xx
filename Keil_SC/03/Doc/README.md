## STM32基础教程

### 利用库函数点亮 LED 灯

原理图:
PC13 -- LED2

PC13 置 0 / 置 1
      亮  /  灭


时钟 RCC 使能配置:

GPIOC 挂载在 APB2 总线

