## STM32第二阶段

### Matrix - 矩阵按键

行列转换


S1		S5		S9		S13		PA4

S2		S6		S10		S14		PA5

S3		S7		S11		S15		PA6

S4		S8		S12		S16		PA7

PA0		PA1		PA2		PA3

1) PA0~PA3 配置内部上拉输入模式
PA4~PA7 配置通用推挽输出, 低电平


2) PA4~PA7 配置内部上拉输入模式
PA0~PA3 配置通用推挽输出, 低电平


PA0~PA3 读取到低电平, 有按键按下, 进行判断得到[列];

反转行列, PA4~PA7 读取到低电平, 有按键按下, 进行判断得到[行].

