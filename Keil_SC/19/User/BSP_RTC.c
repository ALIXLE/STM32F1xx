#include "BSP_RTC.h"

unsigned int days_in_month[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// 返回值 == 1, 代表已经配置过 RTC, 则无需重新配置;
// 若返回值 == 0, 表示未配置过 RTC, 则需指定时间
int RTC_Configuration() {

	// RTC 初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
	if(0xA587 != BKP_ReadBackupRegister(BKP_DR1)){
		
		BKP_DeInit();	// 寄存器复位
						// 仅第一次运行时初始化, 不断电时不初始化
		RCC_LSEConfig(RCC_LSE_ON);	// 32.768 kHz
		while(RESET == RCC_GetFlagStatus(RCC_FLAG_LSERDY));
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		RCC_RTCCLKCmd(ENABLE);
		RTC_WaitForLastTask();	// 等待最近一次对 RTC 寄存器的写操作完成
								// RTC->CRL 位5 为 1
		RTC_WaitForSynchro();	// 等待同步完成
								// RTC->CRL 位3 为 1
		RTC_SetPrescaler(32767);	// [寄存器自加 1]	32768分频 = 32767 + 1
		RTC_WaitForLastTask();
		BKP_WriteBackupRegister(BKP_DR1, 0xA587);	// 此值 0xA587 可自定义
		
		return 0;
	}
	else
		return 1;
}

void Time_Display(u32 TimVal) {
	
	//
	struct rtc_tm tm;
	unsigned int days, hms;
	unsigned int i;
	
	//printf("%d\n", TimVal);
	days = TimVal / 86400;	// 天数
	hms = TimVal % 86400;	// 秒数
	
	//
	tm.hour = hms / 3600;	// 小时
	tm.minute = (hms % 3600) / 60;	// 分钟
	tm.second = (hms % 3600) % 60;	// 秒
	
	for(i = 1970; days >= days_in_year(i); i++) {
		days -= days_in_year(i);
	}
	tm.year = i;
	if(leapyear(i))
		days_in_month[2] = 29;
	else
		days_in_month[2] = 28;
	
	for(i = 1; days >= days_in_month[i]; i++) {
		days -= days_in_month[i];
	}
	tm.month = i;
	tm.day = days + 1;
	
	printf("%.4d-%.2d-%.2d %.2d:%.2d:%.2d\n", tm.year, tm.month, tm.day, tm.hour, tm.minute, tm.second);
	
}

void Set_Time() {
	
	printf("Please enter a time:");
	
	RTC_SetCounter(1200000000);	// 写入值, 根据当前时间计算
	
}
