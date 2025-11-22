#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//配置时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//配置ADC预分频，推动ADC转换
	
	GPIO_InitTypeDef GPIO_InitStrucure;//配置GPIOA0为模拟输入模式
	GPIO_InitStrucure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStrucure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStrucure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStrucure);
		
	ADC_InitTypeDef ADC_InitStructure;//配置ADC模块
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);//开启电源
	
	ADC_ResetCalibration(ADC1);//校准ADC
	while(ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1) == SET);
}

uint16_t AD_GetValue(uint8_t ADC_Channel)//获取AD转换值的函数
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_239Cycles5);//配置通道
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//若配置为连续转换模式，这一行放到初始化函数最后
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);//若配置为连续转换模式，这一行删掉
	return ADC_GetConversionValue(ADC1);
}
