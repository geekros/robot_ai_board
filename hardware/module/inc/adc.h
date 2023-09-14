/**
 ******************************************************************************
 * @file    adc.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef MODULE_ADC
#define MODULE_ADC

#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "utils.h"

// 注意：用作ADC采集的IO必须没有复用，否则采集电压会有影响
/********************ADC1输入通道（引脚）配置**************************/
#define    ADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    ADC_CLK                       RCC_APB2Periph_ADC1

#define    ADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define    ADC_GPIO_CLK                  RCC_APB2Periph_GPIOA  
#define    ADC_PORT                      GPIOA


// 转换通道个数
#define    NOFCHANEL										 2

#define    ADC_PIN1                      GPIO_Pin_4
#define    ADC_CHANNEL1                  ADC_Channel_4

#define    ADC_PIN2                      GPIO_Pin_5
#define    ADC_CHANNEL2                  ADC_Channel_5


// ADC1 对应 DMA1通道1，ADC3对应DMA2通道5，ADC2没有DMA功能
#define    ADC_x                         ADC1
#define    ADC_DMA_CHANNEL               DMA1_Channel1
#define    ADC_DMA_CLK                   RCC_AHBPeriph_DMA1

extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
extern float ADC_ConvertedValueLocal[NOFCHANEL];

void Adc_Init(void);

#endif
