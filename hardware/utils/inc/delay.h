/**
 ******************************************************************************
 * @file    delay.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef UTILS_DELAY
#define UTILS_DELAY

#include <stm32f10x.h>
#include "stm32f10x_it.h"

void Delay_Init(uint32_t TICK_RATE_HZ);

void delay_us(unsigned int microsecond);

void delay_ms(unsigned int millisecond);

int Get_Tick_Count(unsigned long *count);

#endif
