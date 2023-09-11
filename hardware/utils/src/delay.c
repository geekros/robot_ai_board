/**
  ******************************************************************************
  * @file    delay.c
  * @author  GEEKROS,  site:www.geekros.com
  ******************************************************************************
*/

#include "delay.h"

static uint8_t fac_us = 0;

void Delay_Init(uint32_t TICK_RATE_HZ){
	uint32_t reload = 0;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us = SystemCoreClock / 8000000;

	if (TICK_RATE_HZ == 0)
	{
		TICK_RATE_HZ = 1000;
	}

	reload = SystemCoreClock / TICK_RATE_HZ / 8;
	reload--;

	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->LOAD = reload;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delay_us(unsigned int microsecond)
{
	uint32_t ticks = 0;
	uint32_t start_tick = 0;
	uint32_t current_tick = 0;
	uint32_t elapsed_ticks = 0;
	uint32_t reload = 0;

	reload = SysTick->LOAD;
	ticks = microsecond * fac_us;
	start_tick = SysTick->VAL;

	while (1)
	{
		current_tick = SysTick->VAL;

		if (current_tick != start_tick)
		{
			if (current_tick < start_tick)
			{
				elapsed_ticks += start_tick - current_tick;
			}
			else
			{
				elapsed_ticks += reload - current_tick + start_tick;
			}

			start_tick = current_tick;

			if (elapsed_ticks >= ticks)
			{
				break;
			}
		}
	}
}

void delay_ms(unsigned int millisecond)
{
	if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		vTaskDelay(millisecond);
	}else{
		delay_us((u32)(millisecond * 1000));
	}
}

int Get_Tick_Count(unsigned long *count)
{
	count[0] = Get_SysTick();
	return 0;
}
