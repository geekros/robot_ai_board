/**
 ******************************************************************************
 * @file    app.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "app.h"

void Setup()
{
	
}

void Loop()
{
	GREEN_LED_TOGGLE();
	if(Key_Status == 0)
	{
		BLUE_LED_ON();
		Key_Status_Data.status = Key_Status;
	}else{
		BLUE_LED_OFF();
		Key_Status_Data.status = Key_Status;
	}
	
	// Pwm_Test();
	
	uint8_t buffer[] = "123";
	uint16_t length = 3;
	CDC_Send_DATA(buffer, length);
	delay_ms(500);
}

void Pwm_Test(void)
{
	Pwm_Control(7, (uint16_t)1500);
	delay_ms(3500);
	Pwm_Control(7, (uint16_t)500);
	delay_ms(3500);
	Pwm_Control(7, (uint16_t)1500);
	delay_ms(3500);
	Pwm_Control(7, (uint16_t)2500);
	delay_ms(3500);
}
