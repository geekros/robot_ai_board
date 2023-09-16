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
	char str[] = "Hello World!\r\n";
	CDC_Send_DATA((uint8_t*)str, 14);
	CDC_Receive_DATA();
	delay_ms(1500);
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
