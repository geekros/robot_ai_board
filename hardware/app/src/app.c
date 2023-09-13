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
	RED_LED_TOGGLE();
	BLUE_LED_TOGGLE();
	uint8_t buffer[] = "123";
	uint16_t length = 3;
	CDC_Send_DATA(buffer, length);
	delay_ms(500);
}
