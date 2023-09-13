/**
 ******************************************************************************
 * @file    key.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "led.h"

void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GREEN_LED_OFF();
	RED_LED_OFF();
	BLUE_LED_OFF();
}

void Led_All_Status(char *status) {
    if (strcmp(status, "off") == 0) {
        GREEN_LED_OFF();
        RED_LED_OFF();
        BLUE_LED_OFF();
    } else if (strcmp(status, "on") == 0) {
        GREEN_LED_ON();
        RED_LED_ON();
        BLUE_LED_ON();
    } else if (strcmp(status, "switch") == 0) {
        GREEN_LED_TOGGLE();
        RED_LED_TOGGLE();
        BLUE_LED_TOGGLE();
    }
}

void Led_Status(char *channel, char *status) {
    if (strcmp(channel, "green") == 0) {
        if (strcmp(status, "switch") == 0) {
            GREEN_LED_TOGGLE();
        } else if (strcmp(status, "off") == 0) {
            GREEN_LED_OFF();
        } else if (strcmp(status, "on") == 0) {
            GREEN_LED_ON();
        }
    } else if (strcmp(channel, "red") == 0) {
        if (strcmp(status, "switch") == 0) {
            RED_LED_TOGGLE();
        } else if (strcmp(status, "off") == 0) {
            RED_LED_OFF();
        } else if (strcmp(status, "on") == 0) {
            RED_LED_ON();
        }
    } else if (strcmp(channel, "blue") == 0) {
        if (strcmp(status, "switch") == 0) {
            BLUE_LED_TOGGLE();
        } else if (strcmp(status, "off") == 0) {
            BLUE_LED_OFF();
        } else if (strcmp(status, "on") == 0) {
            BLUE_LED_ON();
        }
    }
}

void Led_Serial_Callback(cJSON *serial_data)
{
    cJSON *type = cJSON_GetObjectItem(serial_data, "type");
    if (type && cJSON_IsString(type))
    {
        if(strcmp(type->valuestring, "led-status") == 0)
        {
            cJSON *channel = cJSON_GetObjectItem(serial_data, "channel");
            cJSON *status = cJSON_GetObjectItem(serial_data, "status");
            if(strcmp(channel->valuestring, "all") == 0)
            {
                Led_All_Status(status->valuestring);
            }else{
                Led_Status(channel->valuestring, status->valuestring);
            }
        }
    }
}
