/**
 ******************************************************************************
 * @file    key.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef MODULE_LED
#define MODULE_LED

#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "utils.h"

#define RED_LED_ON() GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define RED_LED_OFF() GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define RED_LED_TOGGLE() GPIO_ToggleBits(GPIOB, GPIO_Pin_12)

#define GREEN_LED_ON() GPIO_ResetBits(GPIOB, GPIO_Pin_13)
#define GREEN_LED_OFF() GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define GREEN_LED_TOGGLE() GPIO_ToggleBits(GPIOB, GPIO_Pin_13)

#define BLUE_LED_ON() GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define BLUE_LED_OFF() GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define BLUE_LED_TOGGLE() GPIO_ToggleBits(GPIOB, GPIO_Pin_14)

void Led_Init(void);

void Led_All_Status(char *status);

void Led_Status(char *channel, char *status);

void Led_Serial_Callback(cJSON *serial_data);

#endif
