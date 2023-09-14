/**
 ******************************************************************************
 * @file    key.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef MODULE_KEY
#define MODULE_KEY

#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "utils.h"

#define Key_Status GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)

typedef struct
{
	int status;
}Key_Status_Struct;

extern Key_Status_Struct Key_Status_Data;

void Key_Init(void);

void Key_Serial_Callback(cJSON *serial_data);

#endif
