/**
 ******************************************************************************
 * @file    module.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef MODULE
#define MODULE

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "cpu.h"
#include "flash.h"
#include "hmi.h"
#include "io.h"
#include "key.h"
#include "led.h"
#include "mpu.h"
#include "power.h"
#include "pwm.h"
#include "rocker.h"
#include "usb.h"
#include "wifi.h"
#include "utils.h"

typedef struct
{
    char *type;
} Serial_Data_Struct;

extern Serial_Data_Struct Serial_Read_Data;

void Module_Handle(char *serial_task_buffer);

void Get_Version(void);

#endif
