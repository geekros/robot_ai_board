/**
 ******************************************************************************
 * @file    pwm.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef MODULE_PWM
#define MODULE_PWM

#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "utils.h"

#define SERVO_PWM_FREQUENCE 50	// 50Hz    
#define SERVO_PWM_RESOLUTION 20000	// 20ms = 20000us
#define SERVO_DEFAULT_DUTY 1500	// 1500us
#define APB1_TIMER_CLOCKS 72000000
#define APB2_TIMER_CLOCKS 72000000
#define SERVO_TIM_PSC_APB1 ((APB1_TIMER_CLOCKS/SERVO_PWM_FREQUENCE)/SERVO_PWM_RESOLUTION -1)
#define SERVO_TIM_PSC_APB2 ((APB2_TIMER_CLOCKS/SERVO_PWM_FREQUENCE)/SERVO_PWM_RESOLUTION -1)

void Pwm_Init(void);

void TIM3_Init(void);

void TIM8_Init(void);

void Pwm_Control(int channel, uint16_t pwm);

#endif
