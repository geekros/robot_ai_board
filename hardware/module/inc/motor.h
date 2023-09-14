/**
 ******************************************************************************
 * @file    motor.h
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#ifndef MODULE_MOTOR
#define MODULE_MOTOR

#include <stm32f10x.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pwm.h"
#include "utils.h"

/*
* 电机PWM定时器参数配置
* APB1 prescaler = 2, APB1 Frequency = 36MHz, APB1 TIMER Frequency = 72MHz
* APB2 prescaler = 1, APB2 Frequency = 72MHz, APB2 TIMER Frequency = 72MHz
*/
#define MOTOR_PWM_FREQUENCE  1000            // 1000Hz    
#define MOTOR_PWM_RESOLUTION 1000		         // 1ms = 1000us
#define MOTOR_DEFAULT_DUTY 	 0      				 // 0us
#define MOTOR_TIM_PSC_APB1 ((APB1_TIMER_CLOCKS/MOTOR_PWM_FREQUENCE)/MOTOR_PWM_RESOLUTION-1)

// 圆周率
#define MOTOR_PI 3.1415926535
// 车轮直径,单位毫米
#define WHEEL_DIAMETER 65
// 车轮周长
#define WHEEL_PERIMETER (MOTOR_PI*WHEEL_DIAMETER)
// 电机每周的脉冲数
#define MOTOR_PULSE 330
// 电机闭环控制时间片,单位毫秒
#define MOTOR_TIME_SLICE 20
// 电机PID闭环控制P参数
#define MOTOR_PID_P 20
// 电机PID闭环控制d参数
#define MOTOR_PID_D 15

/*电机状态结构体*/
typedef struct
{
	int16_t motor1_pwm;						// 电机1PWM值
	int16_t motor2_pwm;						// 电机2PWM值
	uint16_t target_speed1;				// 电机1目标速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint16_t target_speed2;				// 电机2目标速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint16_t current_speed1;			// 电机1当前速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint16_t current_speed2;			// 电机2当前速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	int16_t motor1_pulse;					// 电机1编码器在上一个时间片内的脉冲数,即当前脉冲数,每两次采样之间的时间称为1个时间片
	int16_t motor2_pulse;					// 电机1编码器在上一个时间片内的脉冲数,即当前脉冲数,每两次采样之间的时间称为1个时间片
	int32_t motor1_pulse_total;		// 电机1脉冲总计
	int32_t motor2_pulse_total;		// 电机2脉冲总计
	uint8_t motor1_status; 				// 电机1状态,0:正常,1:编码器异常
	uint8_t motor2_status; 				// 电机2状态,0:正常,1:编码器异常
}Encoder_Motor_Status_Struct;

void Encoder_Motor_Init(void);

void Encoder_TIM1_Init(void);
void Encoder_TIM2_Init(void);

void Encoder_Motor1_Init(void);
void Encoder_Motor2_Init(void);

void Encoder_Motor_Read(void);

void Encoder_Motor_Duty(int channel, uint16_t duty);

void Encoder_Motor1_Control(int16_t motor);
void Encoder_Motor2_Control(int16_t motor);

void Encoder_Motor1_Brake(void);
void Encoder_Motor2_Brake(void);

void Encoder_Motor1_Speed_Control(void);
void Encoder_Motor2_Speed_Control(void);

void Encoder_Motor_Speed_Calculate(void);

void Encoder_Motor_Task(void);

#endif
