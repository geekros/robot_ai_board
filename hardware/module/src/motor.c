/**
 ******************************************************************************
 * @file    motor.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "pwm.h"
#include "motor.h"

Encoder_Motor_Status_Struct Encoder_Motor_Status = {
	.motor1_pwm = 0,					// 电机1PWM值
	.motor2_pwm = 0,					// 电机2PWM值
	.target_speed1 = 0,				// 电机1目标速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.target_speed2 = 0,				// 电机2目标速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.current_speed1 = 0,			// 电机1当前速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.current_speed2 = 0,			// 电机2当前速度,毫米每秒,最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	.motor1_pulse = 0,				// 电机1编码器在上一个时间片内的脉冲数,即当前脉冲数,每两次采样之间的时间称为1个时间片
	.motor2_pulse = 0,				// 电机1编码器在上一个时间片内的脉冲数,即当前脉冲数,每两次采样之间的时间称为1个时间片
	.motor1_pulse_total = 0,	// 电机1脉冲总计
	.motor2_pulse_total = 0,	// 电机2脉冲总计
	.motor1_status = 0, 			// 电机1状态,0:正常,1:编码器异常
	.motor2_status = 0  			// 电机2状态,0:正常,1:编码器异常
};

void Encoder_Motor_Init(void)
{
	Encoder_TIM1_Init();
	Encoder_TIM2_Init();
	
	Encoder_Motor1_Control(0);
	Encoder_Motor2_Control(0);
	
	Encoder_Motor_Status.motor1_pwm = 500;
	Encoder_Motor_Status.motor2_pwm = 500;
	
	Encoder_Motor1_Init();
	Encoder_Motor2_Init();
}

void Encoder_TIM1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  // 输出比较通道GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为1ms
	
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_RESOLUTION;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_TIM_PSC_APB1;	
	// 时钟分频因子，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/*--------------------输出比较结构体初始化-------------------*/	
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// 输出比较通道 1
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	// 输出比较通道 4
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	//TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	// 使能计数器
	TIM_Cmd(TIM1, ENABLE);
	
	// 主输出使能，当使用的是通用定时器时，这句不需要
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void Encoder_TIM2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
  // 输出比较通道GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 开启定时器时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为1ms
	
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_RESOLUTION;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_TIM_PSC_APB1;	
	// 时钟分频因子，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/*--------------------输出比较结构体初始化-------------------*/	
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// 输出比较通道 3
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	// 输出比较通道 4
	TIM_OCInitStructure.TIM_Pulse = MOTOR_DEFAULT_DUTY;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	//TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	// 使能计数器
	TIM_Cmd(TIM2, ENABLE);
}

void Encoder_Motor1_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	// 使能定时器5的时钟
	
	/*使能GPIOA和AFIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	/*初始化PA0和PA1端口为IN_FLOATING模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;			// 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;		// 设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				// 选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		// TIM向上计数  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	// 使用编码器模式
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);				// 清除TIM的更新标志位
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
  TIM_SetCounter(TIM5, 0);		// 清除计数器
  TIM_Cmd(TIM5, ENABLE); 
}

void Encoder_Motor2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	// 使能定时器5的时钟
	
	/*使能GPIOB和AFIO外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	/*初始化PB6和PB7端口为IN_FLOATING模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;			// 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;		// 设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				// 选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		// TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	// 使用编码器模式
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);				// 清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
  TIM_SetCounter(TIM4, 0);		// 清除计数器
  TIM_Cmd(TIM4, ENABLE); 
}

// TIM5中断服务函数
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  // 清除中断标志位
	}	    
}

// TIM4中断服务函数
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  // 清除中断标志位
	}
}

// 读取单位时间内的编码器计数,用于计算电机转速
void Encoder_Motor_Read(void)
{
	// 读取电机1脉冲数,注意这里用了short,编码器正反转都不会出错
	Encoder_Motor_Status.motor1_pulse = -(short)TIM5 -> CNT;
	TIM5 -> CNT = 0;
	
	Encoder_Motor_Status.motor2_pulse = (short)TIM4 -> CNT;
	TIM4 -> CNT = 0;
}

// 将电机PWM值应用到定时器, 使定时器输出目标PWM波形
void Encoder_Motor_Duty(int channel, uint16_t duty)
{
	if(channel > 4 || channel < 1)
	{
		return;
	}
	
	if(duty > 1000)
	{
		return;
	}
	
	switch(channel)
	{
		case 1: TIM1->CCR1 = duty; break;
		case 2: TIM1->CCR4 = duty; break;
		case 3: TIM2->CCR3 = duty; break;
		case 4: TIM2->CCR4 = duty; break;
	}
}

// 设置电机1的PWM值：-1000~1000, PWM值为正则正转,为负则反转,为0则停转(滑行)
void Encoder_Motor1_Control(int16_t motor)
{
	if(motor > 1000)
	{
		motor = 1000;
	}
	else if(motor < -1000)
	{
		motor = -1000;
	}
	
	// 左侧电机
	if(motor >= 0)				// 正转或停转
	{
		Encoder_Motor_Duty(1, motor);
		Encoder_Motor_Duty(2, 0);
	}
	else									// 反转
	{
		Encoder_Motor_Duty(1, 0);
		Encoder_Motor_Duty(2, -motor);
	}
}

// 设置电机2的PWM值：-1000~1000, PWM值为正则正转,为负则反转,为0则停转(滑行)
void Encoder_Motor2_Control(int16_t motor)
{
	if(motor > 1000)
	{
		motor = 1000;
	}
	else if(motor < -1000)
	{
		motor = -1000;
	}
	
	// 右侧电机
	if(motor >= 0)				// 正转或停转
	{
		Encoder_Motor_Duty(3, 0);
		Encoder_Motor_Duty(4, motor);
	}
	else									// 反转
	{
		Encoder_Motor_Duty(3, -motor);
		Encoder_Motor_Duty(4, 0);
	}
}

// 阻尼刹车, 利用发电效应使电机减速
void Encoder_Motor1_Brake(void)
{
	Encoder_Motor_Duty(1, MOTOR_PWM_RESOLUTION);
	Encoder_Motor_Duty(2, MOTOR_PWM_RESOLUTION);
}

// 阻尼刹车, 利用发电效应使电机减速
void Encoder_Motor2_Brake(void)
{
	Encoder_Motor_Duty(3, MOTOR_PWM_RESOLUTION);
	Encoder_Motor_Duty(4, MOTOR_PWM_RESOLUTION);
}

/*
 * 根据速度计算电机PWM值,速度单位:毫米每秒
 * 这里需要用到PID闭环控制算法,对电机的速度进行控制
 *
 */
void Encoder_Motor1_Speed_Control(void)
{
	// 上次的电机脉冲偏差
	static int16_t motor1_pulse_last_bias = 0;
	
	// 本次的电机脉冲偏差,用来保存目标脉冲数和当前脉冲数的偏差
	int16_t motor1_pulse_bias = 0, int16_temp = 0;
	
	/* 
	 * 根据目标线速度计算目标转速,也就是一个时间片内需要达到的脉冲数,
	 * 因为后边PID算法控制的目标变量就是电机在一个时间片内需要转过的脉冲数
	 * 线速度单位毫米每秒,转速单位为脉冲数每时间片
	 */
	int16_t motor1_pulse_target = 0;
	
	// 判断是否要进入惯性滑行或者阻尼刹车状态
	// target_speed的最高2位用来含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint8_t motor1_mode = 0;
	int8_t motor1_sign = 1;
	
	motor1_mode = Encoder_Motor_Status.target_speed1 >> 14;
	
	// 根据不同状态作出相应处理
	if(motor1_mode == 0)				// 惯性滑行
	{
		Encoder_Motor_Status.motor1_pwm = 0;
		Encoder_Motor1_Control(Encoder_Motor_Status.motor1_pwm);
		return;
	}
	else if(motor1_mode == 3)		// 阻尼刹车
	{
		Encoder_Motor_Status.motor1_pwm = 0;
		Encoder_Motor1_Brake();
		return;
	}
	else if(motor1_mode == 1)		// 正转
	{
		motor1_sign = 1;
	}
	else if(motor1_mode == 2)		// 反转
	{
		motor1_sign = -1;
	}
	
	// 目标脉冲数
	motor1_pulse_target = motor1_sign * ((Encoder_Motor_Status.target_speed1 & 0x3FFF) / WHEEL_PERIMETER * MOTOR_PULSE) / (1000 / MOTOR_TIME_SLICE);
	
//	printf("t:%d, p:%d\n", motor1_pulse_target, motorStatus.motor1_pulse);
	
	// 计算脉冲偏差
	motor1_pulse_bias = motor1_pulse_target - Encoder_Motor_Status.motor1_pulse;
	
	// 计算PWM值
	int16_temp = Encoder_Motor_Status.motor1_pwm + motor1_pulse_bias * MOTOR_PID_P - motor1_pulse_last_bias * MOTOR_PID_D;
	if(int16_temp > 1000)
	{
		int16_temp = 1000;
	}
	else if(int16_temp < -1000)
	{
		int16_temp = -1000;
	}
	
	Encoder_Motor_Status.motor1_pwm = int16_temp;
	
	motor1_pulse_last_bias = motor1_pulse_bias;
	
	Encoder_Motor1_Control(Encoder_Motor_Status.motor1_pwm);
}

/*
 * 根据速度计算电机PWM值,速度单位:毫米每秒
 * 这里需要用到PID闭环控制算法,对电机的速度进行控制
 *
 */
void Encoder_Motor2_Speed_Control(void)
{
	// 上次的电机脉冲偏差
	static int16_t motor2_pulse_last_bias = 0;
	
	// 本次的电机脉冲偏差,用来保存目标脉冲数和当前脉冲数的偏差
	int16_t motor2_pulse_bias = 0, int16_temp = 0;
	
	/* 
	 * 根据目标线速度计算目标转速,也就是一个时间片内需要达到的脉冲数,
	 * 因为后边PID算法控制的目标变量就是电机在一个时间片内需要转过的脉冲数
	 * 线速度单位毫米每秒,转速单位为脉冲数每时间片
	 */
	int16_t motor2_pulse_target = 0;
	
	// 判断是否要进入惯性滑行或者阻尼刹车状态
	// target_speed的最高2位含义为, 二进制00:惯性滑行,01:正转,10:反转,11:阻尼刹车,详情参考手册中通信协议部分
	uint8_t motor2_mode = 0;
	int8_t motor2_sign = 1;
	
	motor2_mode = Encoder_Motor_Status.target_speed2 >> 14;
	
	// 根据不同状态作出相应处理
	if(motor2_mode == 0)				// 惯性滑行
	{
		Encoder_Motor_Status.motor2_pwm = 0;
		Encoder_Motor2_Control(Encoder_Motor_Status.motor2_pwm);
		return;
	}
	else if(motor2_mode == 3)		// 阻尼刹车
	{
		Encoder_Motor_Status.motor2_pwm = 0;
		Encoder_Motor2_Brake();
		return;
	}
	else if(motor2_mode == 1)		// 正转
	{
		motor2_sign = 1;
	}
	else if(motor2_mode == 2)		// 反转
	{
		motor2_sign = -1;
	}
	
	// 目标脉冲数
	motor2_pulse_target = motor2_sign * ((Encoder_Motor_Status.target_speed2 & 0x3FFF) / WHEEL_PERIMETER * MOTOR_PULSE) / (1000 / MOTOR_TIME_SLICE);
	
	// 计算脉冲偏差
	motor2_pulse_bias = motor2_pulse_target - Encoder_Motor_Status.motor2_pulse;
	
	// 计算PWM值
	int16_temp = Encoder_Motor_Status.motor2_pwm + motor2_pulse_bias * MOTOR_PID_P - motor2_pulse_last_bias * MOTOR_PID_D;
	if(int16_temp > 1000)
	{
		int16_temp = 1000;
	}
	else if(int16_temp < -1000)
	{
		int16_temp = -1000;
	}
	
	Encoder_Motor_Status.motor2_pwm = int16_temp;
	
	motor2_pulse_last_bias = motor2_pulse_bias;
	
	Encoder_Motor2_Control(Encoder_Motor_Status.motor2_pwm);
}

// 计算电机当前输出线速度,单位毫米每秒
void Encoder_Motor_Speed_Calculate(void)
{
	int16_t m = 0;
	m = ((float)(Encoder_Motor_Status.motor1_pulse * (1000 / MOTOR_TIME_SLICE)) / MOTOR_PULSE * WHEEL_PERIMETER);
	if(m > 0)
	{
		Encoder_Motor_Status.current_speed1 = (1 << 14) | (m & 0x3FFF);
	}else if(m < 0){
		Encoder_Motor_Status.current_speed1 = (2 << 14) | (m & 0x3FFF);
	}else if(m == 0){
		Encoder_Motor_Status.current_speed1 = Encoder_Motor_Status.target_speed1 & 0xC000;
	}
	
	m = ((float)(Encoder_Motor_Status.motor2_pulse * (1000 / MOTOR_TIME_SLICE)) / MOTOR_PULSE * WHEEL_PERIMETER);
	if(m > 0){
		Encoder_Motor_Status.current_speed2 = (1 << 14) | (m & 0x3FFF);
	}else if(m < 0){
		Encoder_Motor_Status.current_speed2 = (2 << 14) | (m & 0x3FFF);
	}else if(m == 0){
		Encoder_Motor_Status.current_speed2 = Encoder_Motor_Status.target_speed2 & 0xC000;
	}
}

// 任务
void Encoder_Motor_Task(void)
{
	Encoder_Motor_Read();
	Encoder_Motor_Speed_Calculate();
	
	Encoder_Motor1_Control(Encoder_Motor_Status.motor1_pwm);
	Encoder_Motor2_Control(Encoder_Motor_Status.motor2_pwm);
	
	Encoder_Motor1_Speed_Control();
	Encoder_Motor2_Speed_Control();
	
	vTaskDelay(MOTOR_TIME_SLICE);
}
