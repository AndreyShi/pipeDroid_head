/*
 * motor.c
 *
 *  Created on: 24 дек. 2016 г.
 *      Author: real_bastard
 */
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "main.h"
#include "motor.h"


static int16_t AB_speed = 500;


void motor_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

	GPIO_InitStructure.GPIO_Pin = MOTOR_A_PIN | MOTOR_AN_PIN | MOTOR_B_PIN
			| MOTOR_BN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(MOTOR_AB_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(MOTOR_AB_GPIO_PORT,
	MOTOR_A_PIN | MOTOR_AN_PIN | MOTOR_B_PIN | MOTOR_BN_PIN);

	return;
	GPIO_PinAFConfig(MOTOR_AB_GPIO_PORT, MOTOR_A_PIN_SOURCE, MOTOR_AB_AF);
	GPIO_PinAFConfig(MOTOR_AB_GPIO_PORT, MOTOR_B_PIN_SOURCE, MOTOR_AB_AF);



	MOTOR_AB_TIMER_CLK_EN;
	// Compute the prescaler value
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 32000 / 1000 / 2);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(MOTOR_AB_TIMER, &TIM_TimeBaseStructure);


	// PWM1 Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(MOTOR_AB_TIMER, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(MOTOR_AB_TIMER, TIM_OCPreload_Enable);
	TIM_OC3Init(MOTOR_AB_TIMER, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(MOTOR_AB_TIMER, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(MOTOR_AB_TIMER, ENABLE);
	TIM_Cmd(MOTOR_AB_TIMER, ENABLE);

	AB_speed = -500;
	motor_setSpeed_AB(0);
}

void motor_setSpeed_AB(int16_t Speed) {
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PWM_Val = 0;
	if (Speed < -1000)
		Speed = -1000;
	if (Speed > 1000)
		Speed = 1000;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	if (Speed == AB_speed)
		return;
	if (((Speed >= 0) && (AB_speed >= 0)) || ((Speed < 0) && (AB_speed < 0))) {
		PWM_Val = abs(Speed);
		TIM_SetCompare1(MOTOR_AB_TIMER, PWM_Val);
		TIM_SetCompare3(MOTOR_AB_TIMER, PWM_Val);
	} else {
		TIM_SetCompare1(MOTOR_AB_TIMER, 0);
		TIM_SetCompare3(MOTOR_AB_TIMER, 0);
		GPIO_ResetBits(MOTOR_AB_GPIO_PORT, MOTOR_AN_PIN | MOTOR_BN_PIN);
		GPIO_InitStructure.GPIO_Pin = MOTOR_A_PIN | MOTOR_B_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_Init(MOTOR_AB_GPIO_PORT, &GPIO_InitStructure);
		for (int i = 0; i < 10000; i++) {
			__NOP();
		}
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		if (Speed >= 0) {
			GPIO_InitStructure.GPIO_Pin = MOTOR_A_PIN;
			GPIO_SetBits(MOTOR_AB_GPIO_PORT, MOTOR_BN_PIN);
		} else {
			GPIO_InitStructure.GPIO_Pin = MOTOR_B_PIN;
			GPIO_SetBits(MOTOR_AB_GPIO_PORT, MOTOR_AN_PIN);
		}
		GPIO_Init(MOTOR_AB_GPIO_PORT, &GPIO_InitStructure);
		PWM_Val = abs(Speed);
		TIM_SetCompare1(MOTOR_AB_TIMER, PWM_Val);
		TIM_SetCompare3(MOTOR_AB_TIMER, PWM_Val);
	}
	AB_speed = Speed;
}

