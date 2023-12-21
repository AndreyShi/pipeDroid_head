/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#define MODBUS_TIMER		TIM7
#define RT_TICK_PER_SECOND	20000
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
static USHORT usT35TimeOut50us;




/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* TIM7 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE_stm);
	;
	usT35TimeOut50us=usTimeOut50us;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = ((uint32_t) usTimeOut50us) - 1;

	TIM_TimeBaseStructure.TIM_Prescaler = 4200 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(MODBUS_TIMER, &TIM_TimeBaseStructure);
	TIM_ITConfig(MODBUS_TIMER, TIM_IT_Update, ENABLE_stm);
	//TIM_Cmd(TIM7, ENABLE_stm);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM7 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE_stm;
	NVIC_Init(&NVIC_InitStructure);

	return TRUE;
}

void vMBMasterPortTimersT35Enable() {
	TIM_Cmd(MODBUS_TIMER, DISABLE_stm);
	TIM_SetCounter(MODBUS_TIMER,0);
	uint32_t timer_tick = (50 * usT35TimeOut50us)
			/ (1000 * 1000 / RT_TICK_PER_SECOND);
	TIM_SetAutoreload(MODBUS_TIMER,timer_tick);
	/* Set current timer mode, don't change it.*/
	vMBMasterSetCurTimerMode(MB_TMODE_T35);

	TIM_Cmd(MODBUS_TIMER, ENABLE_stm);
}

void vMBMasterPortTimersConvertDelayEnable() {
	uint32_t timer_tick = MB_MASTER_DELAY_MS_CONVERT * RT_TICK_PER_SECOND
			/ 1000;
	TIM_Cmd(MODBUS_TIMER, DISABLE_stm);
	TIM_SetCounter(MODBUS_TIMER,0);
	/* Set current timer mode, don't change it.*/
	vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
	TIM_SetAutoreload(MODBUS_TIMER,timer_tick);
	TIM_Cmd(MODBUS_TIMER, ENABLE_stm);
}

void vMBMasterPortTimersRespondTimeoutEnable() {
	uint32_t timer_tick = MB_MASTER_TIMEOUT_MS_RESPOND * RT_TICK_PER_SECOND
			/ 1000;
	TIM_Cmd(MODBUS_TIMER, DISABLE_stm);
	TIM_SetCounter(MODBUS_TIMER,0);
	/* Set current timer mode, don't change it.*/
	vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);
	TIM_SetAutoreload(MODBUS_TIMER,timer_tick);
	TIM_Cmd(MODBUS_TIMER, ENABLE_stm);
}

void vMBMasterPortTimersDisable() {
	TIM_Cmd(MODBUS_TIMER, DISABLE_stm);
}

void TIM7_IRQHandler(void) {
	/* TIM Update event */
	if (TIM_GetFlagStatus(MODBUS_TIMER, TIM_FLAG_Update) != RESET_stm) {
		TIM_ClearFlag(MODBUS_TIMER, TIM_FLAG_Update);
		(void) pxMBMasterPortCBTimerExpired();
	}
}

#endif
