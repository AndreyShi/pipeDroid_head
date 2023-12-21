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
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "main.h"
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/


/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/







/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*Configure GPIO pin Output Level */
	GPIO_WriteBit(TX_R_X__485_GPIO_Port, TX_R_X__485_Pin, Bit_RESET);
	/*Configure GPIO pin : TX_R_X__485_Pin */
	GPIO_InitStruct.GPIO_Pin = TX_R_X__485_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(TX_R_X__485_GPIO_Port, &GPIO_InitStruct);

	/**USART1 GPIO Configuration
	 PA09     ------> USART1_TX
	 PA10     ------> USART1_RX
	 */

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = USARTx_TX_PIN;
	GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = USARTx_RX_PIN;
	GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	/* Connect USART pins to AF7 */
	GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
	GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

	/* Enable USART clock */
	USARTx_CLK_INIT(USARTx_CLK, ENABLE_stm);
	/* Enable the USART OverSampling by 8 */
	USART_OverSampling8Cmd(RS485_PORT, ENABLE_stm);
	/* USARTx configuration ----------------------------------------------------*/
	USART_InitStructure.USART_BaudRate = (uint32_t) ulBaudRate;

	if (ucDataBits == 9)
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	else
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	switch (eParity) {
	case MB_PAR_NONE:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	case MB_PAR_ODD:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	case MB_PAR_EVEN:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	default:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		break;
	};
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RS485_PORT, &USART_InitStructure);


	/* NVIC configuration */
	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE_stm;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable USART */
	USART_Cmd(RS485_PORT, ENABLE_stm);
	RS485_PORT->SR=0;
	RS485_PORT->SR=0;
	RS485_PORT->SR=0;
	RS485_PORT->SR=0;
	vMBMasterPortSerialEnable( TRUE, FALSE);
	RS485_PORT->SR=0;
	RS485_PORT->SR=0;
	RS485_PORT->SR=0;
	RS485_PORT->SR=0;
/*	trace_printf("USART SR=%d\r\n",RS485_PORT->SR);
	trace_printf("USART SR=%d\r\n",RS485_PORT->SR);
	trace_printf("USART SR=%d\r\n",RS485_PORT->SR);
	trace_printf("USART SR=%d\r\n",RS485_PORT->SR);*/
	return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{

	if (xRxEnable) {
		SET_BIT(RS485_PORT->CR1, USART_CR1_RXNEIE);
	} else {
		CLEAR_BIT(RS485_PORT->CR1, USART_CR1_RXNEIE);
	}

	if (xTxEnable) {
		/* Enable the UART Transmit data register empty Interrupt */
		SET_BIT(RS485_PORT->CR1, USART_CR1_TXEIE);
		SET_BIT(RS485_PORT->CR1, USART_CR1_TCIE);
	} else {
		CLEAR_BIT(RS485_PORT->CR1, USART_CR1_TXEIE);
		CLEAR_BIT(RS485_PORT->CR1, USART_CR1_TCIE);
	}
	if (xRxEnable) {

		GPIO_WriteBit(TX_R_X__485_GPIO_Port, TX_R_X__485_Pin, Bit_RESET);
	} else {
		GPIO_WriteBit(TX_R_X__485_GPIO_Port, TX_R_X__485_Pin, Bit_SET);
	}
}

void vMBMasterPortClose(void)
{

}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
	RS485_PORT->DR = ((uint16_t) ucByte & (uint16_t) 0x01FF);
    return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
	*pucByte = (CHAR) (RS485_PORT->DR & (uint16_t) 0x00FF);
    return TRUE;
}






void xMBPortSerial_IRQHandler(void) {
	uint32_t isrflags = RS485_PORT->SR;

	if ((isrflags & USART_SR_RXNE) != RESET_stm) {
		//USART_ClearITPendingBit( USART3, USART_IT_RXNE );
		pxMBMasterFrameCBByteReceived();
	}
	if ((isrflags & USART_SR_TC) != RESET_stm) {
		/* Clear the TC flag in the SR register by writing 0 to it */
		USART_ClearFlag(RS485_PORT, USART_FLAG_TC_stm);
		//pxMBFrameCBTransmitterEmpty();
		vMBMasterPortSerialEnable( TRUE, FALSE);
	}
	if ((isrflags & USART_SR_TXE) != RESET_stm) {
		USART_ClearFlag(RS485_PORT, USART_FLAG_TXE);
		pxMBMasterFrameCBTransmitterEmpty();
	}
}

#endif
