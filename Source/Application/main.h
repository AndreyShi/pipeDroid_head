/**
 ******************************************************************************
 * @file    main.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    07-October-2011
 * @brief   This file contains all the functions prototypes for the main.c
 *          file.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H



/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4x7_eth_bsp.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//

#define HEAD_PART	1



#if HEAD_PART==1
#define FIRST_MODBUS_ADR	10
#define LAST_MODBUS_ADR		9
#else
#define FIRST_MODBUS_ADR	10
#define LAST_MODBUS_ADR		12
#endif

/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
//#define IP_ADDR0   169
//#define IP_ADDR1   254
//#define IP_ADDR2   219
#define IP_ADDR0   169
#define IP_ADDR1   254
#define IP_ADDR2   43

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   0x00
#define MAC_ADDR1   0x1B
#define MAC_ADDR2   0x38
#define MAC_ADDR3   0xAF
#define MAC_ADDR4   0xC9


//#define IP_ADDR2   100
#if HEAD_PART==1
//#define IP_ADDR3   100
#define IP_ADDR3   150
#define MAC_ADDR5   0x10
#else
#define IP_ADDR3   101
#define MAC_ADDR5   0x11
#endif

//#define DEST_IP_ADDR0   169
//#define DEST_IP_ADDR1   254
//#define DEST_IP_ADDR2   219
//#define DEST_IP_ADDR3   106

//home ip ethrnet adapter Dhcp ip addres
#define DEST_IP_ADDR0   169
#define DEST_IP_ADDR1   254
#define DEST_IP_ADDR2   43
#define DEST_IP_ADDR3   151

#define UDP_LOCAL_PORT    	10000   /* define the UDP local connection port */
#define UDP_SERVER_PORT    	10001   /* define the UDP local connection port */

/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   0
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   169
#define GW_ADDR1   254
#define GW_ADDR2   219
#define GW_ADDR3   100  

//#define MII_MODE

/* Exported macro ------------------------------------------------------------*/




#define MOTOR_A_PIN                    	GPIO_Pin_12
#define MOTOR_AN_PIN                   	GPIO_Pin_13
#define MOTOR_B_PIN                    	GPIO_Pin_14
#define MOTOR_BN_PIN                   	GPIO_Pin_15
#define MOTOR_A_PIN_SOURCE             	GPIO_PinSource12
#define MOTOR_B_PIN_SOURCE             	GPIO_PinSource14
#define MOTOR_AB_GPIO_PORT				GPIOD
#define MOTOR_AB_AF              		GPIO_AF_TIM4
#define MOTOR_AB_TIMER_CLK_EN      		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE_stm)
#define MOTOR_AB_TIMER					TIM4




#define TX_R_X__485_Pin 		GPIO_Pin_8
#define TX_R_X__485_GPIO_Port 	GPIOA

#define RS485_PORT                       USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn_stm
#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_USART1

#define USARTx_RX_PIN                    GPIO_Pin_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_USART1


/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM             DMA2_Stream7
/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn               DMA2_Stream7_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler


#define ENCODER_SPI                   SPI3
#define ENCODER_SPI_CLK                       RCC_APB1Periph_SPI3
#define ENCODER_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd
#define ENCODER_SPI_IRQn                      SPI3_IRQn
#define ENCODER_SPI_IRQHANDLER                SPI3_IRQHandler

#define ENCODER_SPI_SCK_PIN                   GPIO_Pin_3
#define ENCODER_SPI_SCK_GPIO_PORT             GPIOB
#define ENCODER_SPI_SCK_SOURCE                GPIO_PinSource3
#define ENCODER_SPI_SCK_AF                    GPIO_AF_SPI3

#define ENCODER_SPI_MISO_PIN                  GPIO_Pin_4
#define ENCODER_SPI_MISO_GPIO_PORT            GPIOB
#define ENCODER_SPI_MISO_SOURCE               GPIO_PinSource4
#define ENCODER_SPI_MISO_AF                   GPIO_AF_SPI3

#define CS_ENCODER__PIN             GPIO_Pin_5
#define CS_ENCODER__PORT			GPIOB

#define I2C                          I2C1_stm
#define I2C_CLK                      RCC_APB1Periph_I2C1

#define I2C_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C_SDA_PIN                  GPIO_Pin_7
#define I2C_SDA_GPIO_PORT            GPIOB
#define I2C_SDA_SOURCE               GPIO_PinSource7
#define I2C_SDA_AF                   GPIO_AF_I2C1

#define I2C_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C_SCL_PIN                  GPIO_Pin_6
#define I2C_SCL_GPIO_PORT            GPIOB
#define I2C_SCL_SOURCE               GPIO_PinSource6
#define I2C_SCL_AF                   GPIO_AF_I2C1


typedef enum{
	simple,
	modbus
}protocol_t;



/* Exported functions ------------------------------------------------------- */
void Time_Update(void);
void Delay_ms(uint32_t nCount);
uint32_t getTime_ms(void);
int udp_printf(const char* format, ...);
void power_set(int state);
int getKey(void);
#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

