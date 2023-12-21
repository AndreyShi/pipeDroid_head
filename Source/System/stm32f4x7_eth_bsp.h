/**
  ******************************************************************************
  * @file    stm32f4x7_eth_bsp.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Header for stm32f4x7_eth_bsp.c file.
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
#ifndef __STM32F4x7_ETH_BSP_H
#define __STM32F4x7_ETH_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define KSZ8041_PHY_ADDRESS       0x01

/* Specific defines for EXTI line, used to manage Ethernet link status */
#define ETH_LINK_EXTI_LINE             EXTI_Line4
#define ETH_LINK_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOB
#define ETH_LINK_EXTI_PIN_SOURCE       EXTI_PinSource4
#define ETH_LINK_EXTI_IRQn             EXTI0_IRQn
/* PC0 */
#define ETH_LINK_PIN                   GPIO_Pin_4
#define ETH_LINK_GPIO_PORT             GPIOB
#define ETH_LINK_GPIO_CLK              RCC_AHB1Periph_GPIOB
/* PHY registers */
#define PHY_IRQ_STAT_CTRL                  0x1b
#define PHY_IRQ_LINK_DOWN         ((uint16_t)(1<<10)) /* link down interrupt enable*/
#define PHY_IRQ_LINK_UP           ((uint16_t)(1<<8)) /* link up interrupt enable*/

#define PHY_LINK_DOWN             ((uint16_t)(1<<2)) /* link down mask */
#define PHY_LINK_UP             ((uint16_t)(1<<0)) /* link up mask */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void  ETH_BSP_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4x7_ETH_BSP_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
