/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERSHIK_DEF_H
#define __ERSHIK_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define spi_clk_mdc_Pin GPIO_PIN_1
#define spi_clk_mdc_GPIO_Port GPIOC
#define mtm_clock_Pin GPIO_PIN_1
#define mtm_clock_GPIO_Port GPIOA
#define spi_mosi_mdio_Pin GPIO_PIN_2
#define spi_mosi_mdio_GPIO_Port GPIOA
#define mtm_rxdv_Pin GPIO_PIN_7
#define mtm_rxdv_GPIO_Port GPIOA
#define mtm_rxd0_Pin GPIO_PIN_4
#define mtm_rxd0_GPIO_Port GPIOC
#define mtm_rxd1_Pin GPIO_PIN_5
#define mtm_rxd1_GPIO_Port GPIOC
#define mtm_txen_Pin GPIO_PIN_11
#define mtm_txen_GPIO_Port GPIOB
#define mtm_txd0_Pin GPIO_PIN_12
#define mtm_txd0_GPIO_Port GPIOB
#define mtm_txd1_Pin GPIO_PIN_13
#define mtm_txd1_GPIO_Port GPIOB
#define rx_tx_Pin GPIO_PIN_8
#define rx_tx_GPIO_Port GPIOA
#define uart_tx_Pin GPIO_PIN_9
#define uart_tx_GPIO_Port GPIOA
#define uart_rx_Pin GPIO_PIN_10
#define uart_rx_GPIO_Port GPIOA
#define intr_n_Pin GPIO_PIN_11
#define intr_n_GPIO_Port GPIOA
#define intr_n_EXTI_IRQn EXTI15_10_IRQn
#define rst_n_Pin GPIO_PIN_12
#define rst_n_GPIO_Port GPIOA
#define phy_int_Pin GPIO_PIN_3
#define phy_int_GPIO_Port GPIOD
#define phy_int_EXTI_IRQn EXTI3_IRQn
#define phy_reset_Pin GPIO_PIN_4
#define phy_reset_GPIO_Port GPIOD
#define lis_int1_Pin GPIO_PIN_4
#define lis_int1_GPIO_Port GPIOB
#define lis_int2_Pin GPIO_PIN_5
#define lis_int2_GPIO_Port GPIOB
#define lis_scl_Pin GPIO_PIN_6
#define lis_scl_GPIO_Port GPIOB
#define lis_sda_Pin GPIO_PIN_7
#define lis_sda_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
