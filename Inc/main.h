/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define mains_in_Pin GPIO_PIN_0
#define mains_in_GPIO_Port GPIOA
#define stab_out_Pin GPIO_PIN_1
#define stab_out_GPIO_Port GPIOA
#define Normal_Ind_Pin GPIO_PIN_0
#define Normal_Ind_GPIO_Port GPIOB
#define Buck_Ind_Pin GPIO_PIN_1
#define Buck_Ind_GPIO_Port GPIOB
#define Boost_Ind_Pin GPIO_PIN_2
#define Boost_Ind_GPIO_Port GPIOB
#define Mains_Fail_ip_Pin GPIO_PIN_10
#define Mains_Fail_ip_GPIO_Port GPIOB
#define RLY3_Pin GPIO_PIN_3
#define RLY3_GPIO_Port GPIOB
#define RLY4_Pin GPIO_PIN_5
#define RLY4_GPIO_Port GPIOB
#define RLY1_Pin GPIO_PIN_6
#define RLY1_GPIO_Port GPIOB
#define RLY2_Pin GPIO_PIN_7
#define RLY2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
