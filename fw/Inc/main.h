/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MTX5_Pin GPIO_PIN_13
#define MTX5_GPIO_Port GPIOC
#define MTX6_Pin GPIO_PIN_14
#define MTX6_GPIO_Port GPIOC
#define MTX7_Pin GPIO_PIN_15
#define MTX7_GPIO_Port GPIOC
#define FS_PWR_Pin GPIO_PIN_0
#define FS_PWR_GPIO_Port GPIOC
#define PS2_CLK_Pin GPIO_PIN_2
#define PS2_CLK_GPIO_Port GPIOC
#define PS2_DAT_Pin GPIO_PIN_3
#define PS2_DAT_GPIO_Port GPIOC
#define M1n_Pin GPIO_PIN_0
#define M1n_GPIO_Port GPIOA
#define MTX0_Pin GPIO_PIN_3
#define MTX0_GPIO_Port GPIOA
#define MTX1_Pin GPIO_PIN_4
#define MTX1_GPIO_Port GPIOA
#define MTX2_Pin GPIO_PIN_5
#define MTX2_GPIO_Port GPIOA
#define MTX3_Pin GPIO_PIN_6
#define MTX3_GPIO_Port GPIOA
#define IORQn_Pin GPIO_PIN_0
#define IORQn_GPIO_Port GPIOB
#define RDn_Pin GPIO_PIN_1
#define RDn_GPIO_Port GPIOB
#define WRn_Pin GPIO_PIN_2
#define WRn_GPIO_Port GPIOB
#define WRn_EXTI_IRQn EXTI2_IRQn
#define NMIn_Pin GPIO_PIN_10
#define NMIn_GPIO_Port GPIOB
#define A8_Pin GPIO_PIN_8
#define A8_GPIO_Port GPIOD
#define A9_Pin GPIO_PIN_9
#define A9_GPIO_Port GPIOD
#define A10_Pin GPIO_PIN_10
#define A10_GPIO_Port GPIOD
#define A11_Pin GPIO_PIN_11
#define A11_GPIO_Port GPIOD
#define A12_Pin GPIO_PIN_12
#define A12_GPIO_Port GPIOD
#define A13_Pin GPIO_PIN_13
#define A13_GPIO_Port GPIOD
#define A14_Pin GPIO_PIN_14
#define A14_GPIO_Port GPIOD
#define A15_Pin GPIO_PIN_15
#define A15_GPIO_Port GPIOD
#define FS_PWR_REL_Pin GPIO_PIN_8
#define FS_PWR_REL_GPIO_Port GPIOA
#define HS_PWR_Pin GPIO_PIN_9
#define HS_PWR_GPIO_Port GPIOA
#define ZB_DIR_Pin GPIO_PIN_10
#define ZB_DIR_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOD
#define SD_CD_Pin GPIO_PIN_3
#define SD_CD_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOD
#define OUT0_Pin GPIO_PIN_5
#define OUT0_GPIO_Port GPIOD
#define OUT1_Pin GPIO_PIN_6
#define OUT1_GPIO_Port GPIOD
#define OUT2_Pin GPIO_PIN_7
#define OUT2_GPIO_Port GPIOD
#define MREQn_Pin GPIO_PIN_8
#define MREQn_GPIO_Port GPIOB
#define INTn_Pin GPIO_PIN_9
#define INTn_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
