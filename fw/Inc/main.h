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

#define ZA2_Pin GPIO_PIN_2
#define ZA2_GPIO_Port GPIOE
#define ZA3_Pin GPIO_PIN_3
#define ZA3_GPIO_Port GPIOE
#define ZA4_Pin GPIO_PIN_4
#define ZA4_GPIO_Port GPIOE
#define ZA5_Pin GPIO_PIN_5
#define ZA5_GPIO_Port GPIOE
#define ZA6_Pin GPIO_PIN_6
#define ZA6_GPIO_Port GPIOE
#define MTX5_Pin GPIO_PIN_13
#define MTX5_GPIO_Port GPIOC
#define MTX6_Pin GPIO_PIN_14
#define MTX6_GPIO_Port GPIOC
#define MTX7_Pin GPIO_PIN_15
#define MTX7_GPIO_Port GPIOC
#define MTX2_Pin GPIO_PIN_0
#define MTX2_GPIO_Port GPIOC
#define MTX3_Pin GPIO_PIN_2
#define MTX3_GPIO_Port GPIOC
#define MTX4_Pin GPIO_PIN_3
#define MTX4_GPIO_Port GPIOC
#define MTX0_Pin GPIO_PIN_3
#define MTX0_GPIO_Port GPIOA
#define MTX1_Pin GPIO_PIN_4
#define MTX1_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_5
#define PS2_CLK_GPIO_Port GPIOA
#define PS2_DAT_Pin GPIO_PIN_6
#define PS2_DAT_GPIO_Port GPIOA
#define OUT0_Pin GPIO_PIN_0
#define OUT0_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_1
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_2
#define OUT2_GPIO_Port GPIOB
#define ZA7_Pin GPIO_PIN_7
#define ZA7_GPIO_Port GPIOE
#define ZA8_Pin GPIO_PIN_8
#define ZA8_GPIO_Port GPIOE
#define ZA9_Pin GPIO_PIN_9
#define ZA9_GPIO_Port GPIOE
#define ZA10_Pin GPIO_PIN_10
#define ZA10_GPIO_Port GPIOE
#define ZA11_Pin GPIO_PIN_11
#define ZA11_GPIO_Port GPIOE
#define ZA12_Pin GPIO_PIN_12
#define ZA12_GPIO_Port GPIOE
#define ZA13_Pin GPIO_PIN_13
#define ZA13_GPIO_Port GPIOE
#define ZA14_Pin GPIO_PIN_14
#define ZA14_GPIO_Port GPIOE
#define ZA15_Pin GPIO_PIN_15
#define ZA15_GPIO_Port GPIOE
#define ZM1_Pin GPIO_PIN_10
#define ZM1_GPIO_Port GPIOB
#define ZD0_Pin GPIO_PIN_8
#define ZD0_GPIO_Port GPIOD
#define ZD1_Pin GPIO_PIN_9
#define ZD1_GPIO_Port GPIOD
#define ZD2_Pin GPIO_PIN_10
#define ZD2_GPIO_Port GPIOD
#define ZD3_Pin GPIO_PIN_11
#define ZD3_GPIO_Port GPIOD
#define ZD4_Pin GPIO_PIN_12
#define ZD4_GPIO_Port GPIOD
#define ZD5_Pin GPIO_PIN_13
#define ZD5_GPIO_Port GPIOD
#define ZD6_Pin GPIO_PIN_14
#define ZD6_GPIO_Port GPIOD
#define ZD7_Pin GPIO_PIN_15
#define ZD7_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOA
#define ZINT_Pin GPIO_PIN_0
#define ZINT_GPIO_Port GPIOD
#define ZNMI_Pin GPIO_PIN_1
#define ZNMI_GPIO_Port GPIOD
#define SD_CD_Pin GPIO_PIN_3
#define SD_CD_GPIO_Port GPIOD
#define ZIORQ_Pin GPIO_PIN_4
#define ZIORQ_GPIO_Port GPIOD
#define ZRD_Pin GPIO_PIN_5
#define ZRD_GPIO_Port GPIOD
#define ZWR_Pin GPIO_PIN_6
#define ZWR_GPIO_Port GPIOD
#define ZMREQ_Pin GPIO_PIN_7
#define ZMREQ_GPIO_Port GPIOD
#define USB_PWR_Pin GPIO_PIN_8
#define USB_PWR_GPIO_Port GPIOB
#define ZA0_Pin GPIO_PIN_0
#define ZA0_GPIO_Port GPIOE
#define ZA0_EXTI_IRQn EXTI0_IRQn
#define ZA1_Pin GPIO_PIN_1
#define ZA1_GPIO_Port GPIOE
#define ZA1_EXTI_IRQn EXTI1_IRQn

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
