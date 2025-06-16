/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ECHO1_Pin GPIO_PIN_6
#define ECHO1_GPIO_Port GPIOF
#define ECHO1_EXTI_IRQn EXTI9_5_IRQn
#define ECHO2_Pin GPIO_PIN_7
#define ECHO2_GPIO_Port GPIOF
#define ECHO2_EXTI_IRQn EXTI9_5_IRQn
#define ECHO3_Pin GPIO_PIN_8
#define ECHO3_GPIO_Port GPIOF
#define ECHO3_EXTI_IRQn EXTI9_5_IRQn
#define PPM_Pin GPIO_PIN_9
#define PPM_GPIO_Port GPIOF
#define PPM_EXTI_IRQn EXTI9_5_IRQn
#define TRIG1_Pin GPIO_PIN_10
#define TRIG1_GPIO_Port GPIOF
#define Arm_Servo_CH1_Pin GPIO_PIN_0
#define Arm_Servo_CH1_GPIO_Port GPIOA
#define Roll_Servo_CH2_Pin GPIO_PIN_1
#define Roll_Servo_CH2_GPIO_Port GPIOA
#define Pitch_Servo_CH3_Pin GPIO_PIN_2
#define Pitch_Servo_CH3_GPIO_Port GPIOA
#define Battery_Voltage_Pin GPIO_PIN_2
#define Battery_Voltage_GPIO_Port GPIOH
#define Yaw_Servo_CH4_Pin GPIO_PIN_3
#define Yaw_Servo_CH4_GPIO_Port GPIOA
#define SDMMC_CD_Pin GPIO_PIN_5
#define SDMMC_CD_GPIO_Port GPIOA
#define Motor_Swtich_1_Pin GPIO_PIN_6
#define Motor_Swtich_1_GPIO_Port GPIOA
#define Motor_Switch_2_Pin GPIO_PIN_7
#define Motor_Switch_2_GPIO_Port GPIOA
#define Motor7_CH3_Pin GPIO_PIN_0
#define Motor7_CH3_GPIO_Port GPIOB
#define Motor8_CH4_Pin GPIO_PIN_1
#define Motor8_CH4_GPIO_Port GPIOB
#define TRIG2_Pin GPIO_PIN_11
#define TRIG2_GPIO_Port GPIOF
#define TRIG3_Pin GPIO_PIN_12
#define TRIG3_GPIO_Port GPIOF
#define OSD_CS_Pin GPIO_PIN_11
#define OSD_CS_GPIO_Port GPIOE
#define OSD_SCK_Pin GPIO_PIN_12
#define OSD_SCK_GPIO_Port GPIOE
#define OSD_MISO_Pin GPIO_PIN_13
#define OSD_MISO_GPIO_Port GPIOE
#define OSD_MOSI_Pin GPIO_PIN_14
#define OSD_MOSI_GPIO_Port GPIOE
#define HC_05_Tx_Pin GPIO_PIN_10
#define HC_05_Tx_GPIO_Port GPIOB
#define HC_05_Rx_Pin GPIO_PIN_11
#define HC_05_Rx_GPIO_Port GPIOB
#define STLink_Tx_Pin GPIO_PIN_14
#define STLink_Tx_GPIO_Port GPIOB
#define STLink__RX_Pin GPIO_PIN_15
#define STLink__RX_GPIO_Port GPIOB
#define GPS_Tx_Pin GPIO_PIN_6
#define GPS_Tx_GPIO_Port GPIOC
#define GPS_Rx_Pin GPIO_PIN_7
#define GPS_Rx_GPIO_Port GPIOC
#define Motor1_CH1_Pin GPIO_PIN_8
#define Motor1_CH1_GPIO_Port GPIOA
#define Motor2_CH2_Pin GPIO_PIN_9
#define Motor2_CH2_GPIO_Port GPIOA
#define Motor3_CH3_Pin GPIO_PIN_10
#define Motor3_CH3_GPIO_Port GPIOA
#define Motor4_CH4_Pin GPIO_PIN_11
#define Motor4_CH4_GPIO_Port GPIOA
#define HC_05_EN_Pin GPIO_PIN_12
#define HC_05_EN_GPIO_Port GPIOA
#define YAW_Buck_EN_Pin GPIO_PIN_0
#define YAW_Buck_EN_GPIO_Port GPIOI
#define MPU6050_EN_Pin GPIO_PIN_15
#define MPU6050_EN_GPIO_Port GPIOA
#define BMP388_EN_Pin GPIO_PIN_0
#define BMP388_EN_GPIO_Port GPIOD
#define QMC5883_EN_Pin GPIO_PIN_1
#define QMC5883_EN_GPIO_Port GPIOD
#define ARM_Buck_EN_Pin GPIO_PIN_13
#define ARM_Buck_EN_GPIO_Port GPIOG
#define PITCH_Buck_EN_Pin GPIO_PIN_14
#define PITCH_Buck_EN_GPIO_Port GPIOG
#define ROL_Buck_EN_Pin GPIO_PIN_15
#define ROL_Buck_EN_GPIO_Port GPIOG
#define Motor5_CH1_Pin GPIO_PIN_4
#define Motor5_CH1_GPIO_Port GPIOB
#define Motor6_CH2_Pin GPIO_PIN_5
#define Motor6_CH2_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* Define DEBUG_BYPASS_HEALTH to skip pre-arm health checks and run the
 * debug menu immediately. Enable via compiler flag or uncomment below. */
/* #define DEBUG_BYPASS_HEALTH */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
