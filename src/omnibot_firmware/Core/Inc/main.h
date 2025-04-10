#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// Exported functions
void Error_Handler(void);

// Private defines
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOD
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

// Motor PWM pins
#define M1_PWM1_Pin GPIO_PIN_0
#define M1_PWM1_GPIO_Port GPIOA
#define M1_PWM2_Pin GPIO_PIN_1
#define M1_PWM2_GPIO_Port GPIOA
#define M2_PWM1_Pin GPIO_PIN_2
#define M2_PWM1_GPIO_Port GPIOA
#define M2_PWM2_Pin GPIO_PIN_3
#define M2_PWM2_GPIO_Port GPIOA
#define M3_PWM1_Pin GPIO_PIN_6
#define M3_PWM1_GPIO_Port GPIOA
#define M3_PWM2_Pin GPIO_PIN_7
#define M3_PWM2_GPIO_Port GPIOA
#define M4_PWM1_Pin GPIO_PIN_8
#define M4_PWM1_GPIO_Port GPIOA
#define M4_PWM2_Pin GPIO_PIN_9
#define M4_PWM2_GPIO_Port GPIOA

// Motor direction pins
#define M1_DIR1_Pin GPIO_PIN_0
#define M1_DIR1_GPIO_Port GPIOB
#define M1_DIR2_Pin GPIO_PIN_1
#define M1_DIR2_GPIO_Port GPIOB
#define M2_DIR1_Pin GPIO_PIN_2
#define M2_DIR1_GPIO_Port GPIOB
#define M2_DIR2_Pin GPIO_PIN_3
#define M2_DIR2_GPIO_Port GPIOB
#define M3_DIR1_Pin GPIO_PIN_4
#define M3_DIR1_GPIO_Port GPIOB
#define M3_DIR2_Pin GPIO_PIN_5
#define M3_DIR2_GPIO_Port GPIOB
#define M4_DIR1_Pin GPIO_PIN_6
#define M4_DIR1_GPIO_Port GPIOB
#define M4_DIR2_Pin GPIO_PIN_7
#define M4_DIR2_GPIO_Port GPIOB

// Encoder pins
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOC
#define ENC2_A_Pin GPIO_PIN_2
#define ENC2_A_GPIO_Port GPIOC
#define ENC2_B_Pin GPIO_PIN_3
#define ENC2_B_GPIO_Port GPIOC
#define ENC3_A_Pin GPIO_PIN_4
#define ENC3_A_GPIO_Port GPIOC
#define ENC3_B_Pin GPIO_PIN_5
#define ENC3_B_GPIO_Port GPIOC
#define ENC4_A_Pin GPIO_PIN_6
#define ENC4_A_GPIO_Port GPIOC
#define ENC4_B_Pin GPIO_PIN_7
#define ENC4_B_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */ 