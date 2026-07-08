#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

/* 方向控制引脚 */
#define LEFT_AIN1_PORT      GPIOC
#define LEFT_AIN1_PIN       GPIO_PIN_0
#define LEFT_AIN2_PORT      GPIOC
#define LEFT_AIN2_PIN       GPIO_PIN_1

#define RIGHT_BIN1_PORT     GPIOC
#define RIGHT_BIN1_PIN      GPIO_PIN_2
#define RIGHT_BIN2_PORT     GPIOC
#define RIGHT_BIN2_PIN      GPIO_PIN_3

/* PWM 定时器，由 CubeMX 生成 htim3 */
extern TIM_HandleTypeDef htim3;
#define MOTOR_TIM           htim3
#define LEFT_PWM_CHANNEL    TIM_CHANNEL_1   /* PA6 */
#define RIGHT_PWM_CHANNEL   TIM_CHANNEL_2   /* PA7 */

#define PWM_MAX             999

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           pwm_channel;
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;
} Motor_t;

extern Motor_t left_motor;
extern Motor_t right_motor;

void Motor_Init(void);
void Motor_SetSpeed(Motor_t *motor, int16_t speed);  /* -999 ~ 999，正值前进 */
void Motor_Stop(Motor_t *motor);
void Motor_StopAll(void);

#endif /* __MOTOR_H */
