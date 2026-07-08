#include "motor.h"

Motor_t left_motor;
Motor_t right_motor;

void Motor_Init(void)
{
    /* 绑定左电机 */
    left_motor.htim        = &MOTOR_TIM;
    left_motor.pwm_channel = LEFT_PWM_CHANNEL;
    left_motor.in1_port    = LEFT_AIN1_PORT;
    left_motor.in1_pin     = LEFT_AIN1_PIN;
    left_motor.in2_port    = LEFT_AIN2_PORT;
    left_motor.in2_pin     = LEFT_AIN2_PIN;

    /* 绑定右电机 */
    right_motor.htim        = &MOTOR_TIM;
    right_motor.pwm_channel = RIGHT_PWM_CHANNEL;
    right_motor.in1_port    = RIGHT_BIN1_PORT;
    right_motor.in1_pin     = RIGHT_BIN1_PIN;
    right_motor.in2_port    = RIGHT_BIN2_PORT;
    right_motor.in2_pin     = RIGHT_BIN2_PIN;

    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&MOTOR_TIM, LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&MOTOR_TIM, RIGHT_PWM_CHANNEL);

    Motor_StopAll();
}

/**
 * @brief 设置电机速度
 * @param motor  目标电机
 * @param speed  -999 ~ 999，正值前进，负值后退
 */
void Motor_SetSpeed(Motor_t *motor, int16_t speed)
{
    if (speed > PWM_MAX)  speed = PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed > 0) {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
        speed = -speed;
    } else {
        /* 刹车 */
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, (uint32_t)speed);
}

void Motor_Stop(Motor_t *motor)
{
    Motor_SetSpeed(motor, 0);
}

void Motor_StopAll(void)
{
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);
}
