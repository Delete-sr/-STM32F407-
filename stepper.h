#ifndef __STEPPER_H
#define __STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include <stdint.h>

#define STEPPER_MOTOR_STEPS_PER_REV     200.0f
#define STEPPER_MICROSTEP               8.0f

/*
    根据你的丝杆修改这个参数。

    常见 T8 丝杆：
    T8x8：导程 8mm
    T8x4：导程 4mm
    T8x2：导程 2mm

    如果你不确定，先写 8.0f，后面实测修正。
*/
#define STEPPER_LEAD_MM_PER_REV         8.0f

#define STEPPER_PULSE_PER_REV           (STEPPER_MOTOR_STEPS_PER_REV * STEPPER_MICROSTEP)
#define STEPPER_PULSE_PER_MM            (STEPPER_PULSE_PER_REV / STEPPER_LEAD_MM_PER_REV)

/*
    根据你在 CubeMX 中选择的定时器和通道
*/
#define STEPPER_TIM                     htim4
#define STEPPER_TIM_CHANNEL             TIM_CHANNEL_1

/*
    DIR 引脚：PE0
*/
#define STEPPER_DIR_GPIO_Port           GPIOE
#define STEPPER_DIR_Pin                 GPIO_PIN_0

/*
    ENA 引脚：PE1
*/
#define STEPPER_ENA_GPIO_Port           GPIOE
#define STEPPER_ENA_Pin                 GPIO_PIN_1

/*
    ENA 极性 —— TB6600 是低有效!

    TB6600 接线: ENA+ → STM32 GPIO, ENA- → GND
    LOW  (0V)  = ENA+与ENA-无压差 = 光耦不导通 = TB6600 输出使能 ✓
    HIGH (3.3V)= ENA+与ENA-有压差 = 光耦导通     = TB6600 输出禁用

    如果你的 TB6600 用相反逻辑，把下面两个宏互换即可。
*/
#define STEPPER_ENABLE_LEVEL            GPIO_PIN_RESET   /* LOW  = 电机通电 */
#define STEPPER_DISABLE_LEVEL           GPIO_PIN_SET    /* HIGH = 电机断电 */

typedef enum
{
    STEPPER_DIR_FORWARD = 0,
    STEPPER_DIR_BACKWARD = 1
} StepperDir_t;

void Stepper_Init(void);
void Stepper_Enable(void);
void Stepper_Disable(void);

void Stepper_SetDir(StepperDir_t dir);
void Stepper_SetSpeedHz(uint32_t freq_hz);

void Stepper_MoveSteps(int32_t steps, uint32_t freq_hz);
void Stepper_MoveMM(float mm, uint32_t freq_hz);

uint8_t Stepper_IsBusy(void);
void Stepper_Stop(void);
void Stepper_Home(void);    /* 归零: 下降到底 */

void Stepper_TIM_PWM_Callback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif
