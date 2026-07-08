#include "stepper.h"
#include <stdlib.h>

static volatile int32_t stepper_target_pulses = 0;
static volatile int32_t stepper_pulse_count = 0;
static volatile uint8_t stepper_busy = 0;
static volatile int32_t stepper_abs_position = 0;  /* 绝对位置 (脉冲数) */
static volatile int8_t  stepper_current_dir = 0;   /* 当前方向: 1=正, -1=反 */

/* 加速斜坡参数 */
#define ACCEL_STEPS     200      /* 加速段脉冲数 */
#define START_FREQ_HZ   150      /* 起步频率 (Hz) */
#define ACCEL_INTERVAL  10       /* 每隔多少个脉冲提一次速 */

static volatile uint32_t stepper_current_freq;
static volatile uint32_t stepper_target_freq;
static volatile int32_t  stepper_accel_count;

void Stepper_Init(void)
{
    Stepper_Disable();

    HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_RESET);

    /*
        默认设置 1000Hz。
        实际运动时 Stepper_MoveSteps / Stepper_MoveMM 会重新设置速度。
    */
    Stepper_SetSpeedHz(1000);
}

void Stepper_Enable(void)
{
    HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, STEPPER_ENABLE_LEVEL);
}

void Stepper_Disable(void)
{
    HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, STEPPER_DISABLE_LEVEL);
}

void Stepper_SetDir(StepperDir_t dir)
{
    if (dir == STEPPER_DIR_FORWARD)
    {
        HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_SET);
    }

    /*
        给方向信号一点建立时间。
        大多数驱动器要求 DIR 在 PUL 之前稳定几微秒。
    */
    for (volatile int i = 0; i < 1000; i++);
}

void Stepper_SetSpeedHz(uint32_t freq_hz)
{
    /*
        TIM4 已经在 CubeMX 中配置为 1MHz 计数频率。
        所以：
        PWM频率 = 1000000 / (ARR + 1)

        ARR = 1000000 / freq_hz - 1
    */

    if (freq_hz < 1)
    {
        freq_hz = 1;
    }

    /*
        不建议一开始太快。
        对丝杆来说 500Hz ~ 3000Hz 比较适合调试。
    */
    if (freq_hz > 50000)
    {
        freq_hz = 50000;
    }

    uint32_t arr = 1000000UL / freq_hz;

    if (arr < 2)
    {
        arr = 2;
    }

    arr = arr - 1;

    __HAL_TIM_SET_AUTORELOAD(&STEPPER_TIM, arr);
    __HAL_TIM_SET_COMPARE(&STEPPER_TIM, STEPPER_TIM_CHANNEL, (arr + 1) / 2);

    /*
        更新计数器，避免修改 ARR 后立即出现异常周期
    */
    __HAL_TIM_SET_COUNTER(&STEPPER_TIM, 0);
}

void Stepper_MoveSteps(int32_t steps, uint32_t freq_hz)
{
    if (steps == 0)
    {
        return;
    }

    /*
        如果上一次运动没结束，先停掉。
    */
    if (stepper_busy)
    {
        Stepper_Stop();
    }

    if (steps > 0)
    {
        Stepper_SetDir(STEPPER_DIR_FORWARD);
        stepper_target_pulses = steps;
        stepper_current_dir = 1;
    }
    else
    {
        Stepper_SetDir(STEPPER_DIR_BACKWARD);
        stepper_target_pulses = -steps;
        stepper_current_dir = -1;
    }

    stepper_pulse_count = 0;
    stepper_busy = 1;
    stepper_target_freq = freq_hz;

    /*
        加速斜坡: 从 START_FREQ_HZ 起步，
        每隔 ACCEL_INTERVAL 个脉冲提一点速，
        用 ACCEL_STEPS 个脉冲加速到目标频率。
    */
    stepper_accel_count = 0;
    stepper_current_freq = START_FREQ_HZ;

    if (stepper_current_freq > stepper_target_freq)
    {
        stepper_current_freq = stepper_target_freq;
    }

    Stepper_SetSpeedHz(stepper_current_freq);
    Stepper_Enable();

    __HAL_TIM_SET_COUNTER(&STEPPER_TIM, 0);

    /*
        启动 PWM，并开启中断。
        每个 PWM 周期会进入一次 HAL_TIM_PWM_PulseFinishedCallback。
    */
    HAL_TIM_PWM_Start_IT(&STEPPER_TIM, STEPPER_TIM_CHANNEL);
}

void Stepper_MoveMM(float mm_abs, uint32_t freq_hz)
{
    /*
        绝对定位: 移动到距离底部 mm_abs 的位置。
        先计算目标脉冲数,再算差值,最后相对移动。
    */
    int32_t target_pulses = (int32_t)(mm_abs * STEPPER_PULSE_PER_MM);
    int32_t delta = target_pulses - stepper_abs_position;

    Stepper_MoveSteps(delta, freq_hz);
}

uint8_t Stepper_IsBusy(void)
{
    return stepper_busy;
}

void Stepper_Stop(void)
{
    HAL_TIM_PWM_Stop_IT(&STEPPER_TIM, STEPPER_TIM_CHANNEL);

    stepper_busy = 0;
    stepper_target_pulses = 0;
    stepper_pulse_count = 0;
    stepper_current_dir = 0;
}

void Stepper_Home(void)
{
    /*
        归零: 向下降回绝对位置 0。
        如果当前位置已经是 0, 不动。
    */
    if (stepper_abs_position <= 0) {
        /* 已经在底部 */
        stepper_abs_position = 0;
        return;
    }

    /* 下降回到位置 0 */
    int32_t steps_down = stepper_abs_position;
    Stepper_MoveSteps(-steps_down, 300);  /* 低速下降, 300Hz */
}

void Stepper_TIM_PWM_Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if (stepper_busy)
        {
            stepper_pulse_count++;

            /* 更新绝对位置 */
            if (stepper_current_dir > 0) {
                stepper_abs_position++;
            } else {
                stepper_abs_position--;
            }

            /*
                加速斜坡逻辑:
                如果还没到目标频率，逐步提速。
            */
            if (stepper_current_freq < stepper_target_freq &&
                stepper_accel_count < ACCEL_STEPS)
            {
                stepper_accel_count++;
                if ((stepper_accel_count % ACCEL_INTERVAL) == 0)
                {
                    /* 线性提速 */
                    uint32_t range = stepper_target_freq - START_FREQ_HZ;
                    uint32_t step = range * stepper_accel_count / ACCEL_STEPS;
                    stepper_current_freq = START_FREQ_HZ + step;
                    if (stepper_current_freq > stepper_target_freq)
                    {
                        stepper_current_freq = stepper_target_freq;
                    }
                    Stepper_SetSpeedHz(stepper_current_freq);
                }
            }

            if (stepper_pulse_count >= stepper_target_pulses)
            {
                HAL_TIM_PWM_Stop_IT(&STEPPER_TIM, STEPPER_TIM_CHANNEL);

                stepper_busy = 0;
                stepper_target_pulses = 0;
                stepper_pulse_count = 0;

                /*
                    运动结束后保持使能，电机锁住位置。
                    如果想省电/减少发热，取消下面的注释：
                    Stepper_Disable();
                */
            }
        }
    }
}
