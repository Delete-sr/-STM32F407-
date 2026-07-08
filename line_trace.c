#include "line_trace.h"

void LineTrace_Init(void)
{
    Motor_Init();
    Sensor_Init();
}

/**
 * @brief 每次主循环调用一次，根据传感器状态调整电机速度
 *
 * 传感器编码（1=在黑线上）：
 *   l1 l2 r1 r2
 *   偏右 → 左转，偏左 → 右转，居中 → 直行
 */
void LineTrace_Run(void)
{
    SensorData_t s = Sensor_Read();

    /* 全部离线：直行 */
    if (!s.l1 && !s.l2 && !s.r1 && !s.r2) {
        Motor_SetSpeed(&left_motor,  BASE_SPEED - LEFT_TRIM);
        Motor_SetSpeed(&right_motor, BASE_SPEED - RIGHT_TRIM);
        return;
    }

    /* 全部在线：停车 */
    if (s.l1 && s.l2 && s.r1 && s.r2) {
        Motor_StopAll();
        return;
    }

    /* 计算偏差：右侧权重为正，左侧为负 */
    int16_t error = (int16_t)(s.r1 * 1 + s.r2 * 2) - (int16_t)(s.l2 * 1 + s.l1 * 2);

    int16_t left_speed;
    int16_t right_speed;

    if (error == 0) {
        /* 居中，直行 */
        left_speed  = BASE_SPEED;
        right_speed = BASE_SPEED;
    } else if (error > 0) {
        /* 偏右，左转 */
        int16_t correction = (error == 1) ? TURN_CORRECTION : SHARP_CORRECTION;
        left_speed  = BASE_SPEED + correction;
        right_speed = BASE_SPEED - correction;
    } else {
        /* 偏左，右转 */
        int16_t correction = (error == -1) ? TURN_CORRECTION : SHARP_CORRECTION;
        left_speed  = BASE_SPEED - correction;
        right_speed = BASE_SPEED + correction;
    }

    /* 两轮独立补偿：哪边偏快就调大哪边的 TRIM */
    left_speed  -= LEFT_TRIM;
    right_speed -= RIGHT_TRIM;

    /* 限幅 */
    if (left_speed  > PWM_MAX)  left_speed  = PWM_MAX;
    if (right_speed > PWM_MAX)  right_speed = PWM_MAX;
    if (left_speed  < 0)        left_speed  = 0;
    if (right_speed < 0)        right_speed = 0;

    Motor_SetSpeed(&left_motor,  left_speed);
    Motor_SetSpeed(&right_motor, right_speed);
}

void LineTrace_Stop(void)
{
    Motor_StopAll();
}
