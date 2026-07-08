#include "sensor.h"

void Sensor_Init(void)
{
    /* 引脚由 CubeMX 生成的 MX_GPIO_Init() 初始化，此处无需额外操作 */
}

SensorData_t Sensor_Read(void)
{
    SensorData_t data;

    /* GPIO_PIN_RESET = 检测到黑线（低电平有效） */
    data.l1 = (HAL_GPIO_ReadPin(SENSOR_L1_PORT, SENSOR_L1_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
    data.l2 = (HAL_GPIO_ReadPin(SENSOR_L2_PORT, SENSOR_L2_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
    data.r1 = (HAL_GPIO_ReadPin(SENSOR_R1_PORT, SENSOR_R1_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
    data.r2 = (HAL_GPIO_ReadPin(SENSOR_R2_PORT, SENSOR_R2_PIN) == GPIO_PIN_RESET) ? 1u : 0u;

    return data;
}

