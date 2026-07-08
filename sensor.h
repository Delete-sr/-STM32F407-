#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32f4xx_hal.h"

/* 4路循迹传感器，HIGH = 检测到黑线 */
#define SENSOR_L1_PORT      GPIOB
#define SENSOR_L1_PIN       GPIO_PIN_0   /* 最左 */
#define SENSOR_L2_PORT      GPIOB
#define SENSOR_L2_PIN       GPIO_PIN_1   /* 次左 */
#define SENSOR_R1_PORT      GPIOB
#define SENSOR_R1_PIN       GPIO_PIN_2   /* 次右 */
#define SENSOR_R2_PORT      GPIOB
#define SENSOR_R2_PIN       GPIO_PIN_3   /* 最右 */

typedef struct {
    uint8_t l1;   /* 1 = 检测到黑线 */
    uint8_t l2;
    uint8_t r1;
    uint8_t r2;
} SensorData_t;

void Sensor_Init(void);
SensorData_t Sensor_Read(void);

#endif /* __SENSOR_H */
