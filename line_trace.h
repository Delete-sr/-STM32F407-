#ifndef __LINE_TRACE_H
#define __LINE_TRACE_H

#include "motor.h"
#include "sensor.h"

/* 基础速度与转向调整量，可按实际小车调整 */
#define BASE_SPEED          250   /* 加重后需更大扭矩 */
#define TURN_CORRECTION     70   /* 轻微转向 */
#define SHARP_CORRECTION    200  /* 急转 */
/* 两轮独立补偿：哪边偏快就调大哪边的 TRIM（正数 = 降低该轮速度） */
#define LEFT_TRIM           50    /* 左轮当前不快 */
#define RIGHT_TRIM          60   /* 右轮偏快，从 50 开始试，不行加到 100 */

void LineTrace_Init(void);
void LineTrace_Run(void);
void LineTrace_Stop(void);

#endif /* __LINE_TRACE_H */
