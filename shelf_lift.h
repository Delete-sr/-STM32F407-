#ifndef __SHELF_LIFT_H
#define __SHELF_LIFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 多层货架升降控制模块
 *
 * ESP32 是大脑，STM32 是手脚：
 *   ESP32 → "LIFT_START"  → STM32 驱动丝杆上升
 *   ESP32 → "LIFT_STOP"   → STM32 停止上升
 *   ESP32 → "LOWER_START" → STM32 降到原位
 *   STM32 → "LOWER_DONE"  → 下降完成, 通知ESP32
 */

/* 上升速度 (Hz), 8000Hz 约 40mm/s */
#define LIFT_SPEED_HZ       8000

/* 下降速度 */
#define LOWER_SPEED_HZ      8000

/* 上升无上限, 靠 ESP32 手动喊停 */
#define LIFT_FULL_STEPS      500000
/* 下降无上限, 靠 ESP32 扫码喊停 */
#define LOWER_FULL_STEPS     500000

/* 状态 */
typedef enum {
    LIFT_IDLE,          /* 空闲 */
    LIFT_GOING_UP,      /* 正在上升 */
    LIFT_GOING_DOWN,    /* 正在下降 */
    LIFT_STOPPED,       /* 已停止(停在当前高度) */
} LiftState_t;

/* 对外接口 */
void ShelfLift_Init(void);
void ShelfLift_StartUp(void);       /* 开始上升 */
void ShelfLift_Stop(void);          /* 停止 */
void ShelfLift_StartDown(void);     /* 开始下降到底 */
void ShelfLift_ForceDown(void);     /* 强制归零(无视当前位置, 直接降LIFT_FULL_STEPS步) */
void ShelfLift_Run(void);           /* 主循环轮询 */
LiftState_t ShelfLift_GetState(void);
uint8_t ShelfLift_IsDone(void);     /* 下降完成? */

#ifdef __cplusplus
}
#endif

#endif
