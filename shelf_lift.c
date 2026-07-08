#include "shelf_lift.h"
#include "stepper.h"

static LiftState_t lift_state = LIFT_IDLE;

void ShelfLift_Init(void)
{
    lift_state = LIFT_IDLE;
}

void ShelfLift_StartUp(void)
{
    if (lift_state != LIFT_IDLE) return;

    /* 固定走满全行程上升, 靠 ESP32 喊停, 不依赖绝对位置 */
    Stepper_MoveSteps(LIFT_FULL_STEPS, LIFT_SPEED_HZ);
    lift_state = LIFT_GOING_UP;
}

void ShelfLift_Stop(void)
{
    if (lift_state != LIFT_GOING_UP && lift_state != LIFT_GOING_DOWN) return;

    Stepper_Stop();
    lift_state = LIFT_STOPPED;
}

void ShelfLift_StartDown(void)
{
    if (lift_state != LIFT_STOPPED && lift_state != LIFT_GOING_UP) return;

    if (lift_state == LIFT_GOING_UP) {
        Stepper_Stop();
    }

    /* 固定走满全行程下降, 不依赖绝对位置 */
    Stepper_MoveSteps(-LOWER_FULL_STEPS, LOWER_SPEED_HZ);
    lift_state = LIFT_GOING_DOWN;
}

/* 上电强制归零: 无视当前绝对位置, 直接向下走满全行程 */
void ShelfLift_ForceDown(void)
{
    Stepper_Stop();
    /* 负数 = 下降, Stepper_MoveSteps 内部自动设方向 */
    Stepper_MoveSteps(-LOWER_FULL_STEPS, LOWER_SPEED_HZ);
    lift_state = LIFT_GOING_DOWN;
}

LiftState_t ShelfLift_GetState(void)
{
    return lift_state;
}

static LiftState_t s_prev_lift = LIFT_IDLE;

uint8_t ShelfLift_IsDone(void)
{
    /* 下降完成: 自然走完 或 下降中被喊停 */
    if (!Stepper_IsBusy()) {
        if (lift_state == LIFT_GOING_DOWN ||
            (lift_state == LIFT_STOPPED && s_prev_lift == LIFT_GOING_DOWN)) {
            lift_state = LIFT_IDLE;
            s_prev_lift = LIFT_IDLE;
            return 1;
        }
    }
    s_prev_lift = lift_state;
    return 0;
}

void ShelfLift_Run(void)
{
    switch (lift_state) {

    case LIFT_GOING_UP:
        /* 上升中被 ESP32 喊停, Stepper_Stop 已经停了 */
        break;

    case LIFT_GOING_DOWN:
        /* 电机自己停了交给 IsDone() 处理 */
        break;

    default:
        break;
    }
}
