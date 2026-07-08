/**
 * robot_ctrl.c — 产线物料巡检小车 机器人控制模块 (合并版)
 *
 * 本文件整合了 UART 协议解析、状态机、巡线控制 与 货架升降。
 * 替代旧版 main.c 中的散落逻辑，与 ESP32 及 Flask 上位机配合使用。
 *
 * 通信协议 (UART1, 115200 8N1):
 *   ESP32 → STM32:  QR_FOUND:<货架号>           — 发现二维码，停车
 *   STM32 → ESP32:  DETECT:<货架号>             — 请求拍照识别
 *   ESP32 → STM32:  RESULT:<货架>:<cls1>=<n1>,... — 识别结果 (所有类别)
 *   ESP32 → STM32:  DETECT_FAIL:<货架>          — 识别失败
 *   ESP32 → STM32:  LIFT_START                  — 丝杆上升
 *   ESP32 → STM32:  LIFT_STOP                   — 丝杆停止
 *   ESP32 → STM32:  LOWER_START                 — 丝杆下降归零
 *   STM32 → ESP32:  RESET                       — 通知 ESP32 状态复位
 */

#include "robot_ctrl.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "line_trace.h"
#include "shelf_lift.h"
#include "stepper.h"

/* ========================= 引脚 & 常量 ========================= */

#define BTN_START_GPIO_Port   GPIOA
#define BTN_START_Pin         GPIO_PIN_15

#define UART_LINE_BUF_LEN            128
#define SHELF_ID_LEN                 16

#define WAIT_RESULT_TIMEOUT_MS       100000
#define RX_RING_BUF_SIZE             256

/* ========================= 环形缓冲 (UART 接收) ========================= */

static uint8_t  g_rx_ring[RX_RING_BUF_SIZE];
static uint16_t g_rx_head = 0;
static uint16_t g_rx_tail = 0;
static uint8_t  g_rx_byte;

/* ========================= 状态机 ========================= */

typedef enum {
    ROBOT_STATE_WAIT_START = 0,   /* 上电等待按键启动 */
    ROBOT_STATE_FOLLOW_LINE,      /* 巡线中 */
    ROBOT_STATE_WAIT_RESULT       /* 等 ESP32 返回识别结果 */
} RobotState_t;

static RobotState_t g_robot_state = ROBOT_STATE_WAIT_START;

/* ========================= 数据缓存 ========================= */

typedef struct {
    char shelf[SHELF_ID_LEN];
    char counts_str[64];
} DetectInfo_t;

static char g_uart_line[UART_LINE_BUF_LEN];
static uint16_t g_uart_idx = 0;

static char     g_current_shelf[SHELF_ID_LEN];
static uint32_t g_wait_start_tick = 0;
static DetectInfo_t g_last_detect;

/* ========================= 前向声明 ========================= */

static void UART_SendLine(const char *s);
static void UART_ReadTask(void);
static void ProcessLine(const char *line);
static void HandleWaitStart(void);
static int  StartsWith(const char *str, const char *prefix);
static void SafeStrCopy(char *dst, const char *src, uint16_t dst_size);
static void ResetRuntimeState(void);
static void ResumeFollowLine(void);
static void StopAndRequestDetect(const char *shelf);
static int  ParseResultLine(const char *line, char *shelf, char *counts_str, uint16_t counts_size);
static int  ParseDetectFailLine(const char *line, char *shelf);

/* ========================= 初始化 ========================= */

void RobotCtrl_Init(void)
{
    LineTrace_Init();       /* 会调用 Motor_Init() + Sensor_Init() */
    Stepper_Init();
    ShelfLift_Init();

    ResetRuntimeState();

    /* 反复发送 RESET，确保 ESP32 无论何时就绪都能收到 */
    for (int i = 0; i < 5; i++) {
        HAL_Delay(500);
        UART_SendLine("RESET");
    }

    HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1);
}

/* ========================= 主循环任务 ========================= */

void RobotCtrl_Task(void)
{
    UART_ReadTask();

    /* ---- 等待按键启动 ---- */
    if (g_robot_state == ROBOT_STATE_WAIT_START) {
        HandleWaitStart();
        ShelfLift_Run();   /* 安全: 让升降状态机也转一下 */
        return;
    }

    /* ---- 电机控制: 升降时禁止巡线, 其他状态按状态机来 ---- */
    if (Stepper_IsBusy()) {
        LineTrace_Stop();
    } else if (g_robot_state == ROBOT_STATE_FOLLOW_LINE) {
        LineTrace_Run();
    } else {
        LineTrace_Stop();
    }

    /* ---- 升降状态机轮询 ---- */
    ShelfLift_Run();

    /* 下降到底后自动恢复巡线 */
    if (ShelfLift_IsDone()) {
        ResumeFollowLine();
    }

    /* ---- 超时保护 ---- */
    if (g_robot_state == ROBOT_STATE_WAIT_RESULT) {
        if (!Stepper_IsBusy() && (HAL_GetTick() - g_wait_start_tick) > WAIT_RESULT_TIMEOUT_MS) {
            ResumeFollowLine();
        }
    }
}

/* ========================= 内部函数 ========================= */

static void ResetRuntimeState(void)
{
    g_robot_state = ROBOT_STATE_FOLLOW_LINE;
    memset(g_uart_line, 0, sizeof(g_uart_line));
    g_uart_idx = 0;
    memset(g_current_shelf, 0, sizeof(g_current_shelf));
    memset(&g_last_detect, 0, sizeof(g_last_detect));
    g_last_detect.shelf[0] = '\0';
    g_last_detect.counts_str[0] = '\0';
    g_wait_start_tick = 0;
}

static void ResumeFollowLine(void)
{
    memset(g_current_shelf, 0, sizeof(g_current_shelf));
    g_robot_state = ROBOT_STATE_FOLLOW_LINE;
}

static void StopAndRequestDetect(const char *shelf)
{
    SafeStrCopy(g_current_shelf, shelf, sizeof(g_current_shelf));
    g_robot_state = ROBOT_STATE_WAIT_RESULT;
    g_wait_start_tick = HAL_GetTick();

    LineTrace_Stop();

    char tx[64];
    snprintf(tx, sizeof(tx), "DETECT:%s", g_current_shelf);
    UART_SendLine(tx);
}

/* ---- UART ---- */

static void UART_SendLine(const char *s)
{
    char txbuf[128];
    snprintf(txbuf, sizeof(txbuf), "%s\n", s);
    HAL_UART_Transmit(&huart1, (uint8_t *)txbuf, strlen(txbuf), 1000);
}

static void UART_ReadTask(void)
{
    while (g_rx_tail != g_rx_head) {
        uint8_t ch = g_rx_ring[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1) % RX_RING_BUF_SIZE;

        if (ch == '\r' || ch == '\n') {
            if (g_uart_idx > 0) {
                g_uart_line[g_uart_idx] = '\0';
                ProcessLine(g_uart_line);
                g_uart_idx = 0;
                memset(g_uart_line, 0, sizeof(g_uart_line));
            }
        } else {
            if (g_uart_idx < (UART_LINE_BUF_LEN - 1)) {
                g_uart_line[g_uart_idx++] = (char)ch;
            } else {
                /* 溢出丢弃 */
                g_uart_idx = 0;
                memset(g_uart_line, 0, sizeof(g_uart_line));
            }
        }
    }
}

/* ---- 指令解析 ---- */

static void ProcessLine(const char *line)
{
    char shelf[SHELF_ID_LEN] = {0};
    char counts_str[64]      = {0};

    /* QR_FOUND: 扫描到货架二维码 */
    if (StartsWith(line, "QR_FOUND:")) {
        HAL_GPIO_TogglePin(LED_DBG_GPIO_Port, LED_DBG_Pin);
        if (g_robot_state == ROBOT_STATE_FOLLOW_LINE) {
            const char *p = line + strlen("QR_FOUND:");
            if (strlen(p) > 0) {
                StopAndRequestDetect(p);
            }
        }
        return;
    }

    /* RESULT: 检测结果 格式 RESULT:shelf:cls1=n1,cls2=n2 */
    if (StartsWith(line, "RESULT:")) {
        if (ParseResultLine(line, shelf, counts_str, sizeof(counts_str))) {
            SafeStrCopy(g_last_detect.shelf, shelf, sizeof(g_last_detect.shelf));
            /* 收到结果后等 ESP32 发 LIFT_START (或超时自动恢复巡线) */
        }
        return;
    }

    /* DETECT_FAIL: 识别失败 */
    if (StartsWith(line, "DETECT_FAIL:")) {
        if (ParseDetectFailLine(line, shelf)) {
            ResumeFollowLine();
        } else {
            ResumeFollowLine();
        }
        return;
    }

    /* 升降指令 (ESP32 多层货架控制) */
    if (StartsWith(line, "LIFT_START")) {
        ShelfLift_StartUp();
        return;
    }

    if (StartsWith(line, "LIFT_STOP")) {
        LiftState_t prev = ShelfLift_GetState();
        ShelfLift_Stop();
        /* 下降时被喊停 = 归位完成, 恢复巡线 */
        if (prev == LIFT_GOING_DOWN) {
            ResumeFollowLine();
        }
        return;
    }

    if (StartsWith(line, "LOWER_START")) {
        ShelfLift_StartDown();
        return;
    }
}

/* ---- 按键启动 ---- */

#define BTN_PRESSED()  (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET)

static void HandleWaitStart(void)
{
    static uint32_t s_blink_tick = 0;
    static uint32_t s_debounce_tick = 0;
    static uint8_t  s_btn_last = 0;

    /* LED 慢闪: 等待中 */
    if ((HAL_GetTick() - s_blink_tick) >= 500) {
        HAL_GPIO_TogglePin(LED_DBG_GPIO_Port, LED_DBG_Pin);
        s_blink_tick = HAL_GetTick();
    }

    uint8_t btn_now = BTN_PRESSED() ? 1 : 0;

    if (btn_now && !s_btn_last) {
        s_debounce_tick = HAL_GetTick();
    }

    if (btn_now && s_btn_last) {
        if ((HAL_GetTick() - s_debounce_tick) >= 50) {
            /* 确认按下, 启动 */
            HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, GPIO_PIN_SET);
            g_robot_state = ROBOT_STATE_FOLLOW_LINE;
            UART_SendLine("RESET");
        }
    }

    s_btn_last = btn_now;
}

/* ---- 字符串工具 ---- */

static int StartsWith(const char *str, const char *prefix)
{
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

static void SafeStrCopy(char *dst, const char *src, uint16_t dst_size)
{
    if (dst == NULL || src == NULL || dst_size == 0) return;
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

/* ---- 协议解析 ---- */

static int ParseResultLine(const char *line, char *shelf, char *counts_str, uint16_t counts_size)
{
    /* 解析 "RESULT:shelf:cls1=n1,cls2=n2" → 取 shelf 和 counts 部分 */
    char buf[UART_LINE_BUF_LEN];
    SafeStrCopy(buf, line, sizeof(buf));

    /* 跳过 "RESULT:" */
    const char *p = buf;
    if (StartsWith(p, "RESULT:")) p += strlen("RESULT:");

    /* 找到第二个冒号 (shelf 结束处) */
    const char *colon = strchr(p, ':');
    if (!colon) return 0;

    uint16_t shelf_len = (uint16_t)(colon - p);
    if (shelf_len >= SHELF_ID_LEN) shelf_len = SHELF_ID_LEN - 1;
    memcpy(shelf, p, shelf_len);
    shelf[shelf_len] = '\0';

    /* 剩余部分就是 counts_str */
    SafeStrCopy(counts_str, colon + 1, counts_size);
    return 1;
}

static int ParseDetectFailLine(const char *line, char *shelf)
{
    char buf[UART_LINE_BUF_LEN];
    char *token;
    char *saveptr = NULL;
    int field = 0;

    memset(buf, 0, sizeof(buf));
    SafeStrCopy(buf, line, sizeof(buf));

    token = strtok_r(buf, ":", &saveptr);
    while (token != NULL) {
        if (field == 1) SafeStrCopy(shelf, token, SHELF_ID_LEN);
        field++;
        token = strtok_r(NULL, ":", &saveptr);
    }
    return (field >= 2) ? 1 : 0;
}

/* ========================= UART 接收回调 ========================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (g_rx_head + 1) % RX_RING_BUF_SIZE;
        if (next != g_rx_tail) {
            g_rx_ring[g_rx_head] = g_rx_byte;
            g_rx_head = next;
        }
        HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1);
    }
}
