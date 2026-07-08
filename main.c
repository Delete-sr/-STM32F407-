/**
 * main.c — 产线物料巡检小车 STM32F407 主程序 (精简版)
 *
 * 本文件仅负责硬件初始化和主循环调度。
 * 所有业务逻辑 (UART协议、状态机、巡线、升降) 均在 robot_ctrl.c 中。
 *
 * 使用方法:
 *   1. 用本文件替换 Core/Src/main.c
 *   2. 将 robot_ctrl.c / robot_ctrl.h 加入 MDK-ARM 工程
 *   3. 从工程中移除旧的 main.c 中重复的 HAL_UART_RxCpltCallback
 *      (本文件不再定义该回调, 统一由 robot_ctrl.c 提供)
 */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "robot_ctrl.h"

/* ========================= 系统时钟 ========================= */

void SystemClock_Config(void);

/* ========================= 主函数 ========================= */

int main(void)
{
    /* HAL 库初始化 */
    HAL_Init();

    /* 系统时钟 168 MHz */
    SystemClock_Config();

    /* 外设初始化 (由 CubeMX 生成) */
    MX_GPIO_Init();
    MX_TIM3_Init();          /* 直流电机 PWM */
    MX_USART1_UART_Init();   /* 与 ESP32 通信 */
    MX_TIM4_Init();          /* 步进电机 PWM */

    /* 上电延迟 6 秒，等 ESP32 启动 + 连 WiFi */
    HAL_Delay(6000);

    /* 初始化所有业务模块 (RobotCtrl_Init 内部会多次发 RESET 给 ESP32) */
    RobotCtrl_Init();

    /* 主循环: 仅调度, 不含业务逻辑 */
    while (1)
    {
        RobotCtrl_Task();
    }
}

/* ========================= 系统时钟配置 ========================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================= 错误处理 ========================= */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 用户可在此添加断言失败处理 */
}
#endif /* USE_FULL_ASSERT */

/* ========================= 步进电机 PWM 回调 ========================= */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    Stepper_TIM_PWM_Callback(htim);
}
