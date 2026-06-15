/**
 * @file    main.c
 * @brief   麦轮机器人主控 — 50Hz 闭环运动控制
 *
 * 调用 AX_ROBOT_Kinematics() 走完整闭环:
 *   编码器(PCNT) → 正解 → 限幅 → 逆解 → PID → PWM(IN1/IN2 H桥)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "ax_robot.h"
#include "ax_kinematics.h"

void app_main(void)
{
    AX_HAL_Init();
    AX_ROBOT_Stop();

    /* ---- 示例: 写入目标速度 ---- */
    AX_ROBOT_SetSpeed(500, 0, 0);   /* X方向 0.5m/s */
    /* AX_ROBOT_SetSpeed(0, 0, 1000);   旋转 1rad/s */
    /* AX_ROBOT_SetSpeed(0, 500, 0);    左移 0.5m/s */

    /* ---- 50Hz 闭环 ---- */
    while (1) {
        int64_t t0 = esp_timer_get_time();

        /* 6步闭环: 编码器→正解→限幅→逆解→PID→PWM输出 */
        AX_ROBOT_Kinematics();

        int32_t d = (1000 / PID_RATE) - (int32_t)((esp_timer_get_time() - t0) / 1000);
        if (d > 0) vTaskDelay(pdMS_TO_TICKS(d));
    }
}
