/**
 * @file    main.c
 * @brief   麦轮机器人主控 — 50Hz 闭环运动控制 + BLE 蓝牙遥控
 *
 * 控制来源:
 *   - 江协科技蓝牙串口微信小程序 (摇杆/滑杆/按键)
 *   - 或预设目标速度 (无蓝牙连接时)
 *
 * 摇杆→小车控制映射:
 *   左摇杆 Y (纵向)  → Vx (前进/后退)
 *   左摇杆 X (横向)  → Vy (左移/右移)
 *   右摇杆 X (横向)  → Vw (旋转)
 *
 * 调用 AX_ROBOT_Kinematics() 走完整闭环:
 *   编码器(PCNT) → 正解 → 限幅 → 逆解 → PID → PWM(IN1/IN2 H桥)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "ax_robot.h"
#include "ax_kinematics.h"
#include "ax_ble.h"

/* ============================================================
 *  摇杆量程映射
 * ============================================================
 *  小程序摇杆值范围取决于用户在"摇杆设置"中配置的"横向最大值"和"纵向最大值"。
 *  默认推荐配置为 ±100 (可在小程序中调整)。
 *  JOYSTICK_MAX 应与小程序中设置的最大值一致。
 */
#define JOYSTICK_MAX        100     /* 摇杆最大值 (需与小程序配置一致) */
#define JOYSTICK_DEADZONE   5       /* 死区 (绝对值小于此值视为零) */

/* ============================================================
 *  超时保护 — 蓝牙断连后自动停车
 * ============================================================ */
#define BLE_TIMEOUT_MS      2000    /* 超过此时间无新数据则停车 */

static int64_t last_ble_data_time = 0;

/* ============================================================
 *  死区处理
 * ============================================================ */
static inline int16_t deadzone(int16_t val, int16_t dz)
{
    if (val > -dz && val < dz) return 0;
    return val;
}

/* ============================================================
 *  摇杆值 → 目标速度映射
 * ============================================================ */
static void map_joystick_to_speed(AX_BLE_Joystick *js,
                                   int16_t *vx, int16_t *vy, int16_t *vw)
{
    /* 去死区 */
    int16_t ly = deadzone(js->left_y,  JOYSTICK_DEADZONE);
    int16_t lx = deadzone(js->left_x,  JOYSTICK_DEADZONE);
    int16_t rx = deadzone(js->right_x, JOYSTICK_DEADZONE);

    /* 线性映射到速度范围
     *   Vx = left_y  × (R_VX_LIMIT / JOYSTICK_MAX)  前进/后退
     *   Vy = left_x  × (R_VY_LIMIT / JOYSTICK_MAX)  左移/右移
     *   Vw = right_x × (R_VW_LIMIT / JOYSTICK_MAX)  旋转
     */
    *vx = (int16_t)((int32_t)ly * R_VX_LIMIT / JOYSTICK_MAX);
    *vy = (int16_t)((int32_t)lx * R_VY_LIMIT / JOYSTICK_MAX);
    *vw = (int16_t)((int32_t)rx * R_VW_LIMIT / JOYSTICK_MAX);
}

void app_main(void)
{
    AX_HAL_Init();
    AX_ROBOT_Stop();

    /* 初始化 BLE (广播名称 "XTARK-ESP32") */
    AX_BLE_Init("XTARK-ESP32");

    /* 默认静止 */
    AX_ROBOT_SetSpeed(0, 0, 0);

    last_ble_data_time = esp_timer_get_time();

    /* ---- 50Hz 闭环 ---- */
    while (1) {
        int64_t t0 = esp_timer_get_time();

        /* ---- 检查 BLE 数据 ---- */
        if (ax_ble_joystick.valid) {
            ax_ble_joystick.valid = false;  /* 消费数据 */

            int16_t vx, vy, vw;
            map_joystick_to_speed(&ax_ble_joystick, &vx, &vy, &vw);
            AX_ROBOT_SetSpeed(vx, vy, vw);

            last_ble_data_time = t0;
        }

        /* ---- 超时保护: 蓝牙断连 → 停车 ---- */
        if (!ax_ble_connected) {
            int64_t elapsed = (t0 - last_ble_data_time) / 1000;  /* ms */
            if (elapsed > BLE_TIMEOUT_MS && elapsed < (BLE_TIMEOUT_MS + 100)) {
                /* 仅触发一次停车 */
                AX_ROBOT_SetSpeed(0, 0, 0);
            } else if (elapsed > BLE_TIMEOUT_MS + 100) {
                AX_ROBOT_SetSpeed(0, 0, 0);
            }
        }

        /* 6步闭环: 编码器→正解→限幅→逆解→PID→PWM输出 */
        AX_ROBOT_Kinematics();

        int32_t d = (1000 / PID_RATE) - (int32_t)((esp_timer_get_time() - t0) / 1000);
        if (d > 0) vTaskDelay(pdMS_TO_TICKS(d));
    }
}
