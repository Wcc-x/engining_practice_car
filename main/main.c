/**
 * @file    main.c
 * @brief   麦轮机器人主控 — 50Hz 闭环运动控制 + BLE 蓝牙遥控
 *
 * 控制来源:
 *   - 蓝牙串口小程序 (摇杆/滑杆/按键)
 *   - 或预设目标速度 (无蓝牙连接时)
 *
 * 蓝牙数据包格式:
 *   [joystick, Vx, Vy, servo1_angle, servo2_angle]
 *     Vx → 机器人前进/后退速度
 *     Vy → 机器人左移/右移速度
 *     servo1_angle / servo2_angle → 舵机角度
 *
 * 调用 AX_ROBOT_Kinematics() 走完整闭环:
 *   编码器(PCNT) → 正解 → 限幅 → 逆解 → PID → PWM(IN1/IN2 H桥)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ax_kinematics.h"
#include "ax_ble.h"
#include "ax_speed.h"

#define JOYSTICK_MAX        100
#define JOYSTICK_DEADZONE   5
#define BLE_TIMEOUT_MS      2000

static const char *TAG = "MAIN";
static int64_t last_ble_data_time = 0;
static int64_t last_print_time = 0;
static bool    last_conn_state = false;

static inline int16_t deadzone(int16_t val, int16_t dz)
{
    if (val > -dz && val < dz) return 0;
    return val;
}

static void map_joystick_to_speed(AX_BLE_Joystick *js,
                                   int16_t *vx, int16_t *vy)
{
    int16_t jx = deadzone(js->vx, JOYSTICK_DEADZONE);
    int16_t jy = deadzone(js->vy, JOYSTICK_DEADZONE);
    *vx = (int16_t)((int32_t)jx * R_VX_LIMIT / JOYSTICK_MAX);
    *vy = (int16_t)((int32_t)jy * R_VY_LIMIT / JOYSTICK_MAX);
}

void app_main(void)
{
    AX_HAL_Init();
    AX_ROBOT_Stop();

    AX_BLE_Init("XTARK-ESP32");

    AX_ROBOT_SetSpeed(0, 0, 0);

    /* ★ 修复3: Init 完成后才开始计时 */
    last_ble_data_time = esp_timer_get_time();

    printf("\n===== BLE Monitor Start =====\n");
    printf("Waiting for BLE connection & data...\n");
    printf("================================\n\n");

    while (1) {
        int64_t t0 = esp_timer_get_time();

        /* ── 打印蓝牙连接状态变化 ── */
        if (ax_ble_connected != last_conn_state) {
            last_conn_state = ax_ble_connected;
            printf("[BLE] Connection: %s\n", ax_ble_connected ? "CONNECTED ✓" : "DISCONNECTED ✗");
        }

        /* ── 打印原始接收数据 (原子读取后清零) ── */
        {
            uint16_t rx_len = ax_ble_rx_len;
            if (rx_len > 0) {
                printf("[BLE] RAW (%d bytes): ", rx_len);
                for (uint16_t i = 0; i < rx_len && i < 64; i++) {
                    uint8_t ch = ax_ble_rx_buf[i];
                    if (ch >= 0x20 && ch <= 0x7E) printf("%c", (char)ch);
                    else printf("<0x%02X>", ch);
                }
                printf("\n");
                ax_ble_rx_len = 0;  /* 消费后清零，避免重复打印 */
            }
        }

        /* ★ 修复2: 临界区保护 volatile 结构体拷贝 */
        bool has_data = false;
        AX_BLE_Joystick js_copy;

        if (ax_ble_joystick.valid) {
            js_copy  = ax_ble_joystick;
            ax_ble_joystick.valid = false;
            has_data = true;
        }

        if (has_data) {
            printf("[BLE] Joystick: Vx=%d Vy=%d Servo1=%d° Servo2=%d°\n",
                   js_copy.vx, js_copy.vy,
                   js_copy.servo1_angle, js_copy.servo2_angle);

            int16_t vx, vy;
            map_joystick_to_speed(&js_copy, &vx, &vy);
            printf("[CTRL] Target Speed: Vx=%d Vy=%d (×0.001 m/s)\n", vx, vy);

            AX_ROBOT_SetSpeed(vx, vy, 0);
            AX_SERVO_S1_SetAngle(js_copy.servo1_angle * 10);
            AX_SERVO_S2_SetAngle(js_copy.servo2_angle * 10);
            last_ble_data_time = t0;
        }

        /* 超时保护 */
        {
            int64_t elapsed = (t0 - last_ble_data_time) / 1000;
            if (elapsed > BLE_TIMEOUT_MS) {
                AX_ROBOT_SetSpeed(0, 0, 0);
            }
        }

        /* ★ 修复1: 先采样编码器，再做运动学计算 */
        AX_ENCODER_UpdateAll();
        AX_ROBOT_Kinematics();

        /* ── 每秒打印一次状态心跳 ── */
        if ((t0 - last_print_time) > 1000000) {
            last_print_time = t0;
            int since_last_data_ms = (int32_t)((t0 - last_ble_data_time) / 1000);
            printf("[HB] BLE=%s | MoveEn=%d | LastData=%dms | TG Vx=%d Vy=%d Vw=%d\n",
                   ax_ble_connected ? "ON" : "OFF",
                   ax_robot_move_enable,
                   since_last_data_ms,
                   R_Vel.TG_IX, R_Vel.TG_IY, R_Vel.TG_IW);
            printf("[HB] Real Vx=%d Vy=%d Vw=%d | GAP evt#=%lu last=%d\n",
                   R_Vel.RT_IX, R_Vel.RT_IY, R_Vel.RT_IW,
                   (unsigned long)ax_ble_gap_evt_cnt, ax_ble_gap_last_evt);
            printf("[HB] RPM A=%.0f B=%.0f C=%.0f D=%.0f | PWM A=%d B=%d C=%d D=%d\n",
                   AX_ENCODER_A_GetSpeedRPM(), AX_ENCODER_B_GetSpeedRPM(),
                   AX_ENCODER_C_GetSpeedRPM(), AX_ENCODER_D_GetSpeedRPM(),
                   R_Wheel_A.PWM, R_Wheel_B.PWM, R_Wheel_C.PWM, R_Wheel_D.PWM);
            printf("[HB] Wheel TG A=%.3f B=%.3f C=%.3f D=%.3f (m/s) | RT A=%.3f B=%.3f C=%.3f D=%.3f\n",
                   (double)R_Wheel_A.TG, (double)R_Wheel_B.TG, (double)R_Wheel_C.TG, (double)R_Wheel_D.TG,
                   R_Wheel_A.RT, R_Wheel_B.RT, R_Wheel_C.RT, R_Wheel_D.RT);
        }

        int32_t d = (1000 / PID_RATE) -
                    (int32_t)((esp_timer_get_time() - t0) / 1000);
        if (d > 0) vTaskDelay(pdMS_TO_TICKS(d));
    }
}