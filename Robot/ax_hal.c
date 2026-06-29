/**
 * @file    ax_hal.c
 * @brief   硬件抽象层实现 — IN1/IN2 H桥电机驱动 + PCNT编码器 + 舵机
 *
 * 严格遵循 BSP 库函数:
 *   - ledc.c:     ledc_Init() / ledc_pwm_set_duty()
 *   - pcnt_encoder.c: pcnt_encoder_init() / pcnt_encoder_get_pulse_count()
 *                     pcnt_encoder_update() / pcnt_encoder_reset() / pcnt_encoder_deinit()
 */

#include "ax_hal.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdlib.h>
#include "driver/ledc.h"   /* ledc_set_duty / ledc_update_duty — 直接写 raw ticks */

static const char *TAG = "AX_HAL";

/* ─────────────── BSP 实例指针 ─────────────── */

ledc_config_t *ax_motor_a_cfg;
ledc_config_t *ax_motor_b_cfg;
ledc_config_t *ax_motor_c_cfg;
ledc_config_t *ax_motor_d_cfg;

pcnt_encoder_config_t *ax_encoder_a_cfg;
pcnt_encoder_config_t *ax_encoder_b_cfg;
pcnt_encoder_config_t *ax_encoder_c_cfg;
pcnt_encoder_config_t *ax_encoder_d_cfg;

/* ─────────────── 舵机内部句柄 ─────────────── */

static ledc_config_t *servo_s1_cfg;
static ledc_config_t *servo_s2_cfg;

/* ─────────────── 内部: 电机 LEDC PWM 初始化 ─────────────── */

static ledc_config_t *motor_pwm_init(ledc_timer_t tmr, ledc_channel_t ch, int gpio)
{
    ledc_config_t *c = malloc(sizeof(ledc_config_t));
    c->clk_cfg         = LEDC_AUTO_CLK;
    c->timer_num       = tmr;
    c->freq_hz         = 1000;
    c->duty_resolution = LEDC_TIMER_14_BIT;
    c->channel         = ch;
    c->duty            = 0;
    c->gpio_num        = gpio;
    ledc_Init(c);                       /* ← BSP 库函数 */
    return c;
}

/* ─────────────── 内部: 单路电机驱动 (IN1/IN2 H桥) ───────────────
 *
 *  pwm > 0   正转: IN1=1, IN2=0
 *  pwm < 0   反转: IN1=0, IN2=1
 *  pwm = 0   停止: IN1=0, IN2=0
 * ─────────────────────────────────────────────────────────── */

/* ── motor_set_speed 诊断计数器 ── */
static int motor_dbg_cnt = 0;

static void motor_set_speed(ledc_config_t *cfg,
                            gpio_num_t in1, gpio_num_t in2, int16_t pwm)
{
    /* ★ 先记录 GPIO 电平 (在任何 set 之前) */
    int will_print = (motor_dbg_cnt < 4 && pwm != 0);
    int ib = 0, jb = 0;
    if (will_print) { ib = gpio_get_level(in1); jb = gpio_get_level(in2); }
    motor_dbg_cnt++;
    if (motor_dbg_cnt >= 200) motor_dbg_cnt = 0;

    int16_t pwm_abs = pwm;
    if (pwm > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (pwm < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        pwm_abs = (int16_t)(-pwm);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    }
    //映射
    uint16_t duty = (uint16_t)((uint32_t)pwm_abs * 100 / AX_MOTOR_PWM_MAX);
    if (duty > 100) duty = 100;

    if (will_print) {
        printf("[HAL] motor GPIO%02d/GPIO%02d: pwm=%d duty=%d%% IN(前=%d,%d → 后=%d,%d)\n",
               in1, in2, pwm, duty,
               ib, jb, gpio_get_level(in1), gpio_get_level(in2));
    }

    ledc_pwm_set_duty(cfg, duty);       /* ← BSP 库函数 */
}

/* ─────────────── 内部: 编码器初始化 ─────────────── */

static pcnt_encoder_config_t *enc_init(int a, int b)
{
    pcnt_encoder_config_t *e = malloc(sizeof(pcnt_encoder_config_t));
    e->a_gpio_num     = a;
    e->b_gpio_num     = b;
    e->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    e->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(e);               /* ← BSP 库函数 */
    return e;
}

/* ─────────────── 内部: 舵机初始化 (50Hz) ───────────────
 *
 *  使用 BSP ledc_Init() — 两个舵机共用 LEDC_TIMER_2，
 *  S1=LEDC_CHANNEL_5, S2=LEDC_CHANNEL_6。
 *  两次 ledc_Init() 调用会重复配置同一 timer（相同参数，无害），
 *  但 channel 和 gpio 不同。
 * ─────────────────────────────────────────────────────── */

static void servo_init(void)
{
    servo_s1_cfg = malloc(sizeof(ledc_config_t));
    servo_s1_cfg->clk_cfg         = LEDC_AUTO_CLK;
    servo_s1_cfg->timer_num       = LEDC_TIMER_2;
    servo_s1_cfg->freq_hz         = AX_SERVO_FREQ_HZ;
    servo_s1_cfg->duty_resolution = LEDC_TIMER_14_BIT;
    servo_s1_cfg->channel         = LEDC_CHANNEL_5;
    servo_s1_cfg->duty            = 0;
    servo_s1_cfg->gpio_num        = AX_SERVO_S1_GPIO;
    ledc_Init(servo_s1_cfg);            /* ← BSP 库函数 */

    servo_s2_cfg = malloc(sizeof(ledc_config_t));
    servo_s2_cfg->clk_cfg         = LEDC_AUTO_CLK;
    servo_s2_cfg->timer_num       = LEDC_TIMER_2;
    servo_s2_cfg->freq_hz         = AX_SERVO_FREQ_HZ;
    servo_s2_cfg->duty_resolution = LEDC_TIMER_14_BIT;
    servo_s2_cfg->channel         = LEDC_CHANNEL_6;
    servo_s2_cfg->duty            = 0;
    servo_s2_cfg->gpio_num        = AX_SERVO_S2_GPIO;
    ledc_Init(servo_s2_cfg);            /* ← BSP 库函数 */
}

static void servo_set_angle(ledc_config_t *cfg, int16_t raw)
{
    float deg = raw / 10.0f;
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;

    /*
     * 脉宽 → LEDC 硬件计数值 (14-bit 分辨率, 周期 = 2^14 = 16384 ticks)
     *
     * 50Hz → 1 tick = 20000μs / 16384 ≈ 1.22μs
     * 不走 BSP ledc_pwm_set_duty(百分比) 是因为百分比只有 ~100 档,
     * 对舵机 500~2500μs 范围 (仅占周期 10%) 分辨率不到 10 档, 误差 ±100μs。
     * 直接写 raw ticks 可获得满 14-bit 分辨率 (~0.1° 精度)。
     */
    float    us    = AX_SERVO_MIN_US + (deg / 180.0f) * (AX_SERVO_MAX_US - AX_SERVO_MIN_US);
    uint32_t ticks = (uint32_t)(us * 16384.0f / 20000.0f + 0.5f);
    if (ticks > 16384) ticks = 16384;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, cfg->channel, ticks);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, cfg->channel);
}

/* ═══════════════════════════════════════════════════════════
 *  HAL 公共接口
 * ═══════════════════════════════════════════════════════════ */

void AX_HAL_Init(void)
{
    /* ── IN1/IN2 GPIO 初始化 ── */
    ESP_LOGI(TAG, "Step 1/5: GPIO config...");
    uint64_t mask = (1ULL << AX_MOTOR_A_IN1_GPIO) | (1ULL << AX_MOTOR_A_IN2_GPIO) |
                    (1ULL << AX_MOTOR_B_IN1_GPIO) | (1ULL << AX_MOTOR_B_IN2_GPIO) |
                    (1ULL << AX_MOTOR_C_IN1_GPIO) | (1ULL << AX_MOTOR_C_IN2_GPIO) |
                    (1ULL << AX_MOTOR_D_IN1_GPIO) | (1ULL << AX_MOTOR_D_IN2_GPIO);
    gpio_config_t gpio_cfg = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = mask,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&gpio_cfg);

    /* 初始全低 */
    gpio_set_level(AX_MOTOR_A_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(AX_MOTOR_B_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_B_IN2_GPIO, 0);
    gpio_set_level(AX_MOTOR_C_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_C_IN2_GPIO, 0);
    gpio_set_level(AX_MOTOR_D_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_D_IN2_GPIO, 0);

    /* ★ GPIO 自检: 逐个引脚 set → read 验证 */
    {
        gpio_num_t test_pins[] = {
            AX_MOTOR_A_IN1_GPIO, AX_MOTOR_A_IN2_GPIO,
            AX_MOTOR_B_IN1_GPIO, AX_MOTOR_B_IN2_GPIO,
            AX_MOTOR_C_IN1_GPIO, AX_MOTOR_C_IN2_GPIO,
            AX_MOTOR_D_IN1_GPIO, AX_MOTOR_D_IN2_GPIO,
        };
        const char *labels[] = {"A_IN1","A_IN2","B_IN1","B_IN2",
                                "C_IN1","C_IN2","D_IN1","D_IN2"};
        printf("[HAL] GPIO selftest START\n");
        int fail_cnt = 0;
        for (int i = 0; i < 8; i++) {
            gpio_num_t p = test_pins[i];
            int r0 = gpio_get_level(p);                       /* 读初始值 */
            gpio_set_level(p, 1);
            int r1 = gpio_get_level(p);                       /* 读 HIGH */
            gpio_set_level(p, 0);
            int r2 = gpio_get_level(p);                       /* 读 LOW (恢复) */
            printf("[HAL] GPIO%-3d (%s): init=%d setH→%d setL→%d  %s\n",
                   p, labels[i], r0, r1, r2,
                   (r1 == 1 && r2 == 0) ? "OK" : "★ FAIL ★");
            if (r1 != 1 || r2 != 0) fail_cnt++;
        }
        printf("[HAL] GPIO selftest END: %d/8 passed, %d FAILED\n", 8 - fail_cnt, fail_cnt);
    }
    ESP_LOGI(TAG, "Step 1/5: GPIO done");

    /* ── 电机 PWM (使用 BSP ledc.h 定义的 channel 宏) ── */
    ESP_LOGI(TAG, "Step 2/5: Motor PWM init...");
    ax_motor_a_cfg = motor_pwm_init(LEDC_PWM_TIMER,  LEDC_PWM_CHANNEL,   LEDC_PWM_CHO_GPIO);
    ESP_LOGI(TAG, "  Motor A OK (GPIO%d)", LEDC_PWM_CHO_GPIO);
    ax_motor_b_cfg = motor_pwm_init(LEDC_PWM_TIMER,  LEDC_PWM_CHANNEL_2, LEDC_PWM_CHO_GPIO_2);
    ESP_LOGI(TAG, "  Motor B OK (GPIO%d)", LEDC_PWM_CHO_GPIO_2);
    ax_motor_c_cfg = motor_pwm_init(LEDC_PWM_TIMER_1,LEDC_PWM_CHANNEL_3, LEDC_PWM_CHO_GPIO_3);
    ESP_LOGI(TAG, "  Motor C OK (GPIO%d)", LEDC_PWM_CHO_GPIO_3);
    ax_motor_d_cfg = motor_pwm_init(LEDC_PWM_TIMER_1,LEDC_PWM_CHANNEL_4, LEDC_PWM_CHO_GPIO_4);
    ESP_LOGI(TAG, "  Motor D OK (GPIO%d)", LEDC_PWM_CHO_GPIO_4);
    ESP_LOGI(TAG, "Step 2/5: Motor PWM done");

    /* ── 编码器 (使用 BSP pcnt_encoder.h 定义的引脚宏) ── */
    ESP_LOGI(TAG, "Step 3/5: Encoder init...");
    ax_encoder_a_cfg = enc_init(PCNT_ENCODER_1_A_GPIO, PCNT_ENCODER_1_B_GPIO);
    ESP_LOGI(TAG, "  Enc A OK");
    ax_encoder_b_cfg = enc_init(PCNT_ENCODER_2_A_GPIO, PCNT_ENCODER_2_B_GPIO);
    ESP_LOGI(TAG, "  Enc B OK");
    ax_encoder_c_cfg = enc_init(AX_ENCODER_C_A_GPIO,   AX_ENCODER_C_B_GPIO);
    ESP_LOGI(TAG, "  Enc C OK");
    ax_encoder_d_cfg = enc_init(AX_ENCODER_D_A_GPIO,   AX_ENCODER_D_B_GPIO);
    ESP_LOGI(TAG, "  Enc D OK");
    ESP_LOGI(TAG, "Step 3/5: Encoder done");

    /* ── 舵机 (使用 BSP ledc_Init) ── */
    ESP_LOGI(TAG, "Step 4/5: Servo init...");
    servo_init();
    ESP_LOGI(TAG, "Step 4/5: Servo done");
    AX_SERVO_S1_SetAngle(900);
    AX_SERVO_S2_SetAngle(900);
    ESP_LOGI(TAG, "Step 5/5: HAL Init complete!");
}

/* ═══════════════════ 编码器接口 ═══════════════════ */

int32_t AX_ENCODER_A_GetCounter(void) { return (int32_t)pcnt_encoder_get_pulse_count(ax_encoder_a_cfg); }
int32_t AX_ENCODER_B_GetCounter(void) { return (int32_t)pcnt_encoder_get_pulse_count(ax_encoder_b_cfg); }
int32_t AX_ENCODER_C_GetCounter(void) { return (int32_t)pcnt_encoder_get_pulse_count(ax_encoder_c_cfg); }
int32_t AX_ENCODER_D_GetCounter(void) { return (int32_t)pcnt_encoder_get_pulse_count(ax_encoder_d_cfg); }

void AX_ENCODER_A_Reset(void) { pcnt_encoder_reset(ax_encoder_a_cfg); }
void AX_ENCODER_B_Reset(void) { pcnt_encoder_reset(ax_encoder_b_cfg); }
void AX_ENCODER_C_Reset(void) { pcnt_encoder_reset(ax_encoder_c_cfg); }
void AX_ENCODER_D_Reset(void) { pcnt_encoder_reset(ax_encoder_d_cfg); }

/** 周期性更新四路编码器的转速(RPM)和角度(°)，建议在主循环 50Hz 中调用 */
void AX_ENCODER_UpdateAll(void)
{
    pcnt_encoder_update(ax_encoder_a_cfg);
    pcnt_encoder_update(ax_encoder_b_cfg);
    pcnt_encoder_update(ax_encoder_c_cfg);
    pcnt_encoder_update(ax_encoder_d_cfg);
}

float AX_ENCODER_A_GetSpeedRPM(void)  { return pcnt_encoder_get_speed_rpm(ax_encoder_a_cfg); }
float AX_ENCODER_B_GetSpeedRPM(void)  { return pcnt_encoder_get_speed_rpm(ax_encoder_b_cfg); }
float AX_ENCODER_C_GetSpeedRPM(void)  { return pcnt_encoder_get_speed_rpm(ax_encoder_c_cfg); }
float AX_ENCODER_D_GetSpeedRPM(void)  { return pcnt_encoder_get_speed_rpm(ax_encoder_d_cfg); }

float AX_ENCODER_A_GetAngleDeg(void)  { return pcnt_encoder_get_angle_deg(ax_encoder_a_cfg); }
float AX_ENCODER_B_GetAngleDeg(void)  { return pcnt_encoder_get_angle_deg(ax_encoder_b_cfg); }
float AX_ENCODER_C_GetAngleDeg(void)  { return pcnt_encoder_get_angle_deg(ax_encoder_c_cfg); }
float AX_ENCODER_D_GetAngleDeg(void)  { return pcnt_encoder_get_angle_deg(ax_encoder_d_cfg); }

/* ═══════════════════ 电机接口 ═══════════════════ */

void AX_MOTOR_A_SetSpeed(int16_t p) { motor_set_speed(ax_motor_a_cfg, AX_MOTOR_A_IN1_GPIO, AX_MOTOR_A_IN2_GPIO, p); }
void AX_MOTOR_B_SetSpeed(int16_t p) { motor_set_speed(ax_motor_b_cfg, AX_MOTOR_B_IN1_GPIO, AX_MOTOR_B_IN2_GPIO, p); }
void AX_MOTOR_C_SetSpeed(int16_t p) { motor_set_speed(ax_motor_c_cfg, AX_MOTOR_C_IN1_GPIO, AX_MOTOR_C_IN2_GPIO, p); }
void AX_MOTOR_D_SetSpeed(int16_t p) { motor_set_speed(ax_motor_d_cfg, AX_MOTOR_D_IN1_GPIO, AX_MOTOR_D_IN2_GPIO, p); }

/* ═══════════════════ 舵机接口 ═══════════════════ */

void AX_SERVO_S1_SetAngle(int16_t a) { servo_set_angle(servo_s1_cfg, a); }
void AX_SERVO_S2_SetAngle(int16_t a) { servo_set_angle(servo_s2_cfg, a); }
