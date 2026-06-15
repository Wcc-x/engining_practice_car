/**
 * @file    ax_hal.c
 * @brief   硬件抽象层实现 — 将原 STM32 接口映射到 ESP32 BSP 驱动
 * @author  塔克创新团队 (XTARK)
 */

#include "ax_hal.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <math.h>

/* ============================================================
 *  BSP 实例指针 (全局可访问)
 * ============================================================ */
ledc_config_t *ax_motor_a_cfg = NULL;
ledc_config_t *ax_motor_b_cfg = NULL;
ledc_config_t *ax_motor_c_cfg = NULL;
ledc_config_t *ax_motor_d_cfg = NULL;

pcnt_encoder_config_t *ax_encoder_a_cfg = NULL;
pcnt_encoder_config_t *ax_encoder_b_cfg = NULL;
pcnt_encoder_config_t *ax_encoder_c_cfg = NULL;
pcnt_encoder_config_t *ax_encoder_d_cfg = NULL;

/* ============================================================
 *  舵机 LEDC 句柄 (内部使用)
 * ============================================================ */
static ledc_channel_t servo_s1_channel = LEDC_CHANNEL_5;
static ledc_channel_t servo_s2_channel = LEDC_CHANNEL_6;
static ledc_timer_t   servo_timer = LEDC_TIMER_2;

/* ============================================================
 *  内部辅助函数
 * ============================================================ */

/** 创建并初始化一个 LEDC 电机 PWM 通道 */
static ledc_config_t *motor_ledc_init(ledc_timer_t timer, ledc_channel_t channel,
                                       int gpio_num, uint32_t freq_hz)
{
    ledc_config_t *cfg = (ledc_config_t *)malloc(sizeof(ledc_config_t));
    if (!cfg) return NULL;

    cfg->clk_cfg         = LEDC_AUTO_CLK;
    cfg->timer_num       = timer;
    cfg->freq_hz         = freq_hz;
    cfg->duty_resolution = LEDC_TIMER_14_BIT;
    cfg->channel         = channel;
    cfg->duty            = 0;
    cfg->gpio_num        = gpio_num;

    ledc_Init(cfg);
    return cfg;
}

/** 设置电机速度和方向: pwm>0 前进, pwm<0 后退 */
static void motor_set_speed(ledc_config_t *cfg, gpio_num_t dir_gpio, int16_t pwm)
{
    if (!cfg) return;

    if (pwm >= 0) {
        /* 前进方向 */
        gpio_set_level(dir_gpio, 1);
    } else {
        /* 后退方向 */
        gpio_set_level(dir_gpio, 0);
        pwm = -pwm;
    }

    /* 将 PWM 值映射为 0~100 占空比百分比 */
    uint16_t duty_pct = (uint16_t)((uint32_t)pwm * 100 / AX_MOTOR_PWM_MAX);
    if (duty_pct > 100) duty_pct = 100;

    ledc_pwm_set_duty(cfg, duty_pct);
}

/** 初始化单个编码器 */
static pcnt_encoder_config_t *encoder_init(int a_gpio, int b_gpio)
{
    pcnt_encoder_config_t *enc = (pcnt_encoder_config_t *)malloc(sizeof(pcnt_encoder_config_t));
    if (!enc) return NULL;

    enc->a_gpio_num    = a_gpio;
    enc->b_gpio_num    = b_gpio;
    enc->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    enc->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(enc);
    return enc;
}

/** 初始化舵机 LEDC (50Hz, 用于 S1/S2 两个通道) */
static void servo_init(void)
{
    /* 舵机定时器: 50Hz, 14-bit 分辨率 */
    ledc_timer_config_t servo_timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num       = servo_timer,
        .freq_hz         = AX_SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer_cfg);

    /* S1 通道 */
    ledc_channel_config_t s1_chan_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .channel    = servo_s1_channel,
        .duty       = 0,
        .gpio_num   = AX_SERVO_S1_GPIO,
        .hpoint     = 0,
        .timer_sel  = servo_timer,
    };
    ledc_channel_config(&s1_chan_cfg);

    /* S2 通道 */
    ledc_channel_config_t s2_chan_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .channel    = servo_s2_channel,
        .duty       = 0,
        .gpio_num   = AX_SERVO_S2_GPIO,
        .hpoint     = 0,
        .timer_sel  = servo_timer,
    };
    ledc_channel_config(&s2_chan_cfg);
}

static void servo_set_angle(ledc_channel_t channel, int16_t angle_raw)
{
    /*
     * 输入: angle_raw = 角度 × 10 (例如 900 = 90.0°)
     * 舵机脉宽: 500~2500μs 对应 0~180°
     * 50Hz 周期 = 20000μs
     * 14-bit 分辨率 = 16384 级
     */
    float angle_deg = angle_raw / 10.0f;
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    float pulse_us = AX_SERVO_MIN_US +
                     (angle_deg / 180.0f) * (AX_SERVO_MAX_US - AX_SERVO_MIN_US);
    float duty_14bit = pulse_us / 20000.0f * 16384.0f;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, (uint32_t)duty_14bit);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/* ============================================================
 *  HAL 公共接口
 * ============================================================ */

void AX_HAL_Init(void)
{
    /* ---- 初始化方向控制 GPIO ---- */
    gpio_config_t dir_io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << AX_MOTOR_A_DIR_GPIO) |
                        (1ULL << AX_MOTOR_B_DIR_GPIO) |
                        (1ULL << AX_MOTOR_C_DIR_GPIO) |
                        (1ULL << AX_MOTOR_D_DIR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&dir_io_conf);

    /* 初始默认: 正向 */
    gpio_set_level(AX_MOTOR_A_DIR_GPIO, 1);
    gpio_set_level(AX_MOTOR_B_DIR_GPIO, 1);
    gpio_set_level(AX_MOTOR_C_DIR_GPIO, 1);
    gpio_set_level(AX_MOTOR_D_DIR_GPIO, 1);

    /* ---- 初始化电机 PWM (4ch) ---- */
    ax_motor_a_cfg = motor_ledc_init(LEDC_TIMER_0, LEDC_CHANNEL_0,
                                      LEDC_PWM_CHO_GPIO, 1000);
    ax_motor_b_cfg = motor_ledc_init(LEDC_TIMER_0, LEDC_CHANNEL_2,
                                      LEDC_PWM_CHO_GPIO_2, 1000);
    ax_motor_c_cfg = motor_ledc_init(LEDC_TIMER_1, LEDC_CHANNEL_3,
                                      LEDC_PWM_CHO_GPIO_3, 1000);
    ax_motor_d_cfg = motor_ledc_init(LEDC_TIMER_1, LEDC_CHANNEL_4,
                                      LEDC_PWM_CHO_GPIO_4, 1000);

    /* ---- 初始化编码器 (4路) ---- */
    ax_encoder_a_cfg = encoder_init(PCNT_ENCODER_1_A_GPIO, PCNT_ENCODER_1_B_GPIO);
    ax_encoder_b_cfg = encoder_init(PCNT_ENCODER_2_A_GPIO, PCNT_ENCODER_2_B_GPIO);
    ax_encoder_c_cfg = encoder_init(AX_ENCODER_C_A_GPIO, AX_ENCODER_C_B_GPIO);
    ax_encoder_d_cfg = encoder_init(AX_ENCODER_D_A_GPIO, AX_ENCODER_D_B_GPIO);

    /* ---- 初始化舵机 ---- */
    servo_init();

    /* 舵机初始归中 */
    AX_SERVO_S1_SetAngle(900);
    AX_SERVO_S2_SetAngle(900);
}

void AX_HAL_EncoderUpdate(void)
{
    if (ax_encoder_a_cfg) pcnt_encoder_update(ax_encoder_a_cfg);
    if (ax_encoder_b_cfg) pcnt_encoder_update(ax_encoder_b_cfg);
    if (ax_encoder_c_cfg) pcnt_encoder_update(ax_encoder_c_cfg);
    if (ax_encoder_d_cfg) pcnt_encoder_update(ax_encoder_d_cfg);
}

/* ============================================================
 *  编码器接口
 * ============================================================ */

static int16_t encoder_get_counter(pcnt_encoder_config_t *enc)
{
    if (!enc) return 0;
    /* 直接读硬件累积脉冲数，映射为 int16_t */
    int count = 0;
    pcnt_unit_get_count(enc->pcnt_unit, &count);
    return (int16_t)count;
}

static void encoder_clear(pcnt_encoder_config_t *enc)
{
    if (!enc) return;
    pcnt_unit_clear_count(enc->pcnt_unit);
}

int16_t AX_ENCODER_A_GetCounter(void) { return encoder_get_counter(ax_encoder_a_cfg); }
int16_t AX_ENCODER_B_GetCounter(void) { return encoder_get_counter(ax_encoder_b_cfg); }
int16_t AX_ENCODER_C_GetCounter(void) { return encoder_get_counter(ax_encoder_c_cfg); }
int16_t AX_ENCODER_D_GetCounter(void) { return encoder_get_counter(ax_encoder_d_cfg); }

void AX_ENCODER_A_SetCounter(int16_t val) { (void)val; encoder_clear(ax_encoder_a_cfg); }
void AX_ENCODER_B_SetCounter(int16_t val) { (void)val; encoder_clear(ax_encoder_b_cfg); }
void AX_ENCODER_C_SetCounter(int16_t val) { (void)val; encoder_clear(ax_encoder_c_cfg); }
void AX_ENCODER_D_SetCounter(int16_t val) { (void)val; encoder_clear(ax_encoder_d_cfg); }

/* ============================================================
 *  电机接口
 * ============================================================ */

void AX_MOTOR_A_SetSpeed(int16_t pwm) { motor_set_speed(ax_motor_a_cfg, AX_MOTOR_A_DIR_GPIO, pwm); }
void AX_MOTOR_B_SetSpeed(int16_t pwm) { motor_set_speed(ax_motor_b_cfg, AX_MOTOR_B_DIR_GPIO, pwm); }
void AX_MOTOR_C_SetSpeed(int16_t pwm) { motor_set_speed(ax_motor_c_cfg, AX_MOTOR_C_DIR_GPIO, pwm); }
void AX_MOTOR_D_SetSpeed(int16_t pwm) { motor_set_speed(ax_motor_d_cfg, AX_MOTOR_D_DIR_GPIO, pwm); }

/* ============================================================
 *  舵机接口
 * ============================================================ */

void AX_SERVO_S1_SetAngle(int16_t angle) { servo_set_angle(servo_s1_channel, angle); }
void AX_SERVO_S2_SetAngle(int16_t angle) { servo_set_angle(servo_s2_channel, angle); }
