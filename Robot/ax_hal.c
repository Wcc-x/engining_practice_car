/**
 * @file    ax_hal.c
 * @brief   硬件抽象层实现 — IN1/IN2 H桥电机驱动 + PCNT编码器 + 舵机
 */

#include "ax_hal.h"
#include "esp_timer.h"
#include <stdlib.h>

/* ============================================================
 *  BSP 实例指针
 * ============================================================ */
ledc_config_t *ax_motor_a_cfg;
ledc_config_t *ax_motor_b_cfg;
ledc_config_t *ax_motor_c_cfg;
ledc_config_t *ax_motor_d_cfg;

pcnt_encoder_config_t *ax_encoder_a_cfg;
pcnt_encoder_config_t *ax_encoder_b_cfg;
pcnt_encoder_config_t *ax_encoder_c_cfg;
pcnt_encoder_config_t *ax_encoder_d_cfg;

/* ============================================================
 *  舵机内部句柄
 * ============================================================ */
static ledc_channel_t servo_s1_ch = LEDC_CHANNEL_5;
static ledc_channel_t servo_s2_ch = LEDC_CHANNEL_6;
static ledc_timer_t   servo_tmr  = LEDC_TIMER_2;

/* ============================================================
 *  内部: 电机 LEDC PWM 初始化
 * ============================================================ */
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
    ledc_Init(c);
    return c;
}

/* ============================================================
 *  内部: 单路电机驱动 — IN1/IN2 H桥模式
 *
 *  pwm > 0  正转: IN1=1 IN2=0
 *  pwm < 0  反转: IN1=0 IN2=1
 *  pwm = 0  停止: IN1=0 IN2=0
 * ============================================================ */
static void motor_set_speed(ledc_config_t *cfg,
                            gpio_num_t in1, gpio_num_t in2, int16_t pwm)
{
    if (pwm > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (pwm < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        pwm = -pwm;
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    }

    uint16_t duty = (uint16_t)((uint32_t)pwm * 100 / AX_MOTOR_PWM_MAX);
    if (duty > 100) duty = 100;
    ledc_pwm_set_duty(cfg, duty);
}

/* ============================================================
 *  内部: 编码器初始化
 * ============================================================ */
static pcnt_encoder_config_t *enc_init(int a, int b)
{
    pcnt_encoder_config_t *e = malloc(sizeof(pcnt_encoder_config_t));
    e->a_gpio_num     = a;
    e->b_gpio_num     = b;
    e->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    e->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(e);
    return e;
}

/* ============================================================
 *  内部: 舵机初始化 (50Hz)
 * ============================================================ */
static void servo_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = servo_tmr,
        .freq_hz   = AX_SERVO_FREQ_HZ,
        .clk_cfg   = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t s1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .channel    = servo_s1_ch, .duty = 0,
        .gpio_num   = AX_SERVO_S1_GPIO, .hpoint = 0, .timer_sel = servo_tmr,
    };
    ledc_channel_config(&s1);

    ledc_channel_config_t s2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .channel    = servo_s2_ch, .duty = 0,
        .gpio_num   = AX_SERVO_S2_GPIO, .hpoint = 0, .timer_sel = servo_tmr,
    };
    ledc_channel_config(&s2);
}

static void servo_set_angle(ledc_channel_t ch, int16_t raw)
{
    float deg = raw / 10.0f;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    float us = AX_SERVO_MIN_US + (deg / 180.0f) * (AX_SERVO_MAX_US - AX_SERVO_MIN_US);
    uint32_t d = (uint32_t)(us / 20000.0f * 16384.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* ============================================================
 *  HAL 公共接口
 * ============================================================ */

void AX_HAL_Init(void)
{
    /* IN1/IN2 GPIO 初始化 */
    uint64_t mask = (1ULL << AX_MOTOR_A_IN1_GPIO) | (1ULL << AX_MOTOR_A_IN2_GPIO) |
                    (1ULL << AX_MOTOR_B_IN1_GPIO) | (1ULL << AX_MOTOR_B_IN2_GPIO) |
                    (1ULL << AX_MOTOR_C_IN1_GPIO) | (1ULL << AX_MOTOR_C_IN2_GPIO) |
                    (1ULL << AX_MOTOR_D_IN1_GPIO) | (1ULL << AX_MOTOR_D_IN2_GPIO);
    gpio_config_t c = { .intr_type = GPIO_INTR_DISABLE, .mode = GPIO_MODE_OUTPUT,
                        .pin_bit_mask = mask,
                        .pull_down_en = 0, .pull_up_en = 0 };
    gpio_config(&c);

    /* 初始全低 */
    gpio_set_level(AX_MOTOR_A_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(AX_MOTOR_B_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_B_IN2_GPIO, 0);
    gpio_set_level(AX_MOTOR_C_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_C_IN2_GPIO, 0);
    gpio_set_level(AX_MOTOR_D_IN1_GPIO, 0); gpio_set_level(AX_MOTOR_D_IN2_GPIO, 0);

    /* 电机 PWM */
    ax_motor_a_cfg = motor_pwm_init(LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_PWM_CHO_GPIO);
    ax_motor_b_cfg = motor_pwm_init(LEDC_TIMER_0, LEDC_CHANNEL_2, LEDC_PWM_CHO_GPIO_2);
    ax_motor_c_cfg = motor_pwm_init(LEDC_TIMER_1, LEDC_CHANNEL_3, LEDC_PWM_CHO_GPIO_3);
    ax_motor_d_cfg = motor_pwm_init(LEDC_TIMER_1, LEDC_CHANNEL_4, LEDC_PWM_CHO_GPIO_4);

    /* 编码器 */
    ax_encoder_a_cfg = enc_init(PCNT_ENCODER_1_A_GPIO, PCNT_ENCODER_1_B_GPIO);
    ax_encoder_b_cfg = enc_init(PCNT_ENCODER_2_A_GPIO, PCNT_ENCODER_2_B_GPIO);
    ax_encoder_c_cfg = enc_init(AX_ENCODER_C_A_GPIO, AX_ENCODER_C_B_GPIO);
    ax_encoder_d_cfg = enc_init(AX_ENCODER_D_A_GPIO, AX_ENCODER_D_B_GPIO);

    /* 舵机 */
    servo_init();
    AX_SERVO_S1_SetAngle(900);
    AX_SERVO_S2_SetAngle(900);
}

/* 编码器 */
static int16_t enc_get(pcnt_encoder_config_t *e) {
    int c = 0; pcnt_unit_get_count(e->pcnt_unit, &c); return (int16_t)c;
}
static void enc_clr(pcnt_encoder_config_t *e) { pcnt_unit_clear_count(e->pcnt_unit); }

int16_t AX_ENCODER_A_GetCounter(void)        { return enc_get(ax_encoder_a_cfg); }
int16_t AX_ENCODER_B_GetCounter(void)        { return enc_get(ax_encoder_b_cfg); }
int16_t AX_ENCODER_C_GetCounter(void)        { return enc_get(ax_encoder_c_cfg); }
int16_t AX_ENCODER_D_GetCounter(void)        { return enc_get(ax_encoder_d_cfg); }
void AX_ENCODER_A_SetCounter(int16_t v)      { (void)v; enc_clr(ax_encoder_a_cfg); }
void AX_ENCODER_B_SetCounter(int16_t v)      { (void)v; enc_clr(ax_encoder_b_cfg); }
void AX_ENCODER_C_SetCounter(int16_t v)      { (void)v; enc_clr(ax_encoder_c_cfg); }
void AX_ENCODER_D_SetCounter(int16_t v)      { (void)v; enc_clr(ax_encoder_d_cfg); }

/* 电机 */
void AX_MOTOR_A_SetSpeed(int16_t p) { motor_set_speed(ax_motor_a_cfg, AX_MOTOR_A_IN1_GPIO, AX_MOTOR_A_IN2_GPIO, p); }
void AX_MOTOR_B_SetSpeed(int16_t p) { motor_set_speed(ax_motor_b_cfg, AX_MOTOR_B_IN1_GPIO, AX_MOTOR_B_IN2_GPIO, p); }
void AX_MOTOR_C_SetSpeed(int16_t p) { motor_set_speed(ax_motor_c_cfg, AX_MOTOR_C_IN1_GPIO, AX_MOTOR_C_IN2_GPIO, p); }
void AX_MOTOR_D_SetSpeed(int16_t p) { motor_set_speed(ax_motor_d_cfg, AX_MOTOR_D_IN1_GPIO, AX_MOTOR_D_IN2_GPIO, p); }

/* 舵机 */
void AX_SERVO_S1_SetAngle(int16_t a) { servo_set_angle(servo_s1_ch, a); }
void AX_SERVO_S2_SetAngle(int16_t a) { servo_set_angle(servo_s2_ch, a); }
