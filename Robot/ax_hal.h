/**
 * @file    ax_hal.h
 * @brief   硬件抽象层 — ESP32 电机驱动 (IN1/IN2 H桥模式)
 *
 * 严格遵循 BSP 库:
 *   - BSP\LEDC\ledc.h / ledc.c      → ledc_Init() / ledc_pwm_set_duty()
 *   - BSP\PCNT\pcnt_encoder.h/.c    → pcnt_encoder_init() / get / update / reset / deinit
 *
 * 引脚映射:
 *   Motor A: PWM=GPIO23, IN1=GPIO0,  IN2=GPIO18
 *   Motor B: PWM=GPIO2,  IN1=GPIO25, IN2=GPIO19
 *   Motor C: PWM=GPIO26, IN1=GPIO12, IN2=GPIO21
 *   Motor D: PWM=GPIO27, IN1=GPIO13, IN2=GPIO22
 *   Encoder A: A=4  B=5
 *   Encoder B: A=32 B=33
 *   Encoder C: A=34 B=35
 *   Encoder D: A=14 B=15
 *   Servo S1: GPIO16, Servo S2: GPIO17
 *
 * H桥驱动逻辑:
 *   正转: IN1=1, IN2=0, PWM=占空比
 *   反转: IN1=0, IN2=1, PWM=占空比
 *   停止: IN1=0, IN2=0
 */

#ifndef __AX_HAL_H
#define __AX_HAL_H

#include <stdint.h>
#include "driver/gpio.h"
#include "ledc.h"
#include "pcnt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─────────────── 电机 IN1/IN2 引脚 (H桥方向控制) ─────────────── */

#define AX_MOTOR_A_IN1_GPIO     GPIO_NUM_1
#define AX_MOTOR_A_IN2_GPIO     GPIO_NUM_18
#define AX_MOTOR_B_IN1_GPIO     GPIO_NUM_25   /* 原 GPIO3=UART0_RX 冲突，换至 GPIO25 */
#define AX_MOTOR_B_IN2_GPIO     GPIO_NUM_19
#define AX_MOTOR_C_IN1_GPIO     GPIO_NUM_12
#define AX_MOTOR_C_IN2_GPIO     GPIO_NUM_21
#define AX_MOTOR_D_IN1_GPIO     GPIO_NUM_13
#define AX_MOTOR_D_IN2_GPIO     GPIO_NUM_22

/* ─────────────── 编码器引脚 (C/D 为自定义扩展，BSP 只定义了 1/2 号) ─────────────── */

#define AX_ENCODER_C_A_GPIO     GPIO_NUM_34   /* 原 GPIO8=Flash 冲突 */
#define AX_ENCODER_C_B_GPIO     GPIO_NUM_35   /* 原 GPIO9=Flash 冲突 */
#define AX_ENCODER_D_A_GPIO     GPIO_NUM_14
#define AX_ENCODER_D_B_GPIO     GPIO_NUM_15

/* ─────────────── 舵机引脚 ─────────────── */

#define AX_SERVO_S1_GPIO        GPIO_NUM_16
#define AX_SERVO_S2_GPIO        GPIO_NUM_17
#define AX_SERVO_FREQ_HZ        50
#define AX_SERVO_MIN_US         500
#define AX_SERVO_MAX_US         2500

/* ─────────────── 电机 PWM 参数 ─────────────── */

#define AX_MOTOR_PWM_MAX        4200

/* ═══════════════════════════════════════════════════════════
 *  HAL API
 * ═══════════════════════════════════════════════════════════ */

void AX_HAL_Init(void);

/* ── 编码器 ── */
int32_t AX_ENCODER_A_GetCounter(void);       /* 读累积脉冲 (正转+ / 反转-) */
int32_t AX_ENCODER_B_GetCounter(void);
int32_t AX_ENCODER_C_GetCounter(void);
int32_t AX_ENCODER_D_GetCounter(void);

void    AX_ENCODER_A_Reset(void);            /* 脉冲归零 (硬件 + 软件同步) */
void    AX_ENCODER_B_Reset(void);
void    AX_ENCODER_C_Reset(void);
void    AX_ENCODER_D_Reset(void);

void    AX_ENCODER_UpdateAll(void);          /* 批量更新四路转速/角度 */

float   AX_ENCODER_A_GetSpeedRPM(void);      /* 当前转速 RPM */
float   AX_ENCODER_B_GetSpeedRPM(void);
float   AX_ENCODER_C_GetSpeedRPM(void);
float   AX_ENCODER_D_GetSpeedRPM(void);

float   AX_ENCODER_A_GetAngleDeg(void);      /* 当前角度 0~360° */
float   AX_ENCODER_B_GetAngleDeg(void);
float   AX_ENCODER_C_GetAngleDeg(void);
float   AX_ENCODER_D_GetAngleDeg(void);

/* ── 电机 (IN1/IN2 H桥模式) ── */
void AX_MOTOR_A_SetSpeed(int16_t pwm);
void AX_MOTOR_B_SetSpeed(int16_t pwm);
void AX_MOTOR_C_SetSpeed(int16_t pwm);
void AX_MOTOR_D_SetSpeed(int16_t pwm);

/* ── 舵机 ── */
void AX_SERVO_S1_SetAngle(int16_t angle);
void AX_SERVO_S2_SetAngle(int16_t angle);

#ifdef __cplusplus
}
#endif

#endif
