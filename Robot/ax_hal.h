/**
 * @file    ax_hal.h
 * @brief   硬件抽象层 (Hardware Abstraction Layer)
 *          将原 STM32 平台的电机/编码器/舵机接口映射到 ESP32 BSP 驱动
 *
 * @author  塔克创新团队 (XTARK)
 * @version 适配 ESP32
 *
 * 引脚映射:
 *   Motor A: PWM=GPIO1, DIR=GPIO0
 *   Motor B: PWM=GPIO2, DIR=GPIO3
 *   Motor C: PWM=GPIO10, DIR=GPIO12
 *   Motor D: PWM=GPIO11, DIR=GPIO13
 *   Encoder A: A相=GPIO4, B相=GPIO5
 *   Encoder B: A相=GPIO6, B相=GPIO7
 *   Encoder C: A相=GPIO8, B相=GPIO9
 *   Encoder D: A相=GPIO14, B相=GPIO15
 *   Servo S1: GPIO16 (50Hz PWM)
 *   Servo S2: GPIO17 (50Hz PWM)
 */

#ifndef __AX_HAL_H
#define __AX_HAL_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "ledc.h"
#include "pcnt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  电机方向引脚定义
 * ============================================================ */
#define AX_MOTOR_A_DIR_GPIO     GPIO_NUM_0
#define AX_MOTOR_B_DIR_GPIO     GPIO_NUM_3
#define AX_MOTOR_C_DIR_GPIO     GPIO_NUM_12
#define AX_MOTOR_D_DIR_GPIO     GPIO_NUM_13

/* ============================================================
 *  编码器引脚定义（如果不同于 pcnt_encoder.h 中的默认值）
 *  编码器 C/D 需要额外 GPIO
 * ============================================================ */
#define AX_ENCODER_C_A_GPIO     GPIO_NUM_8
#define AX_ENCODER_C_B_GPIO     GPIO_NUM_9
#define AX_ENCODER_D_A_GPIO     GPIO_NUM_14
#define AX_ENCODER_D_B_GPIO     GPIO_NUM_15

/* ============================================================
 *  舵机引脚及参数
 * ============================================================ */
#define AX_SERVO_S1_GPIO        GPIO_NUM_16
#define AX_SERVO_S2_GPIO        GPIO_NUM_17
#define AX_SERVO_FREQ_HZ        50          /* 舵机标准频率 50Hz (20ms 周期) */
#define AX_SERVO_MIN_US         500         /* 0°对应脉宽 (μs) */
#define AX_SERVO_MAX_US         2500        /* 180°对应脉宽 (μs) */

/* ============================================================
 *  电机 PWM 参数
 * ============================================================ */
#define AX_MOTOR_PWM_MAX        4200        /* PID 输出最大绝对值，对应 100% 占空比 */

/* ============================================================
 *  PS2 手柄键值结构体 (兼容原代码，具体实现由应用层提供)
 * ============================================================ */
typedef struct {
    int16_t lx;     /* 左摇杆 X */
    int16_t ly;     /* 左摇杆 Y */
    int16_t rx;     /* 右摇杆 X */
    int16_t ry;     /* 右摇杆 Y */
    uint8_t btn;    /* 按键状态 */
} JOYSTICK_TypeDef;

/* ============================================================
 *  外部 BSP 实例声明 (由 ax_hal.c 定义)
 * ============================================================ */
extern ledc_config_t *ax_motor_a_cfg;
extern ledc_config_t *ax_motor_b_cfg;
extern ledc_config_t *ax_motor_c_cfg;
extern ledc_config_t *ax_motor_d_cfg;

extern pcnt_encoder_config_t *ax_encoder_a_cfg;
extern pcnt_encoder_config_t *ax_encoder_b_cfg;
extern pcnt_encoder_config_t *ax_encoder_c_cfg;
extern pcnt_encoder_config_t *ax_encoder_d_cfg;

/* ============================================================
 *  HAL 初始化和周期性更新
 * ============================================================ */

/** 初始化所有硬件：电机 PWM、编码器、舵机、方向引脚 */
void AX_HAL_Init(void);

/** 周期性更新编码器读数（在主循环中定时调用） */
void AX_HAL_EncoderUpdate(void);

/* ============================================================
 *  编码器接口 (兼容原 STM32 API)
 * ============================================================ */

/** 读取编码器 A 当前脉冲计数值（带方向） */
int16_t AX_ENCODER_A_GetCounter(void);

/** 读取编码器 B 当前脉冲计数值（带方向） */
int16_t AX_ENCODER_B_GetCounter(void);

/** 读取编码器 C 当前脉冲计数值（带方向） */
int16_t AX_ENCODER_C_GetCounter(void);

/** 读取编码器 D 当前脉冲计数值（带方向） */
int16_t AX_ENCODER_D_GetCounter(void);

/** 清零编码器 A 计数器 */
void AX_ENCODER_A_SetCounter(int16_t val);

/** 清零编码器 B 计数器 */
void AX_ENCODER_B_SetCounter(int16_t val);

/** 清零编码器 C 计数器 */
void AX_ENCODER_C_SetCounter(int16_t val);

/** 清零编码器 D 计数器 */
void AX_ENCODER_D_SetCounter(int16_t val);

/* ============================================================
 *  电机接口 (兼容原 STM32 API)
 * ============================================================ */

/** 设置电机 A 转速, 正=前进, 负=后退, 范围 ±4200 对应 0~100% 占空比 */
void AX_MOTOR_A_SetSpeed(int16_t pwm);

/** 设置电机 B 转速 */
void AX_MOTOR_B_SetSpeed(int16_t pwm);

/** 设置电机 C 转速 */
void AX_MOTOR_C_SetSpeed(int16_t pwm);

/** 设置电机 D 转速 */
void AX_MOTOR_D_SetSpeed(int16_t pwm);

/* ============================================================
 *  舵机接口 (兼容原 STM32 API)
 * ============================================================ */

/** 设置舵机 S1 角度, 角度 = val/10 度 (例如 900=90.0°) */
void AX_SERVO_S1_SetAngle(int16_t angle);

/** 设置舵机 S2 角度 */
void AX_SERVO_S2_SetAngle(int16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __AX_HAL_H */
