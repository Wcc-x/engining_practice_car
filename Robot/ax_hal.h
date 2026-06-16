/**
 * @file    ax_hal.h
 * @brief   硬件抽象层 — ESP32 电机驱动 (IN1/IN2 H桥模式)
 *
 * 引脚映射:
 *   Motor A: PWM=GPIO1,  IN1=GPIO0,  IN2=GPIO18
 *   Motor B: PWM=GPIO2,  IN1=GPIO3,  IN2=GPIO19
 *   Motor C: PWM=GPIO10, IN1=GPIO12, IN2=GPIO21
 *   Motor D: PWM=GPIO11, IN1=GPIO13, IN2=GPIO22
 *   Encoder A: A=4  B=5
 *   Encoder B: A=6  B=7
 *   Encoder C: A=8  B=9
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

/*   
 *  电机 IN1/IN2 引脚 (H桥方向控制)
 *    */
#define AX_MOTOR_A_IN1_GPIO     GPIO_NUM_0
#define AX_MOTOR_A_IN2_GPIO     GPIO_NUM_18
#define AX_MOTOR_B_IN1_GPIO     GPIO_NUM_3
#define AX_MOTOR_B_IN2_GPIO     GPIO_NUM_19
#define AX_MOTOR_C_IN1_GPIO     GPIO_NUM_12
#define AX_MOTOR_C_IN2_GPIO     GPIO_NUM_21
#define AX_MOTOR_D_IN1_GPIO     GPIO_NUM_13
#define AX_MOTOR_D_IN2_GPIO     GPIO_NUM_22

/*   
 *  编码器引脚
 *    */
#define AX_ENCODER_C_A_GPIO     GPIO_NUM_8
#define AX_ENCODER_C_B_GPIO     GPIO_NUM_9
#define AX_ENCODER_D_A_GPIO     GPIO_NUM_14
#define AX_ENCODER_D_B_GPIO     GPIO_NUM_15

/*   
 *  舵机引脚
 *    */
#define AX_SERVO_S1_GPIO        GPIO_NUM_16
#define AX_SERVO_S2_GPIO        GPIO_NUM_17
#define AX_SERVO_FREQ_HZ        50
#define AX_SERVO_MIN_US         500
#define AX_SERVO_MAX_US         2500

/*   
 *  电机 PWM 参数
 *    */
#define AX_MOTOR_PWM_MAX        4200

/*   
 *  HAL API
 *    */
void AX_HAL_Init(void);

/* 编码器 */
int16_t AX_ENCODER_A_GetCounter(void);
int16_t AX_ENCODER_B_GetCounter(void);
int16_t AX_ENCODER_C_GetCounter(void);
int16_t AX_ENCODER_D_GetCounter(void);
void    AX_ENCODER_A_SetCounter(int16_t val);
void    AX_ENCODER_B_SetCounter(int16_t val);
void    AX_ENCODER_C_SetCounter(int16_t val);
void    AX_ENCODER_D_SetCounter(int16_t val);

/* 电机 (IN1/IN2 H桥模式) */
void AX_MOTOR_A_SetSpeed(int16_t pwm);
void AX_MOTOR_B_SetSpeed(int16_t pwm);
void AX_MOTOR_C_SetSpeed(int16_t pwm);
void AX_MOTOR_D_SetSpeed(int16_t pwm);

/* 舵机 */
void AX_SERVO_S1_SetAngle(int16_t angle);
void AX_SERVO_S2_SetAngle(int16_t angle);

#ifdef __cplusplus
}
#endif

#endif
