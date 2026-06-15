/**
 * @file    ax_robot.c
 * @brief   机器人全局变量 & 目标速度接口
 */

#include "ax_robot.h"
#include "ax_kinematics.h"

ROBOT_Velocity  R_Vel;
ROBOT_Wheel     R_Wheel_A, R_Wheel_B, R_Wheel_C, R_Wheel_D;

int16_t ax_motor_kp = 800;
int16_t ax_motor_kd = 1000;
uint8_t ax_robot_move_enable = 1;

/**
  * @简  述  设置机器人目标速度 — 唯一的对外速度接口
  * @参  数  vx: X方向线速度 (×1000, 即千分之一 m/s)
  *          vy: Y方向线速度 (×1000)
  *          vw: Z轴角速度  (×1000, 即千分之一 rad/s)
  *          例: AX_ROBOT_SetSpeed(500, 0, 0)  → X方向 0.5m/s 前进
  *              AX_ROBOT_SetSpeed(0, 0, 1000) → 约 1rad/s 旋转
  */
void AX_ROBOT_SetSpeed(int16_t vx, int16_t vy, int16_t vw)
{
    R_Vel.TG_IX = vx;
    R_Vel.TG_IY = vy;
    R_Vel.TG_IW = vw;
}
