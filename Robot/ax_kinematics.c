/**
               ____                    _____ _______ _____       XTARK@塔克创新
              / __ \                  / ____|__   __|  __ \
             | |  | |_ __   ___ _ __ | |       | |  | |__) |
             | |  | | '_ \ / _ \ '_ \| |       | |  |  _  /
             | |__| | |_) |  __/ | | | |____   | |  | | \ \
              \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
                    | |
                    |_|                OpenCTR   机器人控制器

  ******************************************************************************
  *
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *
  ******************************************************************************
  * @作  者  塔克创新团队
  * @内  容  麦轮完整解算逻辑
  *          编码器接口(PCNT) → 运动学正解 → 逆解 → PID → PWM输出接口(LEDC)
  *
  ******************************************************************************
  */

#include "ax_kinematics.h"
#include "ax_robot.h"
#include "ax_speed.h"

/*   
 *  麦轮底盘运动模型 (Mecanum Wheel Kinematics)
 *   
 *
 *   轮编号 (俯视图, X 前进, Y 左移):
 *
 *          A(前左)        B(前右)
 *            ↖              ↗
 *           辊子\          /辊子
 *                \        /
 *        ┌───────┐      ┌───────┐
 *        │   ○   │      │   ○   │
 *        └───────┘      └───────┘
 *            |  ← Lw →  |
 *     ─ ─ ─ ┼ ─ ─ ─ ─ ─ ┼ ─ ─ ─   ← 轮距 Lw (MEC_WHEEL_BASE)
 *            |           |
 *        ┌───────┐      ┌───────┐
 *        │   ○   │      │   ○   │
 *        └───────┘      └───────┘
 *            /辊子        \辊子
 *          ↙              ↘
 *          D(后左)        C(后右)
 *
 *          |← 轴距 La →|  (MEC_ACLE_BASE)
 *
 *   辊子安装方向: A/D ↙↖ (左旋), B/C ↗↘ (右旋)
 *   辊子角度: 45° (标准麦轮)
 *
 *   
 *  运动学公式
 *   
 *
 *   正解 (轮速 → 机器人速度):
 *     Vx = ( Va + Vb + Vc + Vd) / 4
 *     Vy = (-Va + Vb + Vc - Vd) / 4
 *     Vω = (-Va + Vb - Vc + Vd) / 4 / R
 *     其中 R = Lw/2 + La/2  (等效旋转半径)
 *
 *   逆解 (机器人速度 → 各轮目标速度):
 *     Va = Vx - Vy - Vω × R
 *     Vb = Vx + Vy + Vω × R
 *     Vc = Vx + Vy - Vω × R
 *     Vd = Vx - Vy + Vω × R
 *
 *   
 *  编码器接口 (ESP32 PCNT 4x 正交解码)
 *   
 *
 *   每转脉冲数: 13线霍尔 × 减速比30 × 4x解码 = 1560 ppr
 *   速度系数:   WHEEL_SCALE = π × D × PID_RATE / ppr
 *              = π × 0.08 × 50 / 1560 ≈ 0.00806 (m/s per count per 20ms)
 *
 *   读取方式:
 *     count = AX_ENCODER_X_GetCounter()  → 读取 PCNT 硬件累加值
 *     AX_ENCODER_X_SetCounter(0)         → 硬件清零
 *     speed = count × WHEEL_SCALE        → 转为 m/s
 *
 *   
 *  PID 控制
 *   
 *
 *   增量式 PD 控制:
 *     bias   = spd_target - spd_current
 *     pwm   += Kp × bias + Kd × (bias - bias_last)
 *     pwm    钳位在 ±4200
 *
 *   
 *  PWM 输出接口 (ESP32 LEDC)
 *   
 *
 *   AX_MOTOR_X_SetSpeed(pwm):
 *     pwm > 0  → GPIO_DIR=HIGH, 占空比 = |pwm|/4200×100%
 *     pwm < 0  → GPIO_DIR=LOW,  占空比 = |pwm|/4200×100%
 *     频率: 1kHz, 精度: 14-bit
 */

/**
  * @简  述  机器人急停 — 四电机 PWM 全部置零
  */
void AX_ROBOT_Stop(void)
{
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    AX_MOTOR_C_SetSpeed(0);
    AX_MOTOR_D_SetSpeed(0);
}

/**
  * @简  述  麦轮完整解算 — 编码器输入→解算→PID→PWM输出
  *
  *          每次调用 (50Hz / 20ms 周期):
  *
  *  ┌────────────────────────────────────────────────────────┐
  *  │                                                        │
  *  │  PCNT硬件寄存器              LEDC硬件输出               │
  *  │      │                           ▲                     │
  *  │      ▼                           │                     │
  *  │  ┌─────────┐  count   ┌──────────┴──────┐              │
  *  │  │ 编码器   │ ───────→ │ AX_MOTOR_SetSpeed │             │
  *  │  │ A B C D  │ 清零     │ 方向+占空比       │             │
  *  │  └─────────┘          └──────────┬──────┘              │
  *  │      │                 pwm(±4200)│                      │
  *  │      ▼ count×SCALE              │                      │
  *  │  ┌─────────┐  m/s   ┌───────────┴──┐                  │
  *  │  │ 轮速RT  │ ──────→ │ PID (PD控制) │                   │
  *  │  │ A B C D │         │ Kp=800 Kd=1000│                  │
  *  │  └────┬────┘         └───────┬──────┘                  │
  *  │       │                      │                          │
  *  │       ▼ m/s                  │ TG m/s                  │
  *  │  ┌──────────┐  整型  ┌───────┴──────┐                  │
  *  │  │ 运动学   │ ─────→ │ 运动学逆解   │                   │
  *  │  │ 正解     │  RT_*  │ 目标→轮速    │                   │
  *  │  │ 轮速→Vxyz│        │ Vxyz→轮速    │                   │
  *  │  └──────────┘        └───────┬──────┘                  │
  *  │                              │                          │
  *  │                      目标速度来源:                       │
  *  │                      R_Vel.TG_IX/IY/IW                 │
  *  │                      (串口/遥控/自主 写入)              │
  *  │                                                        │
  *  └────────────────────────────────────────────────────────┘
  */
void AX_ROBOT_Kinematics(void)
{
    /*   
     *  Step 1 — 编码器接口: 读 PCNT 硬件 → 清零 → 轮速 (m/s)
     *
     *  AX_ENCODER_X_GetCounter()  → 直接读 PCNT 硬件累加器
     *  AX_ENCODER_X_SetCounter(0) → 硬件清零, 下周期重新积累
     *
     *  count × WHEEL_SCALE → 线速度 (m/s)
     *
     *  B/D 轮符号取反: 电机物理安装方向与 A/C 相对
     *    */
    R_Wheel_A.RT = (double)((int16_t)AX_ENCODER_A_GetCounter()) * MEC_WHEEL_SCALE;
    AX_ENCODER_A_SetCounter(0);
    R_Wheel_B.RT = (double)(-(int16_t)AX_ENCODER_B_GetCounter()) * MEC_WHEEL_SCALE;
    AX_ENCODER_B_SetCounter(0);
    R_Wheel_C.RT = (double)( (int16_t)AX_ENCODER_C_GetCounter()) * MEC_WHEEL_SCALE;
    AX_ENCODER_C_SetCounter(0);
    R_Wheel_D.RT = (double)(-(int16_t)AX_ENCODER_D_GetCounter()) * MEC_WHEEL_SCALE;
    AX_ENCODER_D_SetCounter(0);

    /*   
     *  Step 2 — 运动学正解: 四轮线速度 → 机器人速度
     *
     *  Va, Vb, Vc, Vd = 四轮线速度 (m/s)
     *  R = Lw/2 + La/2 = 0.182/2 + 0.124/2 = 0.153 m
     *
     *  Vx = ( Va + Vb + Vc + Vd) / 4          ← 前进速度 (m/s)
     *  Vy = (-Va + Vb + Vc - Vd) / 4          ← 横移速度 (m/s)
     *  Vω = (-Va + Vb - Vc + Vd) / 4 / R      ← 旋转角速度 (rad/s)
     *    */
    double va = R_Wheel_A.RT;
    double vb = R_Wheel_B.RT;
    double vc = R_Wheel_C.RT;
    double vd = R_Wheel_D.RT;
    double R  = MEC_WHEEL_BASE / 2.0 + MEC_ACLE_BASE / 2.0;   /* 0.153 m */

    double r_fx = ( va + vb + vc + vd) / 4.0;      /* 实时 Vx (m/s) */
    double r_fy = (-va + vb + vc - vd) / 4.0;      /* 实时 Vy (m/s) */
    double r_fw = (-va + vb - vc + vd) / 4.0 / R;  /* 实时 Vω (rad/s) */

    /* 存入整型字段 (×1000, 单位: 千分之一 m/s 或 rad/s) */
    R_Vel.RT_IX = (short)(r_fx * 1000.0);
    R_Vel.RT_IY = (short)(r_fy * 1000.0);
    R_Vel.RT_IW = (short)(r_fw * 1000.0);

    /*   
     *  Step 3 — 目标速度限幅 & 运动使能检查
     *
     *  限幅: |Vx| ≤ 1.5 m/s, |Vy| ≤ 1.2 m/s, |Vω| ≤ 2π rad/s
     *  使能关 → 目标清零 → 机器人自然停转 (PID 会将偏差收敛到 0)
     *    */
    if (ax_robot_move_enable == 0) {
        R_Vel.TG_IX = 0;
        R_Vel.TG_IY = 0;
        R_Vel.TG_IW = 0;
    }

    if (R_Vel.TG_IX >  R_VX_LIMIT) R_Vel.TG_IX =  R_VX_LIMIT;
    if (R_Vel.TG_IX < -R_VX_LIMIT) R_Vel.TG_IX = -R_VX_LIMIT;
    if (R_Vel.TG_IY >  R_VY_LIMIT) R_Vel.TG_IY =  R_VY_LIMIT;
    if (R_Vel.TG_IY < -R_VY_LIMIT) R_Vel.TG_IY = -R_VY_LIMIT;
    if (R_Vel.TG_IW >  R_VW_LIMIT) R_Vel.TG_IW =  R_VW_LIMIT;
    if (R_Vel.TG_IW < -R_VW_LIMIT) R_Vel.TG_IW = -R_VW_LIMIT;

    /* 整型 → 浮点 (m/s, rad/s) */
    float tx = R_Vel.TG_IX / 1000.0f;
    float ty = R_Vel.TG_IY / 1000.0f;
    float tw = R_Vel.TG_IW / 1000.0f;

    /*   
     *  Step 4 — 运动学逆解: 机器人目标速度 → 四轮目标线速度 (m/s)
     *
     *  Va = Vx - Vy - Vω × R
     *  Vb = Vx + Vy + Vω × R
     *  Vc = Vx + Vy - Vω × R
     *  Vd = Vx - Vy + Vω × R
     *    */
    float r  = (float)R;  /* R = Lw/2 + La/2 */

    R_Wheel_A.TG = tx - ty - tw * r;
    R_Wheel_B.TG = tx + ty + tw * r;
    R_Wheel_C.TG = tx + ty - tw * r;
    R_Wheel_D.TG = tx - ty + tw * r;

    /*   
     *  Step 5 — PID 控制: 目标速度 vs 实际速度 → PWM (-4200 ~ +4200)
     *
     *  AX_SPEED_PidCtlX(target, current):
     *    bias = target - current
     *    pwm += Kp × bias + Kd × (bias - bias_last)
     *    pwm 钳位 ±4200
     *    */
    R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, (float)R_Wheel_A.RT);
    R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, (float)R_Wheel_B.RT);
    R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, (float)R_Wheel_C.RT);
    R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, (float)R_Wheel_D.RT);

    /*   
     *  Step 6 — PWM 输出接口: 驱动 ESP32 LEDC 硬件
     *
     *  AX_MOTOR_X_SetSpeed(pwm):
     *    pwm ≥ 0 → GPIO_DIR=HIGH (前进), LEDC占空比=pwm/4200×100%
     *    pwm < 0 → GPIO_DIR=LOW  (后退), LEDC占空比=|pwm|/4200×100%
     *
     *  A/B 反向, C/D 正向 (配合实物电机安装方向)
     *    */
    AX_MOTOR_A_SetSpeed(-R_Wheel_A.PWM);
    AX_MOTOR_B_SetSpeed(-R_Wheel_B.PWM);
    AX_MOTOR_C_SetSpeed( R_Wheel_C.PWM);
    AX_MOTOR_D_SetSpeed( R_Wheel_D.PWM);
}

/******************* (C) 版权 2023 XTARK **************************************/
