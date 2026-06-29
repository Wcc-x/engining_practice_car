
/******************************************************************************
 *
 *    ██╗  ██╗███████╗██╗     ██╗ ██████╗ ███████╗
 *    ██║  ██║██╔════╝██║     ██║██╔═══██╗██╔════╝
 *    ███████║█████╗  ██║     ██║██║   ██║███████╗
 *    ██╔══██║██╔══╝  ██║     ██║██║   ██║╚════██║
 *    ██║  ██║███████╗███████╗██║╚██████╔╝███████║
 *    ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝ ╚═════╝ ╚══════╝
 *
 *                    ☠ ROBOMASTER ☠
 *                     HELIOS WCC
 *
 ******************************************************************************/
#include "ax_kinematics.h"
#include "ax_speed.h"
#include <stdio.h>

/* ── 诊断用: 调用计数 (每 50 次 ≈ 1Hz 打印一次) ── */
static int  kin_dbg_cnt = 0;

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
    R_Wheel_A.RT = (double)AX_ENCODER_A_GetCounter() * MEC_WHEEL_SCALE;
    AX_ENCODER_A_Reset();
    R_Wheel_B.RT = (double)(-AX_ENCODER_B_GetCounter()) * MEC_WHEEL_SCALE;
    AX_ENCODER_B_Reset();
    R_Wheel_C.RT = (double)AX_ENCODER_C_GetCounter() * MEC_WHEEL_SCALE;
    AX_ENCODER_C_Reset();
    R_Wheel_D.RT = (double)(-AX_ENCODER_D_GetCounter()) * MEC_WHEEL_SCALE;
    AX_ENCODER_D_Reset();
  //RT是received target，TG是target goal，PWM是最终输出的PWM值
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


    /*---------------------------------------------------------------------从这*/
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
/*---------------------------------------都是打印出来的垃圾，没必要管*/



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

    /* ★ 目标为零时重置积分 → 彻底消除积分饱和 */
    static short prev_tg_zero = 0;
    short      is_tg_zero = (R_Vel.TG_IX == 0 && R_Vel.TG_IY == 0 && R_Vel.TG_IW == 0);
    if (is_tg_zero) {
        PID_ResetIntegral(&A_PID);
        PID_ResetIntegral(&B_PID);
        PID_ResetIntegral(&C_PID);
        PID_ResetIntegral(&D_PID);
    }
    if (is_tg_zero != prev_tg_zero) {
        printf("[KIN] PID integral %s\n", is_tg_zero ? "RESET (TG=0)" : "ACTIVE (TG≠0)");
        prev_tg_zero = is_tg_zero;
    }

    if (R_Vel.TG_IX >  R_VX_LIMIT) R_Vel.TG_IX =  R_VX_LIMIT;
    if (R_Vel.TG_IX < -R_VX_LIMIT) R_Vel.TG_IX = -R_VX_LIMIT;
    if (R_Vel.TG_IY >  R_VY_LIMIT) R_Vel.TG_IY =  R_VY_LIMIT;
    if (R_Vel.TG_IY < -R_VY_LIMIT) R_Vel.TG_IY = -R_VY_LIMIT;
    if (R_Vel.TG_IW >  R_VW_LIMIT) R_Vel.TG_IW =  R_VW_LIMIT;
    if (R_Vel.TG_IW < -R_VW_LIMIT) R_Vel.TG_IW = -R_VW_LIMIT;

    /* ★ 诊断: Step 3 结果 (约 1Hz) */
    if (kin_dbg_cnt++ % 50 == 0) {
        printf("[KIN] Step3: move_en=%d | TG_IX=%d TG_IY=%d TG_IW=%d | RT_IX=%d RT_IY=%d RT_IW=%d\n",
               ax_robot_move_enable,
               R_Vel.TG_IX, R_Vel.TG_IY, R_Vel.TG_IW,
               R_Vel.RT_IX, R_Vel.RT_IY, R_Vel.RT_IW);
    }

    /* 整型 → 浮点 (m/s, rad/s) 承接tx*/
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
    
    R_Wheel_A.PWM = PID_Handle(&A_PID,R_Wheel_A.TG, (float)R_Wheel_A.RT);
    R_Wheel_B.PWM = PID_Handle(&B_PID,R_Wheel_B.TG, (float)R_Wheel_B.RT);
    R_Wheel_C.PWM = PID_Handle(&C_PID,R_Wheel_C.TG, (float)R_Wheel_C.RT);
    R_Wheel_D.PWM = PID_Handle(&D_PID,R_Wheel_D.TG, (float)R_Wheel_D.RT);

    /* ★ 诊断: Step 5 PID 输出 (约 1Hz, 仅当有目标时打印详情) */
    if (kin_dbg_cnt % 50 == 1) {
        int any_tg = (R_Wheel_A.TG != 0.0f || R_Wheel_B.TG != 0.0f ||
                      R_Wheel_C.TG != 0.0f || R_Wheel_D.TG != 0.0f);
        if (any_tg) {
            printf("[KIN] Step5 PID:  TG A=%.3f B=%.3f C=%.3f D=%.3f (m/s)\n",
                   (double)R_Wheel_A.TG, (double)R_Wheel_B.TG,
                   (double)R_Wheel_C.TG, (double)R_Wheel_D.TG);
            printf("[KIN] Step5 PID:  RT A=%.3f B=%.3f C=%.3f D=%.3f (m/s)\n",
                   R_Wheel_A.RT, R_Wheel_B.RT, R_Wheel_C.RT, R_Wheel_D.RT);
            printf("[KIN] Step5 PID: PWM A=%.1f B=%.1f C=%.1f D=%.1f | bias A=%.3f B=%.3f C=%.3f D=%.3f\n",
                   (double)R_Wheel_A.PWM, (double)R_Wheel_B.PWM,
                   (double)R_Wheel_C.PWM, (double)R_Wheel_D.PWM,
                   (double)A_PID.bias, (double)B_PID.bias,
                   (double)C_PID.bias, (double)D_PID.bias);
        }
    }

    /*
     *  Step 6 — PWM 输出接口: 驱动 ESP32 LEDC 硬件
     *
     *  AX_MOTOR_X_SetSpeed(pwm):
     *    pwm ≥ 0 → GPIO_DIR=HIGH (前进), LEDC占空比=pwm/4200×100%
     *    pwm < 0 → GPIO_DIR=LOW  (后退), LEDC占空比=|pwm|/4200×100%
     *
     *  A/B 反向, C/D 正向 (配合实物电机安装方向)
     *    */
    /* ★ 诊断: Step 6 最终电机指令 */
    if (kin_dbg_cnt % 50 == 1 &&
        (R_Wheel_A.PWM || R_Wheel_B.PWM || R_Wheel_C.PWM || R_Wheel_D.PWM)) {
        printf("[KIN] Step6 MOTOR: A=%.1f B=%.1f C=%.1f D=%.1f (PWM raw)\n",
               (double)R_Wheel_A.PWM, (double)(-R_Wheel_B.PWM),
               (double)R_Wheel_C.PWM, (double)(-R_Wheel_D.PWM));
    }

    AX_MOTOR_A_SetSpeed( -R_Wheel_A.PWM);
    AX_MOTOR_B_SetSpeed( R_Wheel_B.PWM);
    AX_MOTOR_C_SetSpeed( R_Wheel_C.PWM);
    AX_MOTOR_D_SetSpeed(-R_Wheel_D.PWM);
}


