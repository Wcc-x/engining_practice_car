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
#ifndef __AX_SPEED_H
#define __AX_SPEED_H

/* Includes ------------------------------------------------------------------*/
#include "ax_hal.h"
//电机PID闭环速度控制函数

typedef struct{
float Kp;
float Ki;
float Kd;
float pwm_out_max;
float I_OutMax;
float Kp_out;
float Ki_out;
float Kd_out;
 float bias;
float bias_last;
} PID_Typedef;

extern PID_Typedef A_PID;
extern PID_Typedef B_PID;
extern PID_Typedef C_PID;
extern PID_Typedef D_PID;

float PID_Handle(PID_Typedef *pid,float spd_target,float std_current);
void  PID_ResetIntegral(PID_Typedef *pid);


/* 结构体 */
typedef struct {
    double  RT;           /* 实时轮速 (m/s) */
    float   TG;           /* 目标轮速 (m/s) */
    short   PWM;          /* PWM 控制量 (±4200) */
} ROBOT_Wheel;

typedef struct {
    short   RT_IX, RT_IY, RT_IW;    /* 实时速度 (×1000) */
    short   TG_IX, TG_IY, TG_IW;    /* 目标速度 (×1000) */
} ROBOT_Velocity;

/* 麦轮底盘参数 */
#define  PI                 3.1416f
#define  PID_RATE           50
#define  MEC_WHEEL_BASE     0.182       /* 轮距 (m) */
#define  MEC_ACLE_BASE      0.124       /* 轴距 (m) */
#define  MEC_WHEEL_DIAMETER  0.080      /* 轮径 (m) */
#define  MEC_WHEEL_RESOLUTION  1560.0   /* 编码器 ppr */
#define  MEC_WHEEL_SCALE     (PI * MEC_WHEEL_DIAMETER * PID_RATE / MEC_WHEEL_RESOLUTION)
//PID_RATE表示一次pid的输出时间，从而得到周期计算数据

/* 速度限制 (×1000) */
#define R_VX_LIMIT  1500
#define R_VY_LIMIT  1200
#define R_VW_LIMIT  6280

/* 全局变量 */
extern ROBOT_Velocity  R_Vel;
extern ROBOT_Wheel     R_Wheel_A, R_Wheel_B, R_Wheel_C, R_Wheel_D;
extern uint8_t         ax_robot_move_enable;

/* 目标速度接口 */
void AX_ROBOT_SetSpeed(int16_t vx, int16_t vy, int16_t vw);
void AX_ROBOT_Stop(void);

#endif

/******************* (C) 版权 2023 XTARK **************************************/
