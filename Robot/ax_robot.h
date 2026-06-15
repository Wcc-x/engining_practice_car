/**
               ____                    _____ _______ _____       XTARK@塔克创新
              / __ \                  / ____|__   __|  __ \
             | |  | |_ __   ___ _ __ | |       | |  | |__) |
             | |  | | '_ \ / _ \ '_ \| |       | |  |  _  /
             | |__| | |_) |  __/ | | | |____   | |  | | \ \
              \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
                    | |
                    |_|                OpenCTR   机器人控制器

  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ROBOT_H
#define __AX_ROBOT_H

/* Includes ------------------------------------------------------------------*/
/* ESP32 平台头文件 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* C库函数声明头文件 --------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 硬件抽象层头文件 ----------------------------------------------------------*/
#include "ax_hal.h"       /* 硬件抽象层: 电机/编码器/舵机接口 */

/* 机器人结构体 --------------------------------------------------------------*/

/* 电机轮子速度结构体 */
typedef struct
{
    double  RT;           /* 实时速度，单位 m/s */
    float   TG;           /* 电机目标速度，单位 m/s */
    short   PWM;          /* 电机PWM控制速度 */

} ROBOT_Wheel;

/* 机器人速度结构体 */
typedef struct
{
    short   RT_IX;        /* 实时X轴速度，16位整型 */
    short   RT_IY;        /* 实时Y轴速度，16位整型 */
    short   RT_IW;        /* 实时Yaw旋转速度，16位整型 */

    short   TG_IX;        /* 目标X轴速度，16位整型 */
    short   TG_IY;        /* 目标Y轴速度，16位整型 */
    short   TG_IW;        /* 目标Yaw旋转速度，16位整型 */

    float   RT_FX;        /* 实时X轴速度（浮点） */
    float   RT_FY;        /* 实时Y轴速度（浮点） */
    float   RT_FW;        /* 实时Yaw旋转速度（浮点） */

    float   TG_FX;        /* 目标X轴速度（浮点） */
    float   TG_FY;        /* 目标Y轴速度（浮点） */
    float   TG_FW;        /* 目标Yaw旋转速度（浮点） */

} ROBOT_Velocity;

/* 数学常量 -----------------------------------------------------------------*/
#define  PI            3.1416f     /* 圆周率 PI */
#define  SQRT3         1.732f      /* 3的平方根 */
#define  PID_RATE      50          /* PID 控制频率 (Hz) */

/* 机器人底盘类型定义 -------------------------------------------------------*/

#define ROBOT_MEC   0x01           /* 麦克纳姆轮底盘 */
#define ROBOT_FWD   0x02           /* 四轮差速底盘 */
#define ROBOT_TWD   0x03           /* 两轮差速转弯底盘 */
#define ROBOT_AKM   0x04           /* 阿克曼转弯底盘 */
#define ROBOT_OMT   0x05           /* 三轮全向底盘 */

/* 当前机器人底盘类型选择 */
#define ROBOT_TYPE   ROBOT_FWD

/* 机器人参数 ----------------------------------------------------------------*/

/* 麦克纳姆轮机器人参数 */
#define  MEC_WHEEL_BASE            0.182       /* 轮距，左右轮的距离 */
#define  MEC_ACLE_BASE             0.124       /* 轴距，前后轮的距离 */
#define  MEC_WHEEL_DIAMETER        0.080       /* 轮子直径 (m) */
#define  MEC_WHEEL_RESOLUTION      1560.0      /* 编码器分辨率 (13线), 减速比30, 13×30×4=1560 */
#define  MEC_WHEEL_SCALE           (PI*MEC_WHEEL_DIAMETER*PID_RATE/MEC_WHEEL_RESOLUTION) /* 轮子速度 m/s 与编码器转换系数 */

/* 四轮差速机器人参数 */
#define  FWD_WHEEL_BASE            0.162       /* 轮距，左右轮的距离 */
#define  FWD_WB_SCALE              1.75        /* 轮距系数（与车体结构/轮胎摩擦/转向半径等有关，常用实验法测定） */
#define  FWD_WHEEL_DIAMETER        0.065       /* 轮子直径 (m) */
#define  FWD_WHEEL_RESOLUTION      1560.0      /* 编码器分辨率 (13线), 减速比30, 13×30×4=1560 */
#define  FWD_WHEEL_SCALE           (PI*FWD_WHEEL_DIAMETER*PID_RATE/FWD_WHEEL_RESOLUTION) /* 轮子速度 m/s 与编码器转换系数 */

/* 两轮差速机器人参数 */
#define  TWD_WHEEL_DIAMETER        0.0724      /* 轮子直径 (m) */
#define  TWD_WHEEL_BASE            0.206       /* 轮距，左右轮的距离 */
#define  TWD_WHEEL_RESOLUTION      1560.0      /* 编码器分辨率 (13线), 减速比30, 13×30×4=1560 */
#define  TWD_WHEEL_SCALE           (PI*TWD_WHEEL_DIAMETER*PID_RATE/TWD_WHEEL_RESOLUTION) /* 轮子速度 m/s 与编码器转换系数 */

/* 阿克曼机器人参数 */
#define  AKM_WHEEL_BASE            0.165       /* 轮距，左右轮的距离 */
#define  AKM_ACLE_BASE             0.175f      /* 轴距，前后轮的距离 */
#define  AKM_WHEEL_DIAMETER        0.075       /* 轮子直径 (m) */
#define  AKM_WHEEL_RESOLUTION      1560.0      /* 编码器分辨率 (13线), 减速比30, 13×30×4=1560 */
#define  AKM_TURN_R_MINI           0.35f       /* 最小转弯半径 ( L×cot30°−W/2 ) */
#define  AKM_WHEEL_SCALE           (PI*AKM_WHEEL_DIAMETER*PID_RATE/AKM_WHEEL_RESOLUTION) /* 轮子速度 m/s 与编码器转换系数 */

/* 三轮全向机器人参数 */
#define  OMT_WHEEL_DIAMETER        0.058       /* 轮子直径 (m) */
#define  OMT_WHEEL_RADIUS          0.206       /* 机器人半径（轮子到机器人中心的距离） */
#define  OMT_WHEEL_RESOLUTION      1560.0      /* 编码器分辨率 (13线), 减速比30, 13×30×4=1560 */
#define  OMT_WHEEL_SCALE           (PI*TWD_WHEEL_DIAMETER*PID_RATE/OMT_WHEEL_RESOLUTION) /* 轮子速度 m/s 与编码器转换系数 */

/* 通信协议ID ----------------------------------------------------------------*/
#define  ID_UTX_DATA     0x10                  /* 上行数据帧ID */
#define  ID_URX_VEL      0x50                  /* 下行速度帧ID */

/* 速度限制 (16位整型, 单位: 千分之一 m/s 或 千分之一 rad/s) -----------------*/
#define R_VX_LIMIT  1500                       /* X轴线速度限制 */
#define R_VY_LIMIT  1200                       /* Y轴线速度限制 */
#define R_VW_LIMIT  6280                       /* Yaw角速度限制 */

/* 全局变量声明 --------------------------------------------------------------*/

extern  ROBOT_Velocity  R_Vel;                 /* 机器人速度数据 */
extern  ROBOT_Wheel  R_Wheel_A, R_Wheel_B, R_Wheel_C, R_Wheel_D;  /* 四个轮子数据 */

extern uint8_t ax_robot_type;                  /* 机器人底盘类型 */
extern uint8_t ax_robot_move_enable;           /* 机器人运动使能开关 */

extern int16_t ax_motor_kp;                    /* 电机 PID 比例系数 */
extern int16_t ax_motor_kd;                    /* 电机 PID 微分系数 */

extern int16_t ax_akm_offset;                  /* 阿克曼前轮零偏角度 */
extern int16_t ax_akm_angle;                   /* 阿克曼前轮转向角度 */

extern JOYSTICK_TypeDef my_joystick;           /* PS2 手柄键值结构体 */

#endif /* __AX_ROBOT_H */
