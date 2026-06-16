/*
*--------------------------------------------------------------------------
此部分主要是四个电机的pid调用环节，写的很一般，但确实方便，只要给定两个参数就可以了6.15
现在不一样了，我重构了6.16
*--------------------------------------------------------------------------

每个huanmotor_pwm_out
*/

/* Includes ------------------------------------------------------------------*/
#include "ax_speed.h"
#include "freertos/FreeRTOS.h"
/*
创建一些PID实例，用于计算
*/
extern PID_Typedef A_PID={
	800,
	1000,
	1,
	4200,
};
extern PID_Typedef B_PID={
	800,
	1000,
	1,
	4200,
};
extern PID_Typedef C_PID={
	800,
	1000,
	1,
	4200,
};
extern PID_Typedef D_PID={
	800,
	1000,
	1,
	4200,
};
//外部声明变量，可以在Kinematics解算里面正常调用
/*
pid_handle的计算,设计最简单的pid单元
/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 ,范围（±250）
  *          spd_current: 编码器速度当前值
  * @返回值  电机PWM速度
  */
uint16_t PID_Handle(PID_Typedef *pid,float spd_target,float std_current){
	uint16_t pwm_out;
	/*计算各个PID输出*/
	pid->bias=spd_target-std_current;
	pid->Kp_out=pid->Kp*pid->bias;
	pid->Ki_out+=pid->bias;
	if(pid->Ki_out>=pid->I_OutMax){
		pid->Ki_out=pid->I_OutMax;
	}
	pid->Kd_out=pid->Kd*(pid->bias-pid->bias_last);
	//输出PWM_out
	pwm_out=pid->Kp_out+pid->Ki_out+pid->Kd_out;
	pid->bias_last=pid->bias;
	//总输出限幅
	if(pwm_out>=pid->pwm_out_max){
		pwm_out=pid->pwm_out_max;
	}
	if(pwm_out<=-pid->pwm_out_max){
		pwm_out=-pid->pwm_out_max;
	}
	return pwm_out;
}

ROBOT_Velocity  R_Vel;
ROBOT_Wheel     R_Wheel_A, R_Wheel_B, R_Wheel_C, R_Wheel_D;

uint8_t ax_robot_move_enable = 1;

/**
  * @简  述 设置机器人目标速度 — 唯一的对外速度接口
  * @参  数(这一部分写的有歧义) vx: X方向线速度 (×1000, 即千分之一 m/s)
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
 