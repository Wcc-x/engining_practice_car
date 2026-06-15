/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  

/* Includes ------------------------------------------------------------------*/
#include "ax_robot.h"
#include "ax_kinematics.h"

//机器人速度数据
ROBOT_Velocity  R_Vel;

//机器人轮子数据
ROBOT_Wheel  R_Wheel_A,R_Wheel_B,R_Wheel_C,R_Wheel_D;

//电机PID控制参数
int16_t ax_motor_kp=800;      
int16_t ax_motor_kd=1000; 

//机器人型号
uint8_t ax_robot_type = ROBOT_TYPE;

//机器人运动使能开关,默认打开状态
uint8_t ax_robot_move_enable = 1;

//阿克曼机器人前轮转向角度
int16_t ax_akm_angle = 0;

//阿克曼机器人前轮零偏角度
int16_t ax_akm_offset = 0;

//PS2手柄键值结构体
JOYSTICK_TypeDef my_joystick;  

/******************* (C) 版权 2023 XTARK **************************************/

