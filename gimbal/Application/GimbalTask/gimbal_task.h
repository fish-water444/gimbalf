#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include "motor.h"
#include "controller.h"
#include "stdbool.h"

#define YAW_MOTOR_ID 0x141
#define YAW_MOTOR_ZERO_OFFSET 10201
#define YAW_MOTOR_MAXOUT 2000 

#define YAW_V_PID_MAXOUT 18000
#define YAW_V_PID_MAXINTEGRAL 10000
#define YAW_V_PID_KP 400  
#define YAW_V_PID_KI 100  
#define YAW_V_PID_KD 0
#define YAW_V_PID_LPF 0.015 

#define YAW_A_PID_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define YAW_A_PID_MAXINTEGRAL 1.0f
#define YAW_A_PID_KP 0.55
#define YAW_A_PID_KI 0
#define YAW_A_PID_KD 0.02  
#define YAW_A_PID_KP_AIMASSIST 0.6
#define YAW_A_PID_KI_AIMASSIST 1.5
#define YAW_A_PID_KD_AIMASSIST 0.004
#define YAW_A_PID_LPF 0.0001
#define YAW_A_PID_D_LPF 0.001

#define YAW_T_PID_MAXOUT 30000
#define YAW_T_PID_MAXINTEGRAL 30000
#define YAW_T_PID_KP 0  
#define YAW_T_PID_KI 300  
#define YAW_T_PID_KD 0
#define YAW_T_PID_LPF 0.0

#define YAW_V_FFC_MAXOUT 0
#define YAW_V_FCC_C0 0.8
#define YAW_V_FCC_C1 0.2
#define YAW_V_FCC_C2 0
#define YAW_V_FCC_LPF 0.005

#define YAW_A_FFC_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define YAW_A_FCC_C0 0
#define YAW_A_FCC_C1 0.0175
#define YAW_A_FCC_C2 0
#define YAW_A_FCC_LPF 0.001

#define RC_YAW_RATIO 1

#define ROLL_THRESHOLD 30
#define PITCH_THRESHOLD 30
#define RATE_THRESHOLD 10

enum GIMBAL
{
    Gimbal_Stop = 0,
    Gimbal_Normal,
    Gimbal_Battle,
};

typedef struct {
    Motor_t YawMotor;                
    float YawAngle;                  // = INS.Yaw (-180°~+180°)
    float PitchAngle;                // = INS.Pitch (只读，用于投影)
    float YawVelocity;               // 投影后的 Yaw 轴角速度
    float YawRefAngle;               // 目标角度
    float YawRefVelocity;            // 遥控器输入的角速度指令
    float EncoderYawAngle;
    uint8_t isRollover;
    float RollTime;
    uint32_t DWT_Count;
    float dt;
    uint8_t Mode;
} Gimbal_t;

extern Gimbal_t Gimbal;

void Gimbal_Init(void);
void Gimbal_Control(void);

#endif
