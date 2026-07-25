/**
 ******************************************************************************
 * @file    gimbal_task.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/08/05
 * @brief   Sub-gimbal task header
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include "motor.h"
#include "controller.h"
#include "system_identification.h"
#include "stdbool.h"

#define GIMBAL_TASK_PERIOD 1

#define GIMBAL_MAX_CRUISEYAW 0.0f
#define GIMBAL_MAX_CRUISEPITCH 0.0f
#define GIMBAL_MIN_CRUISEPITCH -15.0f

/* Gimbal motor defines */
#define YAW_MOTOR_ID 0x207
#define PITCH_MOTOR_ID 0x205
#define YAW_MOTOR_ZERO_OFFSET 4002
#define PITCH_MOTOR_ZERO_OFFSET 4434
#define SLAMYAW_MOTOR_ZERO_OFFSET 240

/* Gimbal attitude limits */
#define PITCH_MIN_HARD_LIMIT -20
#define PITCH_MAX_HARD_LIMIT 5

#define PITCH_MIN_ATTACK_OUTPOST 0
#define PITCH_MAX_ATTACK_OUTPOST 20

/* Gimbal control ratios */
#define RC_STICK_YAW_RATIO 1
#define RC_STICK_PITCH_RATIO 0.6f
#define RC_MOUSE_YAW_RATIO 0.75f
#define RC_MOUSE_PITCH_RATIO -1.5f

/* Yaw velocity PID */
#define YAW_V_PID_MAXOUT 25000
#define YAW_V_PID_MAXINTEGRAL 10000
#define YAW_V_PID_KP 800
#define YAW_V_PID_KI 600
#define YAW_V_PID_KD 0
#define YAW_V_PID_LPF 0.000001f

/* Yaw angle PID */
#define YAW_A_PID_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define YAW_A_PID_MAXINTEGRAL 2.0f
#define YAW_A_PID_KP 2.5f
#define YAW_A_PID_KI 0
#define YAW_A_PID_KD 0.2f
#define YAW_A_PID_LPF 0.0001f
#define YAW_A_PID_D_LPF 0.023f

/* Yaw velocity feedforward */
#define YAW_V_FFC_MAXOUT 25000
#define YAW_V_FCC_C0 2
#define YAW_V_FCC_C1 0.37f
#define YAW_V_FCC_C2 0
#define YAW_V_FCC_LPF 0.005f

/* Yaw angle feedforward */
#define YAW_A_FFC_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define YAW_A_FCC_C0 0
#define YAW_A_FCC_C1 0.005f
#define YAW_A_FCC_C2 0
#define YAW_A_FCC_LPF 0.001f

/* Pitch velocity PID */
#define PITCH_V_PID_MAXOUT 25000
#define PITCH_V_PID_MAXINTEGRAL 10000
#define PITCH_V_PID_KP 400
#define PITCH_V_PID_KI 0
#define PITCH_V_PID_KD 0
#define PITCH_V_PID_LPF 0.001f

/* Pitch angle PID */
#define PITCH_A_PID_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define PITCH_A_PID_MAXINTEGRAL 1.0f
#define PITCH_A_PID_KP 2
#define PITCH_A_PID_KI 0
#define PITCH_A_PID_KD 0.09f
#define PITCH_A_PID_KP_AIMASSIST 0.55f
#define PITCH_A_PID_KI_AIMASSIST 0.1f
#define PITCH_A_PID_KD_AIMASSIST 0
#define PITCH_A_PID_LPF 0.0001f
#define PITCH_A_PID_D_LPF 0.001f

/* Pitch velocity feedforward */
#define PITCH_V_FFC_MAXOUT 18000
#define PITCH_V_FCC_C0 3
#define PITCH_V_FCC_C1 0.75f
#define PITCH_V_FCC_C2 0
#define PITCH_V_FCC_LPF 0.000f

/* Pitch angle feedforward */
#define PITCH_A_FFC_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define PITCH_A_FCC_C0 0
#define PITCH_A_FCC_C1 0.0175f
#define PITCH_A_FCC_C2 0
#define PITCH_A_FCC_LPF 0.003f

#define YAW_INCREMENT 0.06f
#define YAW_REFANGLE_LPF 0.15f

#define PITCH_INCREMENT (0.12f * 0.5f)
#define PITCH_REFANGLE_LPF 0.1f

#define YAW_MOTOR_MAXOUT 30000
#define PITCH_MOTOR_MAXOUT 30000

#define YAW_ANGLE_MIN 90
#define YAW_ANGLE_MAX 270

/* Gimbal modes */
enum
{
    NORMAL_MODE = 0X00,
    AIMASSIST_MODE = 0x01,
    FOLLOW_MODE = 0x02,
    COUNT_MODE = 0x04,
    BATTLE_MODE = 0x08,
    ADJUST_MODE = 0x10,
    Gimbal_Reserve6 = 0x20,
    Gimbal_Reserve7 = 0x40,
    Gimbal_Reserve8 = 0x80,
};

enum
{
    wait_command = 0x00,
    listen_lidar = 0x01,
    find_target = 0x02,
    lost_target = 0x04,
};

typedef struct
{
    FirstOrderSI_t YawSI;
    FirstOrderSI_t PitchSI;

    uint8_t ResetFlag;

    float Q0;
    float Q1;
    float Q2;
    float R;
    float lambda;
} GimbalSI_t;

typedef struct _GimbalControl
{
    Motor_t YawMotor;
    Motor_t PitchMotor;

    float YawAngle;
    float PitchAngle;
    float EncoderYawAngle;
    float EncoderPitchAngle;
    float YawAngularVelocity;
    float PitchAngularVelocity;
    float FollowYawVelocity;
    float FollowCoef;

    TD_t YawRefAngularVelocityTD;
    TD_t PitchRefAngularVelocityTD;
    TD_t YawRefAngleTD;
    TD_t PitchRefAngleTD;

    float YawRefAngularVelocity;
    float PitchRefAngularVelocity;
    float YawCtrlAngle;
    float PitchCtrlAngle;
    float YawRefAngle;
    float PitchRefAngle;
    float TempYawRefAngle;

    float FilteredYawRefAngle;
    float FilteredPitchRefAngle;

    float LastYawCtrlAngle;
    float LastPitchCtrlAngle;
    float LastFilteredYawRefAngle;
    float LastFilteredPitchRefAngle;

    float rcStickYawRatio;
    float rcStickPitchRatio;
    float rcMouseYawRatio;
    float rcMousePitchRatio;

    float YawIncrement;
    int8_t YawCruiseDirection;
    int8_t LastYawCruiseDirection;
    uint16_t YawCruiseCount;
    float YawRefAngleLPF;

    float PitchIncrement;
    int8_t PitchCruiseDirection;
    int8_t LastPitchCruiseDirection;
    float PitchRefAngleLPF;

    uint8_t isReached;
    uint8_t Status;
    float ChassisOmega[3];

    uint8_t Mode;
    uint8_t ModeLast;

    float PitchMinHardLimit;
    float PitchMaxHardLimit;
    float PitchCruiseMax;
    float PitchCruiseMin;
    float PitchMinIMU;
    float PitchMaxIMU;

    uint8_t allow_to_attack_outpost;

    uint8_t InPositionFlag;

    uint8_t Slope;
    uint8_t isrollover;

    uint8_t ReachFlag;
    uint8_t DecisionPlace;
} Gimbal_t;

typedef struct
{
    float yaw;
    float pitch;
} Visual_Calibration_t;

typedef struct
{
    Motor_t YawMotor;

    float TotalTheta;
    float Theta;
    float FollowTheta;
    float YawMotorOutput;
} SLAM_t;

enum
{
    LaserOff = 0,
    LaserOn = 1,
};

enum
{
    VelocityMode = 0,
    AngleMode = 1,
};

extern Gimbal_t Gimbal;
extern Visual_Calibration_t Visual_Calibration;
extern SLAM_t SLAM;
extern uint8_t FOLLOW_Data_Buf[24];
extern uint8_t FOLLOW_Update;
extern uint8_t ResetFlag;

void Gimbal_Init(void);
void Gimbal_Control(void);
void GetAngularVelocity(uint8_t *buff);

#endif
