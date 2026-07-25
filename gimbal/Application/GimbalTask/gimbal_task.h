#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include "motor.h"
#include "controller.h"
#include "system_identification.h"
#include "stdbool.h"

#define GIMBAL_TASK_PERIOD 1

#define GIMBAL_MAX_CRUISEYAW 67.5f
#define GIMBAL_MAX_CRUISEPITCH 6.0f
#define GIMBAL_MIN_CRUISEPITCH -13.5f
#define GIMBAL_MAX_DEPRESSION -20
#define GIMBAL_MAX_ELEVATION 25

#define YAW_MOTOR_ID 0x141
#define SUB_YAW_MOTOR_ID 0x207
#define PITCH_MOTOR_ID 0x206
#define YAW_MOTOR_ZERO_OFFSET 10201
#define SUB_YAW_MOTOR_ZERO_OFFSET 4002
#define PITCH_MOTOR_ZERO_OFFSET 6950
#define SLAMYAW_MOTOR_ZERO_OFFSET 240

/* RC sensitivity */
#define RC_STICK_YAW_RATIO 1
#define RC_STICK_PITCH_RATIO 0.6f
#define RC_MOUSE_YAW_RATIO 0.75f
#define RC_MOUSE_PITCH_RATIO -1.5f

/* Yaw velocity PID */
#define YAW_V_PID_MAXOUT 18000
#define YAW_V_PID_MAXINTEGRAL 10000
#define YAW_V_PID_KP 400
#define YAW_V_PID_KI 100
#define YAW_V_PID_KD 0
#define YAW_V_PID_LPF 0.015f

/* Yaw angle PID */
#define YAW_A_PID_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define YAW_A_PID_MAXINTEGRAL 1.0f
#define YAW_A_PID_KP 0.55f
#define YAW_A_PID_KI 0
#define YAW_A_PID_KD 0.02f
#define YAW_A_PID_KP_AIMASSIST 0.6f
#define YAW_A_PID_KI_AIMASSIST 1.5f
#define YAW_A_PID_KD_AIMASSIST 0.004f
#define YAW_A_PID_LPF 0.0001f
#define YAW_A_PID_D_LPF 0.001f

/* Yaw velocity feedforward */
#define YAW_V_FFC_MAXOUT 0
#define YAW_V_FCC_C0 0.8f
#define YAW_V_FCC_C1 0.2f
#define YAW_V_FCC_C2 0
#define YAW_V_FCC_LPF 0.005f

/* Yaw angle feedforward */
#define YAW_A_FFC_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define YAW_A_FCC_C0 0
#define YAW_A_FCC_C1 0.0175f
#define YAW_A_FCC_C2 0
#define YAW_A_FCC_LPF 0.001f

/* Pitch velocity PID */
#define PITCH_V_PID_MAXOUT 18000
#define PITCH_V_PID_MAXINTEGRAL 10000
#define PITCH_V_PID_KP -4500
#define PITCH_V_PID_KI -8000
#define PITCH_V_PID_KD 0
#define PITCH_V_PID_LPF 0.001f

/* Pitch angle PID */
#define PITCH_A_PID_MAXOUT (320.0f / 60.0f * 2.0f * 3.14159f)
#define PITCH_A_PID_MAXINTEGRAL 1.0f
#define PITCH_A_PID_KP 0.55f
#define PITCH_A_PID_KI 0
#define PITCH_A_PID_KD 0.025f
#define PITCH_A_PID_KP_AIMASSIST 0.55f
#define PITCH_A_PID_KI_AIMASSIST 0
#define PITCH_A_PID_KD_AIMASSIST 0.025f
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

#define YAW_REFANGLE_LPF 0.01f
#define PITCH_INCREMENT (0.12f * 0.5f)
#define PITCH_REFANGLE_LPF 0.1f

#define YAW_MOTOR_MAXOUT 2000
#define PITCH_MOTOR_MAXOUT 30000

#define ROLL_THRESHOLD 30.0f
#define PITCH_THRESHOLD 30.0f
#define RATE_THRESHOLD 10.0f

enum GimbalMode {
    NORMAL_MODE = 0X00,
    AIMASSIST_MODE = 0x01,
    FOLLOW_MODE = 0x02,
    COUNT_MODE = 0x04,
    BATTLE_MODE = 0x08,
    TEST_MODE = 0x10,
};

enum GimbalStatus {
    WAIT_COMMAND = 0x00,
    LISTEN_LIDAR = 0x01,
    FIND_TARGET = 0x02,
    LOST_TARGET = 0x04,
};

typedef struct {
    FirstOrderSI_t YawSI;
    uint8_t ResetFlag;
    float Q0, Q1, Q2, R, lambda;
} GimbalSI_t;

typedef struct {
    Motor_t YawMotor;
    Motor_t SubYawMotor;
    Motor_t PitchMotor;         /* deprecated, kept for CAN protocol compat */

    float YawAngle;
    float SubYawAngle;
    float LastSubYawAngle;
    float PitchAngle;
    float EncoderYawAngle;
    float EncoderPitchAngle;
    float YawAngularVelocity;
    float PitchAngularVelocity;
    float FollowYawVelocity;

    TD_t YawRefAngularVelocityTD;
    TD_t YawRefAngleTD;

    float YawRefAngularVelocity;
    float YawCtrlAngle;
    float YawRefAngle;
    float InitDeviateTotalTheta;
    float DeviateTotalTheta;
    float InitDeviateTheta;
    float DeviateTheta;
    float FollowTheta;

    float FilteredYawRefAngle;
    float LastYawCtrlAngle;
    float LastFilteredYawRefAngle;

    float rcStickYawRatio;
    float rcStickPitchRatio;
    float rcMouseYawRatio;
    float rcMousePitchRatio;

    float YawCruiseStepDeg;
    int8_t YawCruiseDirection;
    uint16_t YawCruiseCount;
    float YawAngleAccum;
    float YawRefAngleLPF;

    uint8_t isReached;
    uint8_t Status;
    float ChassisOmega[3];

    uint16_t ModeSwitchCount;

    uint8_t LaserState;
    uint8_t Mode;
    uint8_t ModeLast;

    float DepressionEncoderInDegree;
    float ElevationEncoderInDegree;

    float SubYawTorque;
    uint8_t isHero;
    uint8_t isLock;
    uint8_t isTarget;
    uint8_t Slope;
    uint8_t isrollover;
    uint8_t ReachFlag;
    uint8_t DecisionPlace;
    float position_x;
    float position_y;
} Gimbal_t;

typedef struct {
    Motor_t YawMotor;
    float TotalTheta;
    float Theta;
    float FollowTheta;
    float YawMotorOutput;
} SLAM_t;

enum {
    LASER_OFF = 0,
    LASER_ON = 1,
};

enum {
    VELOCITY_MODE = 0,
    ANGLE_MODE = 1,
};

extern Gimbal_t Gimbal;
extern SLAM_t SLAM;
extern uint8_t FOLLOW_Data_Buf[24];
extern uint8_t FOLLOW_Update;
extern uint8_t InPositionFlag;

void Gimbal_Init(void);
void Gimbal_Control(void);
void GetAngularVelocity(uint8_t *buff);
void Emergency_Stop_Gimbal(void);

#endif
