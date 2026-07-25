/**
 ******************************************************************************
 * @file    gimbal_task.c
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/08/05
 * @brief   Gimbal task implementation
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "gimbal_task.h"
#include "remote_control.h"
#include "ins_task.h"
#include "QuaternionEKF.h"
#include "math.h"
#include "stdbool.h"
#include "user_lib.h"
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "motor.h"
#include "detect_task.h"
#include "controller.h"
#include "system_identification.h"
#include "arm_math.h"
#include "main.h"
#include "cmsis_os.h"
#include "aimassist_task.h"

Gimbal_t Gimbal = {0};
GimbalSI_t GimbalSI;
SLAM_t SLAM = {0};

uint8_t GimbalTuningEnable = 0;

static float rollingTime = 0;
static uint8_t rollingState = 0;
static uint8_t yaw_smooth_active = 0;

static float c[3] = {0};

uint32_t Gimbal_DWT_Count = 0;
static float dt = 0, t = 0;
uint32_t CAN_SEND_ERROR_COUNT = 0;

float AimAssistYaw, AimAssistPitch;

float ChassisCoef = 504 / 8.3f;
static float ang_vel_scale = 1 / 0.95f;
static float ChassisTorque, ChassisTorqueLPF;
static float ChassisTorqueLPFcoef = 0.03f;

float preYaw;

static void GimbalSI_Init(void);
static void Gimbal_Set_Mode(void);
static void Gimbal_Get_CtrlValue(void);
static void Gimbal_CtrlValue_Limit(void);
static void Gimbal_Set_Control(void);
static void Send_Gimbal_Current(void);
static void GimbalSI_Calculate(void);

void Gimbal_Init(void)
{
    Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
    Gimbal.SubYawMotor.zero_offset = SUB_YAW_MOTOR_ZERO_OFFSET;

    Gimbal.YawMotor.CAN_ID = YAW_MOTOR_ID;

    Gimbal.DepressionEncoderInDegree = GIMBAL_MAX_DEPRESSION;
    Gimbal.ElevationEncoderInDegree = GIMBAL_MAX_ELEVATION;

    Gimbal.rcStickYawRatio = RC_STICK_YAW_RATIO;

    Gimbal.YawCruiseDirection = 1;
    Gimbal.YawCruiseStepDeg = 0.05f;
    Gimbal.YawRefAngleLPF = YAW_REFANGLE_LPF;
    AimAssist.Lpf = 0.016f;

    TD_Init(&Gimbal.YawRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.YawRefAngleTD, 2000000, 0.003);

    PID_Init(&Gimbal.YawMotor.PID_Torque, 2000, 2000, 0,
             0, 300, 0, 0, 0, 0, 0, 0, 360.0f,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1; c[1] = 0; c[2] = 0;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Torque, 2000, c, 0, 4, 4);
    Gimbal.YawMotor.Ke = 0;

    PID_Init(&Gimbal.YawMotor.PID_Velocity, YAW_V_PID_MAXOUT, YAW_V_PID_MAXINTEGRAL, 0,
             YAW_V_PID_KP, YAW_V_PID_KI, YAW_V_PID_KD, 0, 0,
             YAW_V_PID_LPF, 0, 0, 360.0f,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = YAW_V_FCC_C0; c[1] = YAW_V_FCC_C1; c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Velocity, YAW_V_FFC_MAXOUT, c, YAW_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.YawMotor.PID_Angle, YAW_A_PID_MAXOUT, YAW_A_PID_MAXINTEGRAL, 0,
             YAW_A_PID_KP, YAW_A_PID_KI, YAW_A_PID_KD, 0, 0,
             YAW_A_PID_LPF, YAW_A_PID_D_LPF, 3, 360.0f,
             Integral_Limit | Trapezoid_Intergral | OutputFilter
             | DerivativeFilter | ErrorWrapping);
    c[0] = YAW_A_FCC_C0; c[1] = YAW_A_FCC_C1; c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 5, 5);
    Gimbal.YawMotor.Max_Out = YAW_MOTOR_MAXOUT;

    GimbalSI_Init();
}

static void GimbalSI_Init(void)
{
    GimbalSI.Q0 = 0.001f; GimbalSI.Q1 = 0.001f; GimbalSI.Q2 = 0.001f;
    GimbalSI.R = 10000; GimbalSI.lambda = 0.999f;
    FirstOrderSI_Init(&GimbalSI.YawSI, 0, 0,
                      GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2,
                      GimbalSI.R, GimbalSI.lambda);
}

void Gimbal_Control(void)
{
    dt = DWT_GetDeltaT(&Gimbal_DWT_Count);
    t += dt;

    Gimbal_Set_Mode();
    Gimbal_Get_CtrlValue();
    Gimbal_Set_Control();
    Send_Gimbal_Current();
    GimbalSI_Calculate();
    Emergency_Stop_Gimbal();
}

static void Gimbal_Set_Mode(void)
{
    switch (remote_control.switch_right)
    {
    case Switch_Up:
    case Switch_Middle:
        Gimbal.Mode = NORMAL_MODE;
        break;
    case Switch_Down:
        Gimbal.Mode = BATTLE_MODE;
        break;
    }

    if (Gimbal.Mode != BATTLE_MODE)
        yaw_smooth_active = 0;

    Gimbal.ModeSwitchCount++;
    if (Gimbal.Mode != Gimbal.ModeLast)
        Gimbal.ModeSwitchCount = 0;
}

static void Gimbal_Get_CtrlValue(void)
{
    static uint16_t LastKeyCode = 0;

    Gimbal.YawAngle = INS.Yaw;
    Gimbal.PitchAngle = INS.Pitch;

    Gimbal.EncoderYawAngle = Gimbal.YawMotor.AngleInDegree;
    Gimbal.EncoderPitchAngle = Gimbal.PitchMotor.AngleInDegree;

    Gimbal.YawAngularVelocity =
        INS.Gyro[Z] * arm_cos_f32(Gimbal.EncoderPitchAngle / RADIAN_COEF) +
        INS.Gyro[Y] * arm_sin_f32(Gimbal.EncoderPitchAngle / RADIAN_COEF);

    Gimbal.YawRefAngularVelocity = -remote_control.ch1 * Gimbal.rcStickYawRatio;
    Gimbal.YawRefAngle += Gimbal.YawRefAngularVelocity * dt;

    if (Gimbal.Mode == BATTLE_MODE)
    {
        if (AimAssist.Status != TgtLost)
        {
            yaw_smooth_active = 1;
            float entry_yaw = Gimbal.YawAngle;
            if (Gimbal.ModeSwitchCount == 0)
            {
                Gimbal.FilteredYawRefAngle = entry_yaw;
                Gimbal.YawRefAngleTD.x = entry_yaw;
                Gimbal.YawRefAngleTD.dx = 0;
            }
            Gimbal.YawRefAngle = entry_yaw;
            Gimbal.FilteredYawRefAngle = Gimbal.YawRefAngle;
        }
        else
        {
            yaw_smooth_active = 1;
            Gimbal.YawRefAngle += Gimbal.YawRefAngularVelocity * dt;
        }
    }

    if (yaw_smooth_active == 0)
    {
        if (Gimbal.YawRefAngle >= 180.0f)
        {
            Gimbal.YawRefAngle -= 360.0f;
            Gimbal.FilteredYawRefAngle -= 360.0f;
            Gimbal.YawRefAngleTD.x -= 360.0f;
        }
        if (Gimbal.YawRefAngle <= -180.0f)
        {
            Gimbal.YawRefAngle += 360.0f;
            Gimbal.FilteredYawRefAngle += 360.0f;
            Gimbal.YawRefAngleTD.x += 360.0f;
        }
        Gimbal.FilteredYawRefAngle = Gimbal.YawRefAngle;
    }

    Gimbal.YawCtrlAngle = TD_Calculate(&Gimbal.YawRefAngleTD, Gimbal.FilteredYawRefAngle);
    Gimbal.YawCtrlAngle = Gimbal.YawAngle
                        + theta_format(Gimbal.YawCtrlAngle - Gimbal.YawAngle);

    if (is_TOE_Error(RC_TOE) || is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))
    {
        Gimbal.YawCtrlAngle = Gimbal.YawAngle;
        Gimbal.YawRefAngle = Gimbal.YawAngle;
        Gimbal.FilteredYawRefAngle = Gimbal.YawAngle;
        Gimbal.YawRefAngleTD.x = Gimbal.YawAngle;
        Gimbal.YawRefAngleTD.dx = 0;
        Gimbal.YawMotor.PID_Velocity.Iout = 0;
        Gimbal.YawMotor.FFC_Angle.Last_Ref = Gimbal.YawCtrlAngle;
    }

    Gimbal_CtrlValue_Limit();
    LastKeyCode = remote_control.key_code;
    Gimbal.ModeLast = Gimbal.Mode;
}

static void Gimbal_CtrlValue_Limit(void)
{
    Gimbal.LastFilteredYawRefAngle = Gimbal.FilteredYawRefAngle;

    if (GlobalDebugMode != GIMBAL_DEBUG)
    {
        switch (Gimbal.Mode)
        {
        case AIMASSIST_MODE:
            Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
            Gimbal.YawMotor.FFC_Velocity.c[1] = 0;
            Gimbal.PitchMotor.FFC_Velocity.c[1] = 0;
            Gimbal.PitchMotor.FFC_Angle.MaxOut = 0;
            break;
        case NORMAL_MODE:
            Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
            Gimbal.PitchMotor.FFC_Velocity.c[1] = PITCH_V_FCC_C1;
            Gimbal.PitchMotor.FFC_Angle.MaxOut = PITCH_A_FFC_MAXOUT;
            break;
        }
    }
}

static void Gimbal_Set_Control(void)
{
    PID_Calculate(&Gimbal.YawMotor.PID_Angle, Gimbal.YawAngle, Gimbal.YawCtrlAngle);

    Gimbal.YawMotor.FFC_Angle.Output = float_constrain(
        Gimbal.YawMotor.FFC_Angle.c[1] * Gimbal.YawRefAngleTD.dx,
        -Gimbal.YawMotor.FFC_Angle.MaxOut,
        Gimbal.YawMotor.FFC_Angle.MaxOut);

    float YawVelocityLoopInput = float_constrain(
        Gimbal.YawMotor.PID_Angle.Output + Gimbal.YawMotor.FFC_Angle.Output,
        -Gimbal.YawMotor.PID_Angle.MaxOut,
        Gimbal.YawMotor.PID_Angle.MaxOut);

    PID_Calculate(&Gimbal.YawMotor.PID_Velocity, Gimbal.YawAngularVelocity, YawVelocityLoopInput);
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Velocity, Gimbal.YawRefAngleTD.dx);

    ChassisTorque = (Gimbal.YawAngularVelocity
                    - Gimbal.YawMotor.OutputVel_RadPS * ang_vel_scale) * ChassisCoef;
    ChassisTorqueLPF = ChassisTorque * ChassisTorqueLPFcoef
                     + ChassisTorqueLPF * (1 - ChassisTorqueLPFcoef);

    float YawTorqueLoopInput = float_constrain(
        Gimbal.YawMotor.PID_Velocity.Output + Gimbal.YawMotor.FFC_Velocity.Output,
        -Gimbal.YawMotor.PID_Velocity.MaxOut,
        Gimbal.YawMotor.PID_Velocity.MaxOut);
    PID_Calculate(&Gimbal.YawMotor.PID_Torque, Gimbal.YawMotor.Real_Current, YawTorqueLoopInput);

    Gimbal.YawMotor.Output = float_constrain(
        Gimbal.YawMotor.PID_Velocity.Output
        + Gimbal.YawMotor.FFC_Velocity.Output
        + ChassisTorqueLPF
        + Gimbal.SubYawTorque,
        -Gimbal.YawMotor.Max_Out,
        Gimbal.YawMotor.Max_Out);
}

static void Send_Gimbal_Current(void)
{
    static uint16_t qFrameNum;
    static float q[4];

    qFrameNum = Find_qFrame(INS.qFrame, (uint32_t)(INS_GetTimeline() - 100));
    memcpy(q, INS.qFrame[qFrameNum].q, sizeof(q));
    preYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);

    int16_t yaw1000 = (int16_t)(preYaw * 1000.0f);
    FOLLOW_Data_Buf[8] = yaw1000 & 0xFF;
    FOLLOW_Data_Buf[9] = (yaw1000 >> 8) & 0xFF;

    if ((is_TOE_Error(RC_TOE) && GlobalDebugMode != GIMBAL_DEBUG) || Gimbal.isrollover == 1)
    {
        if (Send_RMD_Current_Single(&hcan1, 1, 0) == HAL_OK);
        else CAN_SEND_ERROR_COUNT++;
    }
    else
    {
        if (Send_RMD_Current_Single(&hcan1, 1, Gimbal.YawMotor.Output) == HAL_OK);
        else CAN_SEND_ERROR_COUNT++;
    }
}

static void GimbalSI_Calculate(void)
{
    if (GimbalSI.ResetFlag)
    {
        GimbalSI.ResetFlag = 0;
        for (uint8_t i = 0; i < 3; i++)
            GimbalSI.YawSI.SI_EKF.xhat_data[i] = 0;
        GimbalSI.YawSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.YawSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.YawSI.SI_EKF.P_data[8] = 10000000;
    }
    FirstOrderSI_EKF_Tuning(&GimbalSI.YawSI,
                            GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2,
                            GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_Update(&GimbalSI.YawSI, Gimbal.YawMotor.PID_Torque.Ref,
                        Gimbal.YawAngularVelocity, dt);
}

void GetAngularVelocity(uint8_t *buff)
{
    static float angularVelocity;
    Gimbal.isReached = 1;

    angularVelocity = (int16_t)((buff[7] << 8) | buff[6]);
    Gimbal.FollowYawVelocity = (angularVelocity / 1000.0f) * dt / (dt + 0.005f)
                             + Gimbal.FollowYawVelocity * 0.005f / (dt + 0.005f);
    Gimbal.FollowYawVelocity = float_constrain(Gimbal.FollowYawVelocity, -2.0f, 2.0f);
}

void Emergency_Stop_Gimbal(void)
{
    static float last_roll, last_pitch;

    float roll_rate = fabsf(INS.Roll - last_roll) / dt;
    float pitch_rate = fabsf(INS.Pitch - last_pitch) / dt;
    last_roll = INS.Roll;
    last_pitch = INS.Pitch;

    rollingState = (fabsf(INS.Roll) > ROLL_THRESHOLD) ||
                   (fabsf(INS.Pitch) > PITCH_THRESHOLD) ||
                   (fabsf(roll_rate) > RATE_THRESHOLD) ||
                   (fabsf(pitch_rate) > RATE_THRESHOLD);

    if (rollingState)
    {
        rollingTime += dt;
        if (rollingTime >= 3.0f) Gimbal.isrollover = 1;
    }
    else
    {
        Gimbal.isrollover = 0;
        rollingTime = 0;
    }
}
