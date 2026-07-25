/**
 ******************************************************************************
 * @file    gimbal_task.c
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/08/05
 * @brief   Sub-gimbal task implementation
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
Visual_Calibration_t Visual_Calibration = {0};
GimbalSI_t GimbalSI;
SLAM_t SLAM = {0};

uint8_t GimbalTuningEnable = 0;

float c[3] = {0};

uint32_t Gimbal_DWT_Count = 0;
static float dt = 0, t = 0;
uint32_t CAN_SEND_ERROR_COUNT = 0;
static float CatchTime;
float AimAssistYaw, AimAssistPitch;

uint8_t debugMode = 3, ControlerMode = 0;
float yaw_offset, pitch_offset;
float YawOmega = 0, PitchOmega = 0, YawAmp = 0, PitchAmp = 0, YawOffset = 0, PitchOffset = 0, YawDisturbance = 0, PitchDisturbance = 0;

float ChassisCoef = 504 / 8.3, ang_vel_scale = 1 / 0.95f, ChassisTorque, ChassisTorqueLPF, ChassisTorqueLPFcoef = 0.03;
uint8_t GameStatus = 0;
float preYaw;
int FollowCount = 0;

uint8_t ResetFlag;

uint8_t is_enable_calibra_debug = 0;

static void GimbalSI_Init(void);
static void Gimbal_Set_Mode(void);
static void Gimbal_Get_CtrlValue(void);
static void Gimbal_CtrlValue_Limit(void);
static void Gimbal_Set_Control(void);
static void Send_Gimbal_Current(void);
static void GimbalSI_Calculate(void);

void Gimbal_Init(void)
{
    Gimbal.PitchMotor.Direction = 0;

    Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
    Gimbal.PitchMotor.zero_offset = PITCH_MOTOR_ZERO_OFFSET;
    SLAM.YawMotor.zero_offset = SLAMYAW_MOTOR_ZERO_OFFSET;

    Gimbal.YawMotor.CAN_ID = YAW_MOTOR_ID;
    Gimbal.PitchMotor.CAN_ID = PITCH_MOTOR_ID;

    Gimbal.PitchMinHardLimit = PITCH_MIN_HARD_LIMIT;
    Gimbal.PitchMaxHardLimit = PITCH_MAX_HARD_LIMIT;

    Gimbal.rcStickYawRatio = RC_STICK_YAW_RATIO;
    Gimbal.rcStickPitchRatio = RC_STICK_PITCH_RATIO;
    Gimbal.rcMouseYawRatio = RC_MOUSE_YAW_RATIO;
    Gimbal.rcMousePitchRatio = RC_MOUSE_PITCH_RATIO;

    Gimbal.YawCruiseDirection = 1;
    Gimbal.LastYawCruiseDirection = 1;
    Gimbal.PitchCruiseDirection = 1;
    Gimbal.LastPitchCruiseDirection = 1;

    Gimbal.YawIncrement = YAW_INCREMENT;
    Gimbal.YawRefAngleLPF = YAW_REFANGLE_LPF;
    Gimbal.PitchIncrement = PITCH_INCREMENT;
    Gimbal.PitchRefAngleLPF = PITCH_REFANGLE_LPF;
    AimAssist.Lpf = 0.016f;

    TD_Init(&Gimbal.YawRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.PitchRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.YawRefAngleTD, 2000000, 0.003);
    TD_Init(&Gimbal.PitchRefAngleTD, 2000000, 0.003);

    /* Yaw Motor Init */
    PID_Init(&Gimbal.YawMotor.PID_Torque, 30000, 30000, 0, 0, 300, 0, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1; c[1] = 0; c[2] = 0;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Torque, 30000, c, 0, 4, 4);
    Gimbal.YawMotor.Ke = 0;

    PID_Init(&Gimbal.YawMotor.PID_Velocity, YAW_V_PID_MAXOUT, YAW_V_PID_MAXINTEGRAL, 0,
             YAW_V_PID_KP, YAW_V_PID_KI, YAW_V_PID_KD, 0, 0, YAW_V_PID_LPF, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = YAW_V_FCC_C0; c[1] = YAW_V_FCC_C1; c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Velocity, YAW_V_FFC_MAXOUT, c, YAW_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.YawMotor.PID_Angle, YAW_A_PID_MAXOUT, YAW_A_PID_MAXINTEGRAL, 0,
             YAW_A_PID_KP, YAW_A_PID_KI, YAW_A_PID_KD, 0, 0, YAW_A_PID_LPF, YAW_A_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = YAW_A_FCC_C0; c[1] = YAW_A_FCC_C1; c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 5, 5);
    Gimbal.YawMotor.Max_Out = YAW_MOTOR_MAXOUT;

    /* Pitch Motor Init */
    PID_Init(&Gimbal.PitchMotor.PID_Torque, 30000, 30000, 0, 0, 300, 0, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1; c[1] = 0; c[2] = 0;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Torque, 30000, c, 0, 4, 4);
    Gimbal.PitchMotor.Ke = 0;

    PID_Init(&Gimbal.PitchMotor.PID_Velocity, PITCH_V_PID_MAXOUT, PITCH_V_PID_MAXINTEGRAL, 0,
             PITCH_V_PID_KP, PITCH_V_PID_KI, PITCH_V_PID_KD, 0, 0, PITCH_V_PID_LPF, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = PITCH_V_FCC_C0; c[1] = PITCH_V_FCC_C1; c[2] = PITCH_V_FCC_C2;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT, c, PITCH_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.PitchMotor.PID_Angle, PITCH_A_PID_MAXOUT, PITCH_A_PID_MAXINTEGRAL, 0,
             PITCH_A_PID_KP, PITCH_A_PID_KI, PITCH_A_PID_KD, 0, 0, PITCH_A_PID_LPF, PITCH_A_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_A_FCC_C0; c[1] = PITCH_A_FCC_C1; c[2] = PITCH_A_FCC_C2;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Angle, PITCH_A_FFC_MAXOUT, c, PITCH_A_FCC_LPF, 5, 5);
    Gimbal.PitchMotor.Max_Out = PITCH_MOTOR_MAXOUT;

    GimbalSI_Init();
}

static void GimbalSI_Init(void)
{
    GimbalSI.Q0 = 0.001;
    GimbalSI.Q1 = 0.001;
    GimbalSI.Q2 = 0.001;
    GimbalSI.R = 10000;
    GimbalSI.lambda = 0.999;

    FirstOrderSI_Init(&GimbalSI.YawSI, 0, 0, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_Init(&GimbalSI.PitchSI, 0, 0, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
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
}

static void Gimbal_Set_Mode(void)
{
    switch (remote_control.switch_right)
    {
    case Switch_Up:
        Gimbal.Mode = NORMAL_MODE;
        break;
    case Switch_Middle:
        Gimbal.Mode = NORMAL_MODE;
        break;
    case Switch_Down:
        Gimbal.Mode = BATTLE_MODE;
        break;
    }
}

static void Gimbal_Get_CtrlValue(void)
{
    static uint16_t LastKeyCode = 0;
    static float TgtLostTime = -1.0f;
    static float HoldYawRefAngle = 0.0f;

    if (GlobalDebugMode != 2)
    {
        switch (Gimbal.Mode)
        {
        case NORMAL_MODE:
            Gimbal.YawAngle = INS.YawTotalAngle;
            Gimbal.PitchAngle = INS.Pitch;
            Gimbal.FollowCoef = 0.0f;
            break;
        case BATTLE_MODE:
            Gimbal.YawAngle = INS.YawTotalAngle;
            Gimbal.PitchAngle = INS.Pitch;
            break;
        }
    }
    else
    {
        Gimbal.YawAngle = Gimbal.YawMotor.AngleInDegree;
        Gimbal.PitchAngle = Gimbal.PitchMotor.AngleInDegree;
    }

    Gimbal.EncoderYawAngle = Gimbal.YawMotor.AngleInDegree;
    Gimbal.EncoderPitchAngle = Gimbal.PitchMotor.AngleInDegree;
    Gimbal.YawAngularVelocity = INS.Gyro[Z] * arm_cos_f32(Gimbal.EncoderPitchAngle / RADIAN_COEF) +
                                INS.Gyro[X] * arm_sin_f32(Gimbal.EncoderPitchAngle / RADIAN_COEF);
    Gimbal.PitchAngularVelocity = -INS.Gyro[Y];
    Gimbal.PitchRefAngularVelocity = remote_control.ch2 * Gimbal.rcStickPitchRatio;
    Gimbal.PitchRefAngularVelocity = TD_Calculate(&Gimbal.PitchRefAngularVelocityTD, Gimbal.PitchRefAngularVelocity);
    Gimbal.YawRefAngle = Gimbal.YawAngle - Gimbal.EncoderYawAngle;
    Gimbal.PitchRefAngle += Gimbal.PitchRefAngularVelocity * dt;

    if (Gimbal.allow_to_attack_outpost)
    {
        Gimbal.PitchCruiseMax = PITCH_MAX_ATTACK_OUTPOST;
        Gimbal.PitchCruiseMin = PITCH_MIN_ATTACK_OUTPOST;
    }
    else
    {
        Gimbal.PitchCruiseMax = GIMBAL_MAX_CRUISEPITCH;
        Gimbal.PitchCruiseMin = GIMBAL_MIN_CRUISEPITCH;
    }

    if (Gimbal.Mode == BATTLE_MODE)
    {
        if (AimAssist.Status != TgtLost)
        {
            TgtLostTime = -1.0f;
            CatchTime = t;
            AimAssistYaw = (AimAssistYaw * AimAssist.Lpf) / (AimAssist.Lpf + dt)
                         + (AimAssist.YawPosition * dt) / (AimAssist.Lpf + dt);
            AimAssistPitch = (AimAssistPitch * AimAssist.Lpf) / (AimAssist.Lpf + dt)
                           + (AimAssist.PitchPosition * dt) / (AimAssist.Lpf + dt);

            if (AimAssist.YawPosition - INS.Yaw < -180.0f)
                Gimbal.YawRefAngle = AimAssist.YawPosition + INS.YawTotalAngle - INS.Yaw + 360.0f;
            else if (AimAssist.YawPosition - INS.Yaw > 180.0f)
                Gimbal.YawRefAngle = AimAssist.YawPosition + INS.YawTotalAngle - INS.Yaw - 360.0f;
            else
                Gimbal.YawRefAngle = AimAssist.YawPosition + INS.YawTotalAngle - INS.Yaw;

            Gimbal.PitchRefAngle = AimAssist.PitchPosition;
            Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.PitchAngle - 10.5f, Gimbal.PitchAngle + 10.5f);

            HoldYawRefAngle = Gimbal.YawRefAngle;
        }
        else
        {
            if (TgtLostTime < 0.0f)
                TgtLostTime = t;

            if (t - TgtLostTime < 0.5f)
            {
                Gimbal.YawRefAngle = HoldYawRefAngle;
            }
            else
            {
                Gimbal.FollowCoef = 0.0f;

                if (Gimbal.Slope == 0)
                {
                    if (Gimbal.PitchRefAngle >= Gimbal.PitchCruiseMax)
                        Gimbal.PitchCruiseDirection = -1;
                    else if (Gimbal.PitchRefAngle <= Gimbal.PitchCruiseMin)
                        Gimbal.PitchCruiseDirection = 1;
                }
                else
                {
                    if (Gimbal.PitchRefAngle >= Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle + Gimbal.PitchCruiseMax)
                        Gimbal.PitchCruiseDirection = -1;
                    else if (Gimbal.PitchRefAngle <= Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle + Gimbal.PitchCruiseMin)
                        Gimbal.PitchCruiseDirection = 1;
                }

                Gimbal.PitchRefAngle += Gimbal.PitchIncrement * Gimbal.PitchCruiseDirection;
                Gimbal.FilteredYawRefAngle = Gimbal.YawRefAngle;
                Gimbal.FilteredPitchRefAngle = Gimbal.PitchRefAngle * dt / (Gimbal.PitchRefAngleLPF + dt)
                                             + Gimbal.LastFilteredPitchRefAngle * Gimbal.PitchRefAngleLPF / (Gimbal.PitchRefAngleLPF + dt);
            }
        }
    }

    if ((Gimbal.Mode == NORMAL_MODE) && AimAssist.miniPC_Online == 1)
    {
        if (AimAssist.Status != TgtLost)
        {
            TgtLostTime = -1.0f;
            CatchTime = t;
            AimAssistYaw = (AimAssistYaw * AimAssist.Lpf) / (AimAssist.Lpf + dt)
                         + (AimAssist.YawPosition * dt) / (AimAssist.Lpf + dt);
            AimAssistPitch = (AimAssistPitch * AimAssist.Lpf) / (AimAssist.Lpf + dt)
                           + (AimAssist.PitchPosition * dt) / (AimAssist.Lpf + dt);

            if (AimAssist.YawPosition - INS.Yaw < -180.0f)
                Gimbal.YawRefAngle = AimAssist.YawPosition + INS.YawTotalAngle - INS.Yaw + 360.0f;
            else if (AimAssist.YawPosition - INS.Yaw > 180.0f)
                Gimbal.YawRefAngle = AimAssist.YawPosition + INS.YawTotalAngle - INS.Yaw - 360.0f;
            else
                Gimbal.YawRefAngle = AimAssist.YawPosition + INS.YawTotalAngle - INS.Yaw;

            Gimbal.PitchRefAngle = AimAssist.PitchPosition;
            Gimbal.YawRefAngle = float_constrain(Gimbal.YawRefAngle, Gimbal.YawAngle - 10.5f, Gimbal.YawAngle + 10.5f);
            Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.PitchAngle - 10.5f, Gimbal.PitchAngle + 10.5f);

            HoldYawRefAngle = Gimbal.YawRefAngle;
        }
        else
        {
            if (TgtLostTime < 0.0f)
                TgtLostTime = t;

            if (t - TgtLostTime < 0.5f)
                Gimbal.YawRefAngle = HoldYawRefAngle;
        }
    }

    Gimbal.FilteredYawRefAngle = Gimbal.YawRefAngle;
    Gimbal.FilteredPitchRefAngle = Gimbal.PitchRefAngle;

    Gimbal.YawCtrlAngle = Gimbal.YawRefAngle;
    Gimbal.PitchCtrlAngle = Gimbal.PitchRefAngle;

    if ((is_TOE_Error(RC_TOE) || (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))) && (GlobalDebugMode != 2) || Gimbal.isrollover == 1)
    {
        Gimbal.YawCtrlAngle = Gimbal.YawAngle;
        Gimbal.PitchCtrlAngle = Gimbal.PitchAngle;

        Gimbal.YawRefAngle = Gimbal.YawAngle;
        Gimbal.FilteredYawRefAngle = Gimbal.YawAngle;
        Gimbal.PitchRefAngle = Gimbal.PitchAngle;

        Gimbal.YawRefAngleTD.x = Gimbal.YawAngle;
        Gimbal.YawRefAngleTD.dx = 0;
        Gimbal.PitchRefAngleTD.x = Gimbal.PitchAngle;
        Gimbal.PitchRefAngleTD.dx = 0;

        Gimbal.YawMotor.PID_Velocity.Iout = 0;
        Gimbal.PitchMotor.PID_Velocity.Iout = 0;

        Gimbal.YawMotor.FFC_Angle.Last_Ref = Gimbal.YawCtrlAngle;
        Gimbal.PitchMotor.FFC_Angle.Last_Ref = Gimbal.PitchCtrlAngle;
    }

    Gimbal_CtrlValue_Limit();
    LastKeyCode = remote_control.key_code;
    Gimbal.ModeLast = Gimbal.Mode;
    Gimbal.LastYawCtrlAngle = Gimbal.YawCtrlAngle;
}

static void Gimbal_CtrlValue_Limit(void)
{
    if (fabsf(Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle) > 9.0f)
    {
        Gimbal.PitchMinIMU = Gimbal.PitchMinHardLimit - (Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle);
        Gimbal.PitchMaxIMU = Gimbal.PitchMaxHardLimit - (Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle);
        Gimbal.Slope = 1;
    }
    else
    {
        Gimbal.PitchMinIMU = Gimbal.PitchMinHardLimit;
        Gimbal.PitchMaxIMU = Gimbal.PitchMaxHardLimit;
        Gimbal.Slope = 0;
    }

    Gimbal.PitchCtrlAngle = float_constrain(Gimbal.PitchCtrlAngle, Gimbal.PitchMinIMU, Gimbal.PitchMaxIMU);
    Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.PitchMinIMU, Gimbal.PitchMaxIMU);

    Gimbal.YawCtrlAngle = float_constrain(Gimbal.YawCtrlAngle,
                                          Gimbal.YawAngle - 40 - Gimbal.YawMotor.AngleInDegree,
                                          Gimbal.YawAngle + 40 - Gimbal.YawMotor.AngleInDegree);
    Gimbal.FilteredPitchRefAngle = float_constrain(Gimbal.FilteredPitchRefAngle, Gimbal.PitchMinIMU, Gimbal.PitchMaxIMU);

    Gimbal.LastFilteredYawRefAngle = Gimbal.FilteredYawRefAngle;
    Gimbal.LastFilteredPitchRefAngle = Gimbal.FilteredPitchRefAngle;

    if (GlobalDebugMode != 2 && GimbalTuningEnable == 0)
    {
        switch (Gimbal.Mode)
        {
        case NORMAL_MODE:
            Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
            Gimbal.PitchMotor.FFC_Velocity.c[1] = PITCH_V_FCC_C1;
            Gimbal.PitchMotor.FFC_Angle.MaxOut = PITCH_A_FFC_MAXOUT;
            break;
        }
    }

    if ((Gimbal.PitchCtrlAngle - Gimbal.PitchMinIMU < 0.5f || Gimbal.PitchMaxIMU - Gimbal.PitchCtrlAngle < 0.5f)
        && (GlobalDebugMode != 2 && GimbalTuningEnable == 0))
    {
        Gimbal.PitchMotor.FFC_Angle.MaxOut = 0;
        Gimbal.PitchMotor.FFC_Velocity.MaxOut = 0;
        Gimbal.PitchMotor.FFC_Torque.MaxOut = 0;
    }
}

static void Gimbal_Set_Control(void)
{
    float YawVelocityLoopInput = 0;
    static float compCoef = 0, gravityTorque;

    /* Yaw angle loop */
    if (is_enable_calibra_debug == 1)
    {
        Gimbal.YawCtrlAngle = Visual_Calibration.yaw;
        PID_Calculate(&Gimbal.YawMotor.PID_Angle, Gimbal.YawMotor.AngleInDegree, Gimbal.YawCtrlAngle);
    }
    else
    {
        PID_Calculate(&Gimbal.YawMotor.PID_Angle, Gimbal.YawAngle, Gimbal.YawCtrlAngle);
    }

    YawVelocityLoopInput = float_constrain(Gimbal.YawMotor.PID_Angle.Output + AimAssist.yaw_dot,
                                           -Gimbal.YawMotor.PID_Angle.MaxOut, Gimbal.YawMotor.PID_Angle.MaxOut);

    PID_Calculate(&Gimbal.YawMotor.PID_Velocity, Gimbal.YawAngularVelocity, YawVelocityLoopInput);
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Velocity, YawVelocityLoopInput);

    ChassisTorque = (Gimbal.YawAngularVelocity - Gimbal.YawMotor.OutputVel_RadPS * ang_vel_scale) * ChassisCoef;
    ChassisTorqueLPF = ChassisTorque * ChassisTorqueLPFcoef + ChassisTorqueLPF * (1 - ChassisTorqueLPFcoef);

    float YawTorqueLoopInput = float_constrain(
        Gimbal.YawMotor.PID_Velocity.Output + Gimbal.YawMotor.FFC_Velocity.Output + ChassisTorqueLPF + AimAssist.yaw_motor_current,
        -Gimbal.YawMotor.PID_Velocity.MaxOut, Gimbal.YawMotor.PID_Velocity.MaxOut);

    PID_Calculate(&Gimbal.YawMotor.PID_Torque, Gimbal.YawMotor.Real_Current, YawTorqueLoopInput);

    Gimbal.YawMotor.FFC_Torque.Output = float_constrain(
        Gimbal.YawMotor.FFC_Torque.c[0] * YawTorqueLoopInput,
        -Gimbal.YawMotor.FFC_Torque.MaxOut, Gimbal.YawMotor.FFC_Torque.MaxOut);

    Gimbal.YawMotor.Output = float_constrain(
        Gimbal.YawMotor.PID_Torque.Output + Gimbal.YawMotor.FFC_Torque.Output,
        -Gimbal.YawMotor.Max_Out, Gimbal.YawMotor.Max_Out);

    /* Pitch angle loop */
    gravityTorque = arm_cos_f32((INS.Pitch + 30) / RADIAN_COEF) * compCoef;

    if (is_enable_calibra_debug == 1)
    {
        Gimbal.PitchCtrlAngle = Visual_Calibration.pitch;
        PID_Calculate(&Gimbal.PitchMotor.PID_Angle, Gimbal.PitchMotor.AngleInDegree, Gimbal.PitchCtrlAngle);
    }
    else
    {
        PID_Calculate(&Gimbal.PitchMotor.PID_Angle, Gimbal.PitchAngle, Gimbal.PitchCtrlAngle);
    }

    Gimbal.PitchMotor.FFC_Angle.Output = float_constrain(
        Gimbal.PitchMotor.FFC_Angle.c[1] * Gimbal.PitchRefAngleTD.dx,
        -Gimbal.PitchMotor.FFC_Angle.MaxOut, Gimbal.PitchMotor.FFC_Angle.MaxOut);

    float PitchVelocityLoopInput = float_constrain(Gimbal.PitchMotor.PID_Angle.Output,
                                                   -Gimbal.PitchMotor.PID_Angle.MaxOut, Gimbal.PitchMotor.PID_Angle.MaxOut);

    PID_Calculate(&Gimbal.PitchMotor.PID_Velocity, Gimbal.PitchAngularVelocity, PitchVelocityLoopInput + AimAssist.pitch_dot);
    Feedforward_Calculate(&Gimbal.PitchMotor.FFC_Velocity, Gimbal.PitchRefAngleTD.dx);

    float PitchTorqueLoopInput = float_constrain(
        Gimbal.PitchMotor.PID_Velocity.Output + Gimbal.PitchMotor.FFC_Velocity.Output + gravityTorque,
        -Gimbal.PitchMotor.PID_Velocity.MaxOut, Gimbal.PitchMotor.PID_Velocity.MaxOut);

    PID_Calculate(&Gimbal.PitchMotor.PID_Torque, Gimbal.PitchMotor.Real_Current, PitchTorqueLoopInput);

    Gimbal.PitchMotor.Output = float_constrain(Gimbal.PitchMotor.PID_Torque.Output,
                                               -Gimbal.PitchMotor.Max_Out, Gimbal.PitchMotor.Max_Out);
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

    if (is_TOE_Error(RC_TOE))
    {
        if (Send_Motor_Current_5_8(&hcan2, 0, 0, 0, 0) == HAL_OK)
            ;
        else
            CAN_SEND_ERROR_COUNT++;
    }
    else
    {
        if (Send_Motor_Current_5_8(&hcan2, Gimbal.PitchMotor.Output, 0, Gimbal.YawMotor.Output, 0) == HAL_OK)
            ;
        else
            CAN_SEND_ERROR_COUNT++;
    }
}

static void GimbalSI_Calculate(void)
{
    if (GimbalSI.ResetFlag)
    {
        GimbalSI.ResetFlag = 0;

        for (uint8_t i = 0; i < 3; i++)
        {
            GimbalSI.YawSI.SI_EKF.xhat_data[i] = 0;
            GimbalSI.PitchSI.SI_EKF.xhat_data[i] = 0;
        }

        GimbalSI.YawSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.YawSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.YawSI.SI_EKF.P_data[8] = 10000000;
        GimbalSI.PitchSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.PitchSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.PitchSI.SI_EKF.P_data[8] = 10000000;
    }
    FirstOrderSI_EKF_Tuning(&GimbalSI.YawSI, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_EKF_Tuning(&GimbalSI.PitchSI, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);

    FirstOrderSI_Update(&GimbalSI.YawSI, Gimbal.YawMotor.PID_Torque.Ref, Gimbal.YawAngularVelocity, dt);
    FirstOrderSI_Update(&GimbalSI.PitchSI, Gimbal.PitchMotor.PID_Torque.Ref, Gimbal.PitchAngularVelocity, dt);
}

void GetAngularVelocity(uint8_t *buff)
{
    static float angularVelocity;

    angularVelocity = (int16_t)((buff[7] << 8) | buff[6]);
    Gimbal.FollowYawVelocity = (angularVelocity / 1000.0f) * dt / (dt + 0.005f)
                             + Gimbal.FollowYawVelocity * 0.005f / (dt + 0.005f);
    Gimbal.FollowYawVelocity = float_constrain(Gimbal.FollowYawVelocity, -2.0f, 2.0f);
}
