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
#include "arm_math.h"
#include "main.h"
#include "cmsis_os.h"

Gimbal_t Gimbal = {0};
float c[3];
static float t, dt;
uint32_t Gimbal_DWT_Count = 0;

void Gimbal_Init(void)
{
    Gimbal.YawMotor.CAN_ID = YAW_MOTOR_ID;
    Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
    Gimbal.YawMotor.Max_Out = YAW_MOTOR_MAXOUT;

    // 角度环初始化
    PID_Init(&Gimbal.YawMotor.PID_Angle,
             YAW_A_PID_MAXOUT, YAW_A_PID_MAXINTEGRAL, 0,
             YAW_A_PID_KP, YAW_A_PID_KI, YAW_A_PID_KD,
             0, 0,
             YAW_A_PID_LPF, YAW_A_PID_D_LPF, 3,
             360.0f,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter | ErrorWrapping);
    c[0] = YAW_A_FCC_C0, c[1] = YAW_A_FCC_C1, c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 5, 5);

    // 速度环初始化
    PID_Init(&Gimbal.YawMotor.PID_Velocity,
             YAW_V_PID_MAXOUT, YAW_V_PID_MAXINTEGRAL, 0,
             YAW_V_PID_KP, YAW_V_PID_KI, YAW_V_PID_KD,
             0, 0, YAW_V_PID_LPF, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = YAW_V_FCC_C0, c[1] = YAW_V_FCC_C1, c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Velocity, YAW_V_FFC_MAXOUT, c, YAW_V_FCC_LPF, 5, 5);

    // 力矩环初始化
    PID_Init(&Gimbal.YawMotor.PID_Torque,
             YAW_T_PID_MAXOUT, YAW_T_PID_MAXINTEGRAL, 0,
             YAW_T_PID_KP, YAW_T_PID_KI, YAW_T_PID_KD,
             0, 0, YAW_T_PID_LPF, 0, 0, 360.0f,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1, c[1] = 0, c[2] = 0;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Torque, 30000, c, 0, 4, 4);
}

void Gimbal_Control(void)
{
    dt = DWT_GetDeltaT(&Gimbal_DWT_Count);
    Gimbal.dt = dt;
    t += dt;

    Gimbal_Set_Mode();

    Gimbal_Get_CtrlValue();

    Gimbal_Set_Control();

    Send_Gimbal_Current();

    Emergency_Check();
}

static void Gimbal_Set_Mode(void)
{
    switch (remote_control.switch_right)
    {
    case Switch_Up:
        Gimbal.Mode = Gimbal_Normal;
        break;

    case Switch_Middle:
        Gimbal.Mode = Gimbal_Normal;
        break;

    case Switch_Down:
        Gimbal.Mode = Gimbal_Battle;
        break;
    }
}

static void Gimbal_Get_CtrlValue(void)
{
    Gimbal.YawAngle = INS.Yaw;     // 偏航角, -180° ~ +180°
    Gimbal.PitchAngle = INS.Pitch; // 俯仰角, 只用于投影计算

    Gimbal.EncoderYawAngle = Gimbal.YawMotor.AngleInDegree;

    float pitch_rad = Gimbal.PitchAngle / RADIAN_COEF;
    Gimbal.YawVelocity = INS.Gyro[2] * arm_cos_f32(pitch_rad) + INS.Gyro[1] * arm_sin_f32(pitch_rad);

    Gimbal.YawRefVelocity = -remote_control.ch1 * RC_YAW_RATIO;
    Gimbal.YawRefAngle += Gimbal.YawRefVelocity * Gimbal.dt;
    Gimbal.YawRefAngle = theta_format(Gimbal.YawRefAngle);

    if (is_TOE_Error(RC_TOE) || is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))
    {
        Gimbal.YawRefAngle = Gimbal.YawAngle;
        Gimbal.YawMotor.PID_Velocity.Iout = 0;
    }
}

static void Gimbal_Set_Control(void)
{
    float angle_out, velocity_out, torque_out;

    // 第1级: 角度环 → 期望角速度
    PID_Calculate(&Gimbal.YawMotor.PID_Angle, Gimbal.YawAngle, Gimbal.YawRefAngle);
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Angle, Gimbal.YawRefAngle);
    angle_out = Gimbal.YawMotor.PID_Angle.Output + Gimbal.YawMotor.FFC_Angle.Output;
    angle_out = float_constrain(angle_out,
        -Gimbal.YawMotor.PID_Angle.MaxOut, Gimbal.YawMotor.PID_Angle.MaxOut);

    // 第2级: 速度环 → 期望力矩电流
    PID_Calculate(&Gimbal.YawMotor.PID_Velocity, Gimbal.YawVelocity, angle_out);
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Velocity, angle_out);
    velocity_out = Gimbal.YawMotor.PID_Velocity.Output + Gimbal.YawMotor.FFC_Velocity.Output;
    velocity_out = float_constrain(velocity_out,
        -Gimbal.YawMotor.PID_Velocity.MaxOut, Gimbal.YawMotor.PID_Velocity.MaxOut);

    // 第3级: 力矩环 → 最终电流值
    PID_Calculate(&Gimbal.YawMotor.PID_Torque, Gimbal.YawMotor.Real_Current, velocity_out);
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Torque, velocity_out);
    torque_out = Gimbal.YawMotor.PID_Torque.Output + Gimbal.YawMotor.FFC_Torque.Output;
    Gimbal.YawMotor.Output = float_constrain(torque_out,
        -Gimbal.YawMotor.Max_Out, Gimbal.YawMotor.Max_Out);
}

static void Send_Gimbal_Current(void)
{
    if (is_TOE_Error(RC_TOE) || Gimbal.isRollover)
    {
        Send_RMD_Current_Single(&hcan1, 1, 0);
    }
    else
    {
        Send_RMD_Current_Single(&hcan1, 1, (int16_t)Gimbal.YawMotor.Output);
    }
}

static void Emergency_Check(void)
{
    static float last_roll = 0, last_pitch = 0;
    float roll_v = (INS.Roll - last_roll) / Gimbal.dt;
    float pitch_v = (INS.Pitch - last_pitch) / Gimbal.dt;
    last_pitch = INS.Pitch;
    last_roll = INS.Roll;
    int is_rolling = (fabsf(INS.Roll) > ROLL_THRESHOLD) || (fabsf(INS.Pitch) > PITCH_THRESHOLD) || (roll_v > RATE_THRESHOLD) || (pitch_v > RATE_THRESHOLD);

    if (is_rolling)
    {
        Gimbal.RollTime += Gimbal.dt;
        if (Gimbal.RollTime >= 3.0f)
            Gimbal.isRollover = 1;
    }
    else
    {
        Gimbal.isRollover = 0;
        Gimbal.RollTime = 0;
    }
}