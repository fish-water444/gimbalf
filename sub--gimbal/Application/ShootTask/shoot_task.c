#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "bsp_CAN.h"
#include "remote_control.h"
#include "user_lib.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "motor.h"

Shoot_t Shoot = {0};
static float t, dt;
uint32_t Shoot_DWT_Count = 0;

// 前置声明
static void Shoot_Set_Mode(void);
static void Shoot_Set_Control(void);
static void Send_Shoot_Current(void);

void Shoot_Init(void)
{
    Shoot.RefFricSpeed = 5800; // RPM
    Shoot.isFricOn = 0;

    // 摩擦轮速度环 PID (M3508 内部电调已有力矩环，只需速度环)
    for (int i = 0; i < 2; i++)
    {
        Shoot.FricMotor[i].CAN_ID = 0x203 + i;
        PID_Init(&Shoot.FricMotor[i].PID_Velocity,
                 16384, 16384, 0, // MaxOut, IntegralLimit, DeadBand
                 15, 0, 0,        // Kp, Ki, Kd
                 0, 0, 0.001f, 0.001f, 5,
                 Integral_Limit | DerivativeFilter | OutputFilter);
        Shoot.FricMotor[i].Max_Out = 16384;
    }
    // 拨弹盘速度环 PID
    Shoot.TriggerMotor.CAN_ID = 0x202;
    Shoot.TriggerMotor.Direction = NEGATIVE;
    PID_Init(&Shoot.TriggerMotor.PID_Velocity,
             10000, 5000, 0, -10, 0, 0,
             500, 1000, 0.001f, 0.001f, 5,
             Integral_Limit | ErrorHandle);
    Shoot.TriggerMotor.Max_Out = 10000;
}

float ShootFreqToRPM(float shootfreq)
{
    return shootfreq / 8 * 36 * 60;
}

void Shoot_Control(void)
{
    dt = DWT_GetDeltaT(&Shoot_DWT_Count);
    t += dt;

    Shoot_Set_Mode();

    Shoot_Set_Control();

    Send_Shoot_Current();
}

static void Shoot_Set_Mode(void)
{
    switch (remote_control.switch_left)
    {
    case Switch_Up:
        Shoot.isFricOn = 1;
        Shoot.Mode = KeepShooting;
        break;

    case Switch_Middle:
        Shoot.isFricOn = 1;
        Shoot.Mode = WaitcShooting;
        break;

    case Switch_Down:
        Shoot.isFricOn = 0;
        Shoot.Mode = NoShooting;
        break;
    }
}


static void Shoot_Set_Control(void)
{
    // 1. 摩擦轮速度控制
    float target_fric = Shoot.isFricOn ? Shoot.RefFricSpeed : 0;
    Motor_Speed_Calculate(&Shoot.FricMotor[0],
                          Shoot.FricMotor[0].Velocity_RPM, -target_fric); // 左轮反转
    Motor_Speed_Calculate(&Shoot.FricMotor[1],
                          Shoot.FricMotor[1].Velocity_RPM,  target_fric); // 右轮正转

    // 更新当前摩擦轮转速（取两轮平均值）
    Shoot.FricSpeed = fabsf((Shoot.FricMotor[0].Velocity_RPM
                           + Shoot.FricMotor[1].Velocity_RPM) * 0.5f);

    // 2. 拨弹盘速度控制（摩擦轮未就绪 或 非发射模式 → 不转）
    int fric_ready = (Shoot.FricSpeed > Shoot.RefFricSpeed * 0.9f);
    int in_shooting = (Shoot.Mode == KeepShooting || Shoot.Mode == WaitcShooting);

    if (fric_ready && in_shooting) 
    {
        Shoot.TriggerSpeed = ShootFreqToRPM(Shoot.RefShootFreq);
    } 
    else 
    {
        Shoot.TriggerSpeed = 0;
    }

    Motor_Speed_Calculate(&Shoot.TriggerMotor,
                          Shoot.TriggerMotor.Velocity_RPM, Shoot.TriggerSpeed);
}

static void Send_Shoot_Current(void)
{
    // 离线/翻倒 → 全部断电
    if (is_TOE_Error(RC_TOE) || Gimbal.isRollover) {
        Send_Motor_Current_1_4(&hcan2, 0, 0, 0, 0);
        return;
    }

    // 电机1(ID 0x201)空闲, 电机2=拨弹盘(0x202), 电机3=左摩擦轮(0x203), 电机4=右摩擦轮(0x204)
    Send_Motor_Current_1_4(&hcan2,
        0,                                  
        Shoot.TriggerMotor.Output,          
        Shoot.FricMotor[0].Output,         
        Shoot.FricMotor[1].Output);        
}

