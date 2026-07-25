#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "bsp_PWM.h"
#include "bsp_CAN.h"
#include "user_lib.h"
#include "detect_task.h"
#include "judgement_info.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "bsp_PWM.h"
#include "bsp_CAN.h"
#include "user_lib.h"
#include "judgement_info.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "aimassist_task.h"
#include "math.h"
#include "usart.h"
#include "user_lib.h"
#include "motor.h"

Motor_t Motor = {0};
float tempSpeed;
Shoot_t Shoot = {0};
Heat_t Heat = {0};
Coolrate_t Cool_rate = {0};
FSM_t g_fsm = {0};

uint8_t Debug_Mode = 0; // 决策和导航调试时用，防止打人
uint8_t set_posture_mode = 0, debug_posture = 3;

uint32_t Shoot_DWT_Count = 0;
static float dt = 0, t = 0;

uint8_t ShootDebugMode = 1;

float isOnRange;
float SpeedInBulletsPerSec = 22;
float NumsOfOneShot = 1;
float ShootDelayInMs = 0;
float DebugAmp = 1000, DebugOmega = 10, DebugOffset = 5000;
float Q0 = 35, Q1 = 0.0000000000001, Q2 = 0.000001, R = 1000;
uint8_t isBlocked;
uint32_t g_total_bullet = 0;   // 总共发弹量，离线热量控制
uint32_t g_pending_bullet = 0; // 已供入弹链、尚未被摩擦轮电流确认的弹丸数

static uint16_t LastKeyCode = 0;
static void Shoot_Set_Mode(void);
static void Shoot_Get_CtrlValue(void);
static void Shoot_Set_Control(void);
static void Send_Shoot_Output(void);
static void Heat_Init(void);      // 热量初始化
static void Heat_Detection(void); // 热量检测
static void Get_Cool_Rate(void);  // 获取场地交互增益冷却速率
static float Heat_Judgment(void); // 热量控制

void Shoot_Init(void)
{
    Shoot.ShootMode = NoShooting;

    Shoot.RefBulletVelocity = 28.5;
    Shoot.BulletVelocity = Shoot.RefBulletVelocity;

    Shoot.TriggerMotor.Direction = NEGATIVE;
    PID_Init(
        &Shoot.TriggerMotor.PID_Velocity, 10000, 5000, 0, -10, 0, 0, 500, 1000, 0.001, 0.001, 5, Integral_Limit | ErrorHandle);
    Shoot.TriggerMotor.Max_Out = 10000.0f;

    TD_Init(&Shoot.FricTD, 50000, 0.001);
    Shoot.RefFricSpeed = 0;

    for (uint8_t i = 0; i < 2; i++)
    {
        Shoot.FricMotor[i].CAN_ID = 0x203 + i;
        PID_Init(
            &Shoot.FricMotor[i].PID_Velocity, 16384, 16384, 0, 15, 80 * 0, 0, 500, 1000, 0.001, 0.001, 5,
            Integral_Limit | DerivativeFilter | OutputFilter);
        Shoot.InitialKi = Shoot.FricMotor[i].PID_Velocity.Ki;
        float c[3] = {0.005, 0, 0};
        Feedforward_Init(&Shoot.FricMotor[i].FFC_Velocity, 16384, c, 0, 4, 4);
        // FirstOrderSI_Init(&Shoot.FricMotorSI[i], 0.009, 0.23, 0.01, 0.001, 0.001, 10000, 1);
        Shoot.FricMotor[i].AccelerationLpf = 50;
        Shoot.FricMotor[i].Max_Out = 16384.0f;
    }
    Shoot.isLidOpen = 1;

    Shoot.PeriodicShootOnMs = 800;  // 打弹窗口500ms
    Shoot.PeriodicShootOffMs = 800; // 停止间隔800ms
    Heat_Init();
}

static void Heat_Init(void)
{
    Heat.heat_max = 260.0f;                  // Q0 热量上限
    Heat.heat_current = 0.0f;                // Q1 当前离线热量
    Heat.heat_lock = Heat.heat_max + 100.0f; // Q2 = Q0 + 100 (17mm弹丸) 锁定阈值
    g_fsm.fsm_mode = FSM_MODE_SILENT;
}
void Shoot_Control(void)
{
    dt = DWT_GetDeltaT(&Shoot_DWT_Count);
    t += dt;
    Shoot_Set_Mode();
    Shoot_Get_CtrlValue();
    Shoot_Set_Control();
    Send_Shoot_Output();
    Heat_Detection();

    isOnRange = ShootAllowRangeDetect(AimAssist.YawRange, AimAssist.PitchRange);

    LastKeyCode = remote_control.key_code;
}

static void Shoot_Set_Mode(void)
{
#ifdef USE_BULLETCOUNT
    static uint32_t BulletCountLast = 0;
#endif
    static uint32_t OneShotCount = 0;
    static uint16_t LastKeyCode = 0;
    static uint16_t LastPressLeft = 0;
    static uint8_t rcLeftSwitchValueLast = 0;
    static uint32_t KeyR_Tick = 0, KeyX_Tick = 0;
    static uint32_t shoot_stamp, debug_stamp;

    if ((remote_control.switch_left == Switch_Down && rcLeftSwitchValueLast != Switch_Down))
    {
        if (Shoot.FricSpeed >= FRIC_MOTOR_MIN_RPM)
            Shoot.isFricOn = 0;
        else
            Shoot.isFricOn = 1;
    }

    // TriggerMotor mode set
    if (Shoot.FricSpeed >= FRIC_MOTOR_MIN_RPM)
    {
        if (Gimbal.Mode == NORMAL_MODE || Gimbal.Mode == BATTLE_MODE)
        {
            HAL_GPIO_WritePin(LAZER_GPIO_Port, LAZER_Pin, GPIO_PIN_SET);
        }

        if (remote_control.switch_left == Switch_Up)
        {
            if (rcLeftSwitchValueLast != Switch_Up)
            {
                shoot_stamp = INS_GetTimeline();
            }
            if ((INS_GetTimeline() - shoot_stamp > 100) && Gimbal.Mode != BATTLE_MODE)
                Shoot.ShootMode = KeepShooting;
            // Shoot.ShootMode = PeriodicShoot;
        }
        else
        {
            Shoot.ShootMode = NoShooting;
        }
    }
    else
    {
        HAL_GPIO_WritePin(LAZER_GPIO_Port, LAZER_Pin, GPIO_PIN_RESET);
        Shoot.ShootMode = NoShooting;
    }

    if (remote_control.switch_left == Switch_Up)
    {
        if (rcLeftSwitchValueLast != Switch_Up && Gimbal.Mode == BATTLE_MODE)
        {
            Debug_Mode = !Debug_Mode;
        }
    }

#ifdef USE_BULLETCOUNT
    if (USER_GetTick() - ModeSwitchTick > 500 && Shoot.ShootMode == DebugShoot)
        Shoot.ShootMode = NoShooting;

    if (BulletCount - BulletCountLast > 0)
        Shoot.BulletToShoot -= BulletCount - BulletCountLast;
    BulletCountLast = BulletCount;
    if (Shoot.BulletToShoot < 0)
        Shoot.BulletToShoot = 0;
#endif

    switch (Shoot.ShootMode)
    {
    case NoShooting:
        Shoot.SpeedInBulletsPerSec = 0;
        Shoot.NumsOfOneShot = 0;
        Shoot.ShootDelayInMs = 0;
        break;

    case KeepShooting:
        Shoot.SpeedInBulletsPerSec = 20; // 8  弹频改这个
        Shoot.NumsOfOneShot = UINT32_MAX;
        Shoot.ShootDelayInMs = 0;
        break;

    case PeriodicShoot:
        Shoot.SpeedInBulletsPerSec = 8;
        Shoot.NumsOfOneShot = UINT32_MAX;
        Shoot.ShootDelayInMs = 0;
        break;

    default:
        Shoot.SpeedInBulletsPerSec = 0;
        Shoot.NumsOfOneShot = 0;
        Shoot.ShootDelayInMs = 0;
        break;
    }

    rcLeftSwitchValueLast = remote_control.switch_left;
    LastPressLeft = remote_control.mouse.press_left;
    LastKeyCode = remote_control.key_code;
}

static void Shoot_Get_CtrlValue(void)
{
    static float heatGain = 2.5;
    static float LastBulletSpeed = 0;
    static float LowSpeedCount = 0;
    static float Shoot_Freq;
    static uint8_t heatornot;
    static float current_heat, warning_heat, max_heat;

    Shoot.BulletSpeedLimit = 22.0f;
    if (Shoot.BulletSpeedLimit > 26.0f)
        Shoot.BulletSpeedLimit /= 5.0f;

    if ((Gimbal.Mode == BATTLE_MODE) || (Shoot.SpeedInBulletsPerSec > 5))
    {
        Shoot.isFricOn = Debug_Mode ? 0 : 1;
        if (AimAssist.Status != TgtLost)
        {
            if (ShootEvaluation.AbleToShoot && AimAssist.Status != TgtLost)
            {
                Shoot_Freq = (ShootEvaluation.Shoot_Freq);
                if (Shoot_Freq <= 0)
                {
                    Shoot_Freq = 0;
                }
                Shoot.NumsOfOneShot = UINT32_MAX;
                Shoot.ShootDelayInMs = 0;
            }
            else
                Shoot_Freq = 0;
        }
        else if (Shoot.ShootMode == KeepShooting || Shoot.ShootMode == PeriodicShoot)
        {
            Shoot_Freq = Shoot.SpeedInBulletsPerSec; // 没自瞄用默认的弹频数据
        }
        else
            Shoot_Freq = 0;
    }

    if (!is_TOE_Error(JUDGE_TOE) && fabsf(LastBulletSpeed - shoot_data.initial_speed) > 0.009f && shoot_data.initial_speed > 8)
    {
        Shoot.BulletSpeedCompensation += float_constrain((23.0 - shoot_data.initial_speed) * 30, -50, 50);
        Shoot.BulletSpeedCompensation = float_constrain(Shoot.BulletSpeedCompensation, -600, 600);
    }
    LastBulletSpeed = shoot_data.initial_speed;

    if (!is_TOE_Error(JUDGE_TOE))
    {
        Shoot.ResidualHeat = 260 - power_heat_data.shooter_17mm_barrel_heat;

        // 热量控制
        Shoot.FreqRatio = Heat_Judgment();
        // Shoot.FreqRatio = 1.0f; // 关闭热量控制，使用默认射频

        if (AimAssist.Status != TgtLost)
            Shoot_Freq = float_constrain(Shoot_Freq, 0, ShootEvaluation.Shoot_Freq * Shoot.FreqRatio);
        else
            Shoot_Freq = float_constrain(Shoot_Freq, 0, Shoot.SpeedInBulletsPerSec * Shoot.FreqRatio);
    }
    else
    {
        // 裁判系统离线，使用默认射频(关闭热量控制)
        Shoot.FreqRatio = 1.0f;
    }

    Shoot.TriggerSpeed = ShootAndDelay(&Shoot.TriggerMotor, Shoot_Freq, Shoot.NumsOfOneShot, Shoot.ShootDelayInMs);

    if (Shoot.ShootMode == PeriodicShoot) // 周期打弹模式根据时间窗口控制发射(调试用)
    {
        static uint32_t periodic_stamp = 0;
        static uint8_t in_shoot_phase = 0;
        uint32_t elapsed = USER_GetTimeline() - periodic_stamp;
        if (in_shoot_phase && elapsed >= Shoot.PeriodicShootOnMs)
        {
            in_shoot_phase = 0;
            periodic_stamp = USER_GetTimeline();
        }
        else if (!in_shoot_phase && elapsed >= Shoot.PeriodicShootOffMs)
        {
            in_shoot_phase = 1;
            periodic_stamp = USER_GetTimeline();
        }
        if (!in_shoot_phase)
            Shoot.TriggerSpeed = 0;
    }

    Shoot.RefFricSpeed = 5800; // 改弹速改这个，始终为正值

    if (Shoot.TriggerSpeed > 10 && Shoot.FricSpeed < fabsf(Shoot.RefFricSpeed) + Shoot.BulletSpeedCompensation)
    {
        Shoot.FricMotor[0].PID_Velocity.Ki = 0;
        Shoot.FricMotor[1].PID_Velocity.Ki = 0;
    }
    else
    {
        Shoot.FricMotor[0].PID_Velocity.Ki = Shoot.InitialKi;
        Shoot.FricMotor[1].PID_Velocity.Ki = Shoot.InitialKi;
    }
}

static void Shoot_Set_Control(void)
{
    static float last_fric_speed;
    static float lastTriggerSpeedTemp = 0;
    static float last_tempSpeed = 0;

    if (Shoot.FricSpeed < (fabsf(Shoot.RefFricSpeed) + Shoot.BulletSpeedCompensation) * 0.9f)
        Shoot.TriggerSpeed = 0;

    Shoot.TriggerSpeed = float_constrain(Shoot.TriggerSpeed, 0, 25.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60);
    Shoot.TriggerSpeedTemp = -Shoot.TriggerSpeed;

    Shoot.TriggerMotor.PID_Velocity.Iout = 0;
    Shoot.FricMotor[0].PID_Velocity.Iout = 0;
    Shoot.FricMotor[1].PID_Velocity.Iout = 0;

    // 期望速度从非零切换到零时清除积分，防止 Iout 残留抵消制动力导致多打弹
    if (Shoot.TriggerSpeedTemp == 0.0f && lastTriggerSpeedTemp != 0.0f)
    {
        Shoot.TriggerMotor.PID_Velocity.Iout = 0;
        Shoot.TriggerMotor.PID_Velocity.ITerm = 0;
        Shoot.TriggerMotor.PID_Velocity.Last_ITerm = 0;
    }
    lastTriggerSpeedTemp = Shoot.TriggerSpeedTemp;

    if (isBlocked) // 电机堵转
    {
        float unblock_speed = (Shoot.TriggerMotor.Direction == NEGATIVE) ? 1000.0f : -1000.0f;
        Motor_Speed_Calculate(&Shoot.TriggerMotor, Shoot.TriggerMotor.Velocity_RPM, unblock_speed);
    }
    else
    {
        Motor_Speed_Calculate(&Shoot.TriggerMotor, Shoot.TriggerMotor.Velocity_RPM, Shoot.TriggerSpeedTemp);
    }

    Shoot.FricSpeed = (fabsf(Shoot.FricMotor[0].Velocity_RPM) + fabsf(Shoot.FricMotor[1].Velocity_RPM)) / 2.0f;
    Shoot.FricAccel = (Shoot.FricSpeed - last_fric_speed) / (0.01f + dt) + Shoot.FricAccel * 0.01f / (0.01f + dt);

    if (Shoot.isFricOn == 1)
    {
        tempSpeed = float_constrain(Shoot.RefFricSpeed + Shoot.BulletSpeedCompensation * Shoot.isFricOn, 0, FRIC_MOTOR_MAX_RPM);
    }
    else
    {
        tempSpeed = 0;
    }

    TD_Calculate(&Shoot.FricTD, tempSpeed);
    if (Shoot.FricTD.x < 0.0f)
    {
        Shoot.FricTD.x = 0.0f;
        Shoot.FricTD.dx = 0.0f;
        Shoot.FricTD.last_dx = 0.0f;
        Shoot.FricTD.ddx = 0.0f;
        Shoot.FricTD.last_ddx = 0.0f;
    }

    // 改摩擦轮方向改这个
    Motor_Speed_Calculate(&Shoot.FricMotor[0], Shoot.FricMotor[0].Velocity_RPM, -Shoot.FricTD.x);
    Motor_Speed_Calculate(&Shoot.FricMotor[1], Shoot.FricMotor[1].Velocity_RPM, Shoot.FricTD.x);

    last_fric_speed = Shoot.FricSpeed;
}

static void Send_Shoot_Output(void)
{
    static uint32_t triggerMotorBlockCount = 0;
    if (GlobalDebugMode != SHOOT_DEBUG)
    {
        if (Shoot.isLidOpen)
            TIM_Set_PWM(&htim8, TIM_CHANNEL_1, 1900);
        else
            TIM_Set_PWM(&htim8, TIM_CHANNEL_1, 700);
    }

    if (is_TOE_Error(RC_TOE))
    {
        Send_Motor_Current_1_4(&hcan2, 0, 0, 0, 0);
    }
    else
    {
        if (Shoot.TriggerMotor.PID_Velocity.ERRORHandler.ERRORType == Motor_Blocked || isBlocked)
        {
            if (!isBlocked)
            {
                isBlocked = 1;
                Shoot.TriggerMotor.PID_Velocity.Iout = 0;
                Shoot.TriggerMotor.PID_Velocity.ITerm = 0;
                Shoot.TriggerMotor.PID_Velocity.Last_ITerm = 0;
            }
            triggerMotorBlockCount++;
            if (triggerMotorBlockCount > 300)
            {
                isBlocked = 0;
                triggerMotorBlockCount = 0;
                Shoot.TriggerMotor.PID_Velocity.ERRORHandler.ERRORType = PID_ERROR_NONE;
                Shoot.TriggerMotor.PID_Velocity.ERRORHandler.ERRORCount = 0;
            }
        }
        else
        {
            triggerMotorBlockCount = 0;
        }
        if (Gimbal.Mode == BATTLE_MODE)
        {
            if (isOnRange == 1)
                Send_Motor_Current_1_4(&hcan2, 0, Shoot.TriggerMotor.Output,
                                       Shoot.FricMotor[0].Output, Shoot.FricMotor[1].Output);
            else
                Send_Motor_Current_1_4(&hcan2, 0, 0,
                                       Shoot.FricMotor[0].Output, Shoot.FricMotor[1].Output);
        }
        else
        {
            Send_Motor_Current_1_4(&hcan2, 0, Shoot.TriggerMotor.Output,
                                   Shoot.FricMotor[0].Output, Shoot.FricMotor[1].Output);
        }
    }
}

// 离线热量检测上下文（文件作用域，由 Heat_Detection 子函数共享）
static HeatDetect_t s_heat_detect = {
    .is_trigger_angle_initialized = 0,
    .last_trigger_total_angle = 0,
    .feed_credit_angle_cnt = HEAT_INITIAL_FEED_CREDIT * TRIGGER_COUNTS_PER_BULLET,
    .state_time_s = 0.0f,
};
static float s_cooling_time_s = 0.0f;

/**
 * @brief 供弹角度累计与额度计算
 * @note  total_angle 已由电机反馈层完成跨圈累计。CAN 未稳定时仅重建角度基准，
 *        避免把初始化跳变误识别为供弹。堵转反转产生负增量，抵消未确认的正向送弹角度。
 * @param dt_s 调用周期 (秒)
 */
static void Heat_Update_FeedCredit(float dt_s)
{
    (void)dt_s; // 仅依赖 total_angle 增量，不依赖时间积分

    if (is_TOE_Error(TRIGGER_MOTOR1_TOE) || Shoot.TriggerMotor.msg_cnt <= 51)
    {
        s_heat_detect.is_trigger_angle_initialized = 0;
        return;
    }

    if (!s_heat_detect.is_trigger_angle_initialized)
    {
        s_heat_detect.last_trigger_total_angle = Shoot.TriggerMotor.total_angle;
        s_heat_detect.is_trigger_angle_initialized = 1;
        return;
    }

    int32_t trigger_delta_cnt = Shoot.TriggerMotor.total_angle
                                - s_heat_detect.last_trigger_total_angle;
    float forward_delta_cnt = (float)trigger_delta_cnt * TRIGGER_FEED_SIGN;
    s_heat_detect.last_trigger_total_angle = Shoot.TriggerMotor.total_angle;

    // 正常送弹增加额度；堵转反转产生负增量，抵消尚未确认的送弹角度
    if (Shoot.TriggerSpeed > 0.0f || isBlocked)
    {
        s_heat_detect.feed_credit_angle_cnt += forward_delta_cnt;
        if (s_heat_detect.feed_credit_angle_cnt < 0.0f)
        {
            s_heat_detect.feed_credit_angle_cnt = 0.0f;
        }
    }

    // 空仓额度上限钳位，防止空转后装弹时额度瞬间消费
    s_heat_detect.feed_credit_angle_cnt = fminf(
        s_heat_detect.feed_credit_angle_cnt,
        HEAT_MAX_FEED_CREDIT_BULLETS * TRIGGER_COUNTS_PER_BULLET);
}

/**
 * @brief 离线发弹检测状态机
 * @note  拨弹电机角度提供供弹额度，双摩擦轮电流实时确认发弹。
 *        任一摩擦轮出现弹丸负载电流并持续确认时间后，消费一个供弹额度；
 *        确认后进入恢复态，避免持续高电流被重复计数。
 * @param dt_s 调用周期 (秒)
 */
static void Heat_Detect_Fire(float dt_s)
{
    // 摩擦轮未就绪（未启动或转速不足）→ 回到静默态，避免启动电流误触发
    float fric_current_raw = fmaxf(fabsf(Shoot.FricMotor[0].Real_Current),
                                   fabsf(Shoot.FricMotor[1].Real_Current));
    float fric_speed_ready = (fabsf(Shoot.RefFricSpeed) + Shoot.BulletSpeedCompensation)
                             * HEAT_FRIC_READY_RATIO;

    if (!Shoot.isFricOn || Shoot.FricSpeed < fric_speed_ready)
    {
        g_fsm.fsm_mode = FSM_MODE_SILENT;
        s_heat_detect.state_time_s = 0.0f;
    }

    switch (g_fsm.fsm_mode)
    {
    case FSM_MODE_SILENT:
        if (Shoot.isFricOn && Shoot.FricSpeed >= fric_speed_ready)
        {
            // 速度就绪后延迟，等待启动电流回落
            s_heat_detect.state_time_s += dt_s;
            if (s_heat_detect.state_time_s >= HEAT_FRIC_SETTLE_TIME_S)
            {
                g_fsm.fsm_mode = FSM_MODE_READY;
                s_heat_detect.state_time_s = 0.0f;
            }
        }
        else
        {
            s_heat_detect.state_time_s = 0.0f;
        }
        break;

    case FSM_MODE_READY:
        s_heat_detect.state_time_s = 0.0f;
        if (g_pending_bullet > 0 &&
            Shoot.TriggerSpeed > 0.0f &&
            !isBlocked &&
            fric_current_raw > HEAT_CURRENT_ON)
        {
            g_fsm.fsm_mode = FSM_MODE_SUSPECT;
        }
        break;

    case FSM_MODE_SUSPECT:
        if (fric_current_raw > HEAT_CURRENT_ON)
        {
            s_heat_detect.state_time_s += dt_s;
            if (s_heat_detect.state_time_s >= HEAT_CONFIRM_TIME)
            {
                g_fsm.fsm_mode = FSM_MODE_CONFIRM;
                s_heat_detect.state_time_s = 0.0f;
            }
        }
        else if (fric_current_raw < HEAT_CURRENT_OFF)
        {
            g_fsm.fsm_mode = FSM_MODE_READY;
            s_heat_detect.state_time_s = 0.0f;
        }
        break;

    case FSM_MODE_CONFIRM:
        if (s_heat_detect.feed_credit_angle_cnt >= TRIGGER_COUNTS_PER_BULLET)
        {
            s_heat_detect.feed_credit_angle_cnt -= TRIGGER_COUNTS_PER_BULLET;
            g_pending_bullet = (uint32_t)(s_heat_detect.feed_credit_angle_cnt
                                          / TRIGGER_COUNTS_PER_BULLET);
            g_total_bullet++;
            Heat.heat_current += HEAT_PER_BULLET;
        }
        g_fsm.fsm_mode = FSM_MODE_RECOVERY;
        s_heat_detect.state_time_s = 0.0f;
        break;

    case FSM_MODE_RECOVERY:
        if (fric_current_raw < HEAT_CURRENT_OFF)
        {
            s_heat_detect.state_time_s += dt_s;
            if (s_heat_detect.state_time_s >= HEAT_RECOVERY_TIME)
            {
                g_fsm.fsm_mode = FSM_MODE_READY;
                s_heat_detect.state_time_s = 0.0f;
            }
        }
        else
        {
            s_heat_detect.state_time_s = 0.0f;
        }
        break;

    default:
        g_fsm.fsm_mode = FSM_MODE_SILENT;
        s_heat_detect.state_time_s = 0.0f;
        break;
    }
}

/**
 * @brief 离线热量冷却计算
 * @note  10 Hz 更新频率（每 HEAT_COOLING_INTERVAL_S 秒冷却一次）。
 *        冷却速率 = 场地增益冷却速率 × 姿态倍率。
 *        - Attack_Posture： 3 倍冷却
 *        - Defense_Posture：1/3 倍冷却
 *        - Mobile_Posture：1/3 倍冷却
 * @param dt_s 调用周期 (秒)
 */
static void Heat_Update_Cooling(float dt_s)
{
    Get_Cool_Rate();

    s_cooling_time_s += dt_s;

    if (s_cooling_time_s < HEAT_COOLING_INTERVAL_S)
    {
        return;
    }

    float cooling_per_second_heat = Cool_rate.total_rate;
    float posture_multiplier = 1.0f;

    switch (sentry_info.SentryPresentPose)
    {
    case Attack_Posture:
        posture_multiplier = ATTACK_COOLING_MULTIPLIER;
        break;
    case Defense_Posture:
        posture_multiplier = DEFENSE_COOLING_MULTIPLIER;
        break;
    case Mobile_Posture:
        posture_multiplier = MOBILE_COOLING_MULTIPLIER;
        break;
    default:
        posture_multiplier = 1.0f;
        break;
    }

    cooling_per_second_heat *= posture_multiplier;

    // 每周期冷却值 = 每秒冷却值 / 更新频率 (10 Hz)
    float cooling_per_cycle_heat = cooling_per_second_heat
                                   / (1.0f / HEAT_COOLING_INTERVAL_S);

    if (Heat.heat_current >= cooling_per_cycle_heat)
    {
        Heat.heat_current -= cooling_per_cycle_heat;
    }
    else
    {
        Heat.heat_current = 0.0f;
    }

    s_cooling_time_s -= HEAT_COOLING_INTERVAL_S; // 保留余数，提高精度
}

/**
 * @brief 离线热量融合检测与冷却计算（入口）
 * @note  拨弹电机角度提供供弹额度，摩擦轮电流实时确认发弹。
 *        RC 离线时跳过全部检测，状态机保持静默。
 */
static void Heat_Detection(void)
{
    // RC离线时重置状态机，不进行检测
    if (is_TOE_Error(RC_TOE))
    {
        g_fsm.fsm_mode = FSM_MODE_SILENT;
        s_heat_detect.state_time_s = 0.0f;
        return;
    }

    // 更新热量上限（来自裁判系统）
    if (robot_state.shooter_barrel_heat_limit > 0)
    {
        Heat.heat_max = robot_state.shooter_barrel_heat_limit;
        Heat.heat_lock = Heat.heat_max + 100.0f;
    }

    float dt_s = dt;

    Heat_Update_FeedCredit(dt_s);

    // 同步供弹额度到全局变量（仅用于调试观测）
    g_pending_bullet = (uint32_t)(s_heat_detect.feed_credit_angle_cnt
                                  / TRIGGER_COUNTS_PER_BULLET);

    Heat_Detect_Fire(dt_s);
    Heat_Update_Cooling(dt_s);
}

/**
 * @brief 获取场地交互增益冷却速率
 * @note 哨兵机器人固定：热量上限260，基础冷却30/秒
 *       根据RFID触发点、堡垒状态、能量机关等计算场地增益冷却速率
 */
static void Get_Cool_Rate(void)
{
    // 哨兵机器人基础冷却速率固定为 30点/秒
    Cool_rate.basic_rate = 30.0f;

    // 地形穿越增益(公路隧道) -> 双倍冷却
    if (rfid_status.TerrainCrossingDownHighwayBuff == 1 &&
        rfid_status.TerrainCrossingUpHighwayBuff == 1)
    {
        Cool_rate.tunnel_gain = Cool_rate.basic_rate * 2.0f;
    }
    else
    {
        Cool_rate.tunnel_gain = 0;
    }

    // 堡垒增益
    uint8_t fort_occupied = (event_data.event_data >> 10) & 0x01;

    if (fort_occupied == 1)
    {
        // 获取己方基地血量
        uint16_t base_hp = game_robot_HP.ally_base_HP;

        // 堡垒增益 = 基础冷却 + 基地血量衰减补偿
        // 基地血量越低，增益越高（最大5000 HP）
        // 范围：30点/秒（满血）~ 155点/秒（基地残血）
        Cool_rate.fort_gain = Cool_rate.basic_rate + (5000.0f - base_hp) / 40.0f;
    }
    else
    {
        Cool_rate.fort_gain = 0;
    }

    // 能量机关增益
    uint8_t power_rune_status = (event_data.event_data >> 7) & 0x03;

    if (power_rune_status > 0)
    {
        // TODO: 根据不同的大能量机关状态设计不同的增益，目前简单处理为双倍冷却
        Cool_rate.power_rune_gain = Cool_rate.basic_rate * 2.0f;
    }
    else
    {
        Cool_rate.power_rune_gain = 0;
    }

    // 取所有增益中的最大值作为最终冷却速率
    float max_coolrate = Cool_rate.basic_rate;

    if (Cool_rate.tunnel_gain > max_coolrate)
        max_coolrate = Cool_rate.tunnel_gain;

    if (Cool_rate.fort_gain > max_coolrate)
        max_coolrate = Cool_rate.fort_gain;

    if (Cool_rate.power_rune_gain > max_coolrate)
        max_coolrate = Cool_rate.power_rune_gain;

    Cool_rate.total_rate = max_coolrate;
}

static float Heat_Judgment(void)
{
    static float safety_margin = 0.0f;      // 距上限保留的安全余量
    static float stop_threshold = 40.0f;    // 低于此热量强制停止
    static float ramp_threshold = 30.0f;    // 低于此热量开始缓降
    static float resume_threshold = 180.0f; // 冷却后热量恢复至此才重新射击

    switch (sentry_info.SentryPresentPose)
    {
    case Attack_Posture:
        stop_threshold = 40.0f;
        ramp_threshold = 80.0f;
        resume_threshold = 100.0f;
        break;
    case Defense_Posture:
        stop_threshold = 60.0f;
        ramp_threshold = 80.0f;
        resume_threshold = 120.0f;
        break;
    case Mobile_Posture:
        stop_threshold = 60.0f;
        ramp_threshold = 80.0f;
        resume_threshold = 120.0f;
        break;
    default:
        stop_threshold = 40.0f;
        ramp_threshold = 0.0f;
        resume_threshold = 100.0f;
        break;
    }

    static uint8_t coolingFlag = 0; // 0=射击中，1=冷却中

    float current_heat = power_heat_data.shooter_17mm_barrel_heat;
    float max_heat = robot_state.shooter_barrel_heat_limit;

    float heat_budget = (max_heat - safety_margin) - current_heat;

    if (coolingFlag == 0)
    {
        if (heat_budget < stop_threshold)
        {
            coolingFlag = 1;
            return 0.0f;
        }
        else if (heat_budget < ramp_threshold)
        {
            return (heat_budget - stop_threshold) / (ramp_threshold - stop_threshold);
        }
        else
        {
            return 1.0f;
        }
    }
    else
    {
        if (heat_budget >= resume_threshold)
            coolingFlag = 0;
        return 0.0f;
    }
}

/**
 * @brief          根据期望射频计算拨弹电机速度，可实现以任意间隔任意射频发射任意数量子弹
 * @param[1]       电机结构体指针
 * @param[2]       射频，单位：发/s
 * @param[3]       单次射击子弹数
 * @param[4]       两次射击间隔时间
 * @retval         拨弹电机期望速度 单位：RPM
 */
float ShootAndDelay(Motor_t *trigger, float speedInNumsPerSec, uint32_t numsOfOneShot, uint32_t delayTimeInMs)
{
    static uint32_t ticksInMs = 0, lastNumsOfOneShot = 0, lastDelayTimeInMs = 0, count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0 || lastNumsOfOneShot != numsOfOneShot || lastDelayTimeInMs != delayTimeInMs)
    {
        ticksInMs = USER_GetTimeline() + delayTimeInMs + 1;
        lastAngle = abs(trigger->total_angle);
    }
    if (abs(trigger->total_angle) - lastAngle > 8191 * TRIGGER_MOTOR_REDUCTION_RATIO / BULLETS_PER_ROUND * numsOfOneShot)
    {
        lastAngle = abs(trigger->total_angle);
        speed = 0;
        ticksInMs = USER_GetTimeline();
    }

    if (USER_GetTimeline() - ticksInMs > delayTimeInMs)
        speed = speedInNumsPerSec / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60;

    count++;
    lastNumsOfOneShot = numsOfOneShot;
    lastDelayTimeInMs = delayTimeInMs;
    return speed;
}

/**
 * @brief 根据期望转动角度计算拨弹电机速度，可实现以任意间隔任意转动角度发射子弹
 *
 * @param trigger 拨弹盘电机
 * @param speedRPM 期望转动速度
 * @param angleInDegree 期望转动角度，正值为顺时针转动，负值为逆时针转动
 * @return float
 */
float RotateAngleInDegree(Motor_t *trigger, float speedRPM, float angleInDegree)
{
    static uint32_t count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0)
        lastAngle = trigger->total_angle;
    count++;
    if (angleInDegree > 0)
    {
        speed = speedRPM;
        if (trigger->total_angle - lastAngle > angleInDegree / 360 * 8191 * TRIGGER_MOTOR_REDUCTION_RATIO)
        {
            count = 0;
            speed = 0;
        }
    }
    if (angleInDegree < 0)
    {
        speed = -speedRPM;
        if (trigger->total_angle - lastAngle < angleInDegree / 360 * 8191 * TRIGGER_MOTOR_REDUCTION_RATIO)
        {
            count = 0;
            speed = 0;
        }
    }
    return speed;
}

/**
 * @brief 云台角度对齐检测
 * @note  使用20个样本多数投票机制，对云台是否在允许射击范围内进行去抖检测
 *        - 当检测结果发生变化时，启动19次采样窗口（加上初次检测共20次）
 *        - 在采样窗口内统计满足条件和不满足条件的次数
 *        - 采样结束后根据多数投票结果（>10次）确定最终状态
 *        - 避免因云台抖动或瞬时偏差导致的误触发
 *
 * @param YawRange   偏航角允许偏差范围
 * @param PitchRange 俯仰角允许偏差范围
 * @return int       1->云台在允许射击范围内; 0->云台不在允许射击范围内
 */
int ShootAllowRangeDetect(float YawRange, float PitchRange)
{
    static uint8_t res = 0;
    static uint8_t last_expression = 0;
    static uint8_t wait_on_event = 0;
    static uint8_t _cnt[2] = {0};
    uint8_t expression = fabsf(Gimbal.YawAngle - Gimbal.YawRefAngle) <= YawRange * RADIAN_COEF && fabsf(Gimbal.PitchAngle - Gimbal.PitchRefAngle) <= PitchRange * RADIAN_COEF;
    if (last_expression != expression && wait_on_event == 0)
    {
        wait_on_event = 19;
    }
    else if (wait_on_event > 0)
    {
        if (expression)
        {
            _cnt[1]++;
        }
        else
        {
            _cnt[0]++;
        }
        wait_on_event--;
        if (_cnt[0] > _cnt[1])
        {
            res = 0;
        }
        else
        {
            res = 1;
        }
    }
    else
    {
        _cnt[0] = 0;
        _cnt[1] = 0;
    }
    last_expression = expression;
    return res;
}
