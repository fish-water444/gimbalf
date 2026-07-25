/**
 ******************************************************************************
 * @file    shoot_task.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2021/10/28
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _SHOOT_TASK_H
#define _SHOOT_TASK_H
#include "main.h"
#include "motor.h"
#include "controller.h"
#include "stdbool.h"


#define SHOOT_TASK_PERIOD 1

#include "bsp_CAN.h"
#include "system_identification.h"
#define FRIC_RM3508_LEFT_ID 0x203
#define FRIC_RM3508_RIGHT_ID 0x204
#define FRIC_MOTOR_MIN_RPM 2000
#define FRIC_MOTOR_MAX_RPM 6800

// Trigger motor configuration
#define TRIGGER_MOTOR_ID 0x202
#define TRIGGER_MOTOR_REDUCTION_RATIO 36
#define BULLETS_PER_ROUND 8
#define TRIGGER_ENCODER_COUNTS_PER_ROUND 8192.0f
#define TRIGGER_COUNTS_PER_BULLET \
  (TRIGGER_ENCODER_COUNTS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO / BULLETS_PER_ROUND)
#define TRIGGER_FEED_SIGN (-1.0f) // 正常送弹时total_angle递减

// 离线发弹融合检测参数
#define HEAT_CURRENT_ON 1228.0f                  // 摩擦轮电流检测阈值 (mA)，超过此值判定弹丸通过
#define HEAT_CURRENT_OFF 1000.0f                 // 摩擦轮电流恢复阈值 (mA)，低于此值判定弹丸已离开
#define HEAT_CONFIRM_TIME 0.015f                 // 电流持续超过 ON 阈值的确认时间 (s)
#define HEAT_RECOVERY_TIME 0.005f                // 电流持续低于 OFF 阈值的恢复时间 (s)
#define HEAT_INITIAL_FEED_CREDIT 1.0f            // 初始供弹额度 (发)，补偿预装弹链及未知机械相位
#define HEAT_MAX_FEED_CREDIT_BULLETS 5.0f        // 空仓额度上限，防止空转堆积
#define HEAT_FRIC_READY_RATIO 0.9f               // 摩擦轮就绪速度比例，低于此不检测
#define HEAT_FRIC_SETTLE_TIME_S 0.1f             // 摩擦轮速度就绪后等待电流回落的时间 (s)
#define HEAT_COOLING_INTERVAL_S 0.1f             // 冷却更新间隔 (秒)
#define HEAT_PER_BULLET 10.0f                    // 每发弹丸热量增量
#define ATTACK_COOLING_MULTIPLIER 3.0f           // 进攻姿态冷却倍率
#define DEFENSE_COOLING_MULTIPLIER (1.0f / 3.0f) // 防守姿态冷却倍率
#define MOBILE_COOLING_MULTIPLIER (1.0f / 3.0f)  // 移动姿态冷却倍率

#define SWITCH_MOTOR_ID 0x201
#define FIXED_OUTPUT 1500 // 1500
#define SLEW_OUTPUT 3000
#define MAX_ROTARY_OUTPUT 10000 // 10000
#define TIMES_COUNT 110
#define Heat_Warning 180 // 超过此上限导致视野变糊，可视度降低，发射机构锁定，实时热量降低为零，恢复正常（17mm)
#define Heat_Limit 260   // 超过此上限锁枪管，发射机构整局锁定，不再解锁(17mm)

typedef struct
{
  float heat_max;     // 热量上限 (Q0)
  float heat_current; // 当前热量 (Q1)
  float heat_lock;    // 锁定阈值 (17mm: Q0+100, 42mm: Q0+200)
} Heat_t;

typedef struct
{
  float basic_rate;      // 基础冷却速率
  float fort_gain;       // 堡垒增益
  float tunnel_gain;     // 地形穿越增益
  float power_rune_gain; // 能量机关增益
  float total_rate;      // 最终冷却速率（取以上最大值）
} Coolrate_t;

enum
{
  NoShooting = 0,
  KeepShooting,
  PeriodicShoot, // 周期打弹（调试）：按弹频打OnMs，停OffMs，循环
};

enum _chassis_status
{
  go_to_des,        // 0 老六位 英雄下前哨站左侧，堵住狗洞
  go_to_healing,    // 1 补血点
  go_to_base,       // 2 RC（挡住自家开花后基地下部前面装甲板）
  go_to_fort,       // 3 登上堡垒
  stay_in_place,    // 4 呆在原地
  reach_des,        // 5
  go_back_start,    // 6
  reach_start,      // 7
  go_to_area,       // 8 RMUC中指自家飞坡点前方 RMUL中指敌方障碍两侧
  reach_area,       // 9
  go_to_area2,      // 10 RMUC中指自家公路区堵住对方狗洞处 RMUL中指己方障碍两侧
  go_to_area3,      // 11 己方上至中央台地狗洞前方
  listen_to_aerial, // 12 根据云台手指令
  go_to_centrl,     // 13 2025RMUL中央点位 RMUC敌方大资源岛后方
  go_to_outpost,    // 14 敌方前哨站下
  go_to_road,       // 15 敌方u型道上
};

enum RobotPosture
{
  No_Posture,
  Attack_Posture = 1,  // 进攻姿态
  Defense_Posture = 2, // 防守姿态
  Mobile_Posture = 3,  // 移动姿态
};

enum _barrel_mode
{
  Unfixed = 0,  // δ�̶�
  BarrelNormal, // ����״̬
  Overheated,   // ����
  Stallad,      // ��ת
};

enum _shooterid
{
  Shooter1 = 1,
  Shooter2,
};

typedef enum
{
  FSM_MODE_SILENT = 1,   // 静默状态
  FSM_MODE_READY = 2,    // 准备检测状态
  FSM_MODE_SUSPECT = 3,  // 嫌疑状态
  FSM_MODE_CONFIRM = 4,  // 确认状态
  FSM_MODE_RECOVERY = 5, // 恢复状态
} FsmMode_e;

typedef struct _rotary
{
  int16_t fixed_output; // �̶�����������С
  int16_t slew_output;  // �л�����������С
  float Recovery_time;  // ˫ǹ�л���϶
  float Resume_timing;  // �л�ʱ���ʱ

  Motor_t RotaryMotor;
  float RotaryMotorVelocity;
  float Rotary_Output;              // ���ڵ�ת��������������С
  int8_t Rotary_Output_Coefficient; // ��+-1 ǹ�ܴ�������״̬ 0�����쳣ֹͣ״̬��
  float Rotary_Output_MAX;          // ת�����������
  uint8_t Rotary_reset;             // �������

  uint8_t Shooter_id; // ʹ�õ�ǹ��ID
  uint8_t Last_Shooterid;
  uint8_t Shooter_state; // ǹ�ܵ�����״̬
  uint16_t Rotarytimes;  // ˫ǹ�л�����
  uint8_t Barrel_Debug;  // �����л�ǹ��

  float Stalladornot;           // ��ת�ж�
  float Stallad_Recovery_time;  // ��ת�Իָ�����ʱ��
  int32_t Motor_limit_angle[2]; // ǹ��������ƽǶ�
  uint8_t Rotary_Stallad_times; // Debug�� ���Զ�ת����
  uint8_t Rotary_debug;

  float current_lpf;
  float current_last;
  float current_judge;      // �˲�
  float motor_last_current; // δ�˲�
  uint8_t Mode_gun;
} Rotary_t;

// 离线热量检测上下文
typedef struct
{
  uint8_t is_trigger_angle_initialized;
  int32_t last_trigger_total_angle;
  float feed_credit_angle_cnt; // 供弹角度额度 (编码器单位)
  float state_time_s;          // 状态机计时 (秒)
} HeatDetect_t;

typedef struct
{
  FsmMode_e fsm_mode;
} FSM_t;

typedef struct _shoot
{
  uint8_t ShootMode;

  int32_t BulletToShoot;

  uint8_t isLidOpen;
  uint8_t isFricOn;

  float TriggerSpeed;
  float TriggerSpeedTemp;
  float TriggerSpeed2;
  Motor_t TriggerMotor;
  float FreqRatio;
  float BulletSpeedLimit;
  float BulletSpeedCompensation;

  float ResidualHeat;

  // Motor_t SwitchMotor;

  float SpeedInBulletsPerSec;
  uint32_t NumsOfOneShot;
  uint32_t ShootDelayInMs;
  uint32_t PeriodicShootOnMs;  // 周期打弹：每次允许打弹的窗口时长(ms)
  uint32_t PeriodicShootOffMs; // 周期打弹：停止打弹的间隔时长(ms)

  float InitialKi;

  float BulletVelocity;
  float RefBulletVelocity;
  Rotary_t Rotary;
  float RefFricSpeed;
  float FricSpeed;
  float FricAccel;
  TD_t FricTD;
  Motor_t FricMotor[2];
  FirstOrderSI_t FricMotorSI[2];
} Shoot_t;

extern uint8_t set_posture_mode, debug_posture;
extern Shoot_t Shoot;
extern Heat_t Heat;
extern FSM_t g_fsm;
extern Coolrate_t Cool_rate;
extern uint32_t g_total_bullet;
extern uint32_t g_pending_bullet;
void Shoot_Init(void);
void Shoot_Control(void);
float ShootAndDelay(Motor_t *trigger, float speedInNumsPerSec, uint32_t NumsOfOneShot, uint32_t delayTimeInMs);
float RotateAngleInDegree(Motor_t *trigger, float speedRPM, float angleInDegree);
int ShootAllowRangeDetect(float YawRange, float PitchRange);

#endif
