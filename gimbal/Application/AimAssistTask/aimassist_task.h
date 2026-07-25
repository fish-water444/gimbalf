/**
 ******************************************************************************
 * @file	aimassist_task.c
 * @author  Wang Hongxi
 * @version v2.5.3
 * @date    2022/4/2
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef _AIMASSIST_H
#define _AIMASSIST_H

#include "kalman_filter.h"
#include "remote_control.h"

#define AUTOAIM_TASK_PERIOD 1
#define CALIBRATION_RX_BUF_NUM 28

#define AimAssist_TX_BUF_NUM 4

#define NAV_RX_BUF_NUM 15
#define NAV_TX_TO_CHASSIS_BUF_NUM 16
#define NAV_TX_BUF_NUM 255

#define PITCH_MAX_DEG 15
#define YAW_MAX_DEG 20

#define X 0
#define Y 1
#define Z 2

/**
 * @Brief: TX2����ս��֡�ṹ��
 */
typedef struct _controlFrame
{
    uint8_t CF_SOF;
    uint8_t flg;
    int16_t x; // mm
    uint16_t y;
    int16_t z;
    uint32_t time_stamp_ms;
    short reserve1;   // �������ȱ�
    uint8_t reserve2; // װ�װ�����
    uint8_t CF_EOF;
} ControlFrame;

typedef struct _controlFrameFull
{
    uint8_t CF_SOF;         //
    uint8_t flg;            // װ�װ�ID
    int16_t px;             // Ŀ�����ϵ���� x
    uint16_t py;            // Ŀ�����ϵ���� y
    int16_t pz;             // Ŀ�����ϵ���� z
    int16_t rx;             // Ŀ�����ϵ������ x
    int16_t ry;             // Ŀ�����ϵ������ y
    int16_t rz;             // Ŀ�����ϵ������ z
    uint16_t time_stamp_ms; // ms*10
    uint8_t type;           // ��/Сװ��
    uint8_t CF_EOF;         // 15
} ControlFrameFull;

/**
 * @Brief: ս���ش�����֡�ṹ��
 */
typedef struct _feedBackFrame
{
    uint8_t FDBF_SOF;
    uint8_t myteam; //  EE DD
    short pitch;    //  10k*rad
    short yaw;
    uint16_t bullet_speed;  // m/s
    uint16_t time_stamp_ms; // ms
    short outpost_alive;
    short status;
    short hit2;
    uint8_t mode;
    uint8_t FDBF_EOF;
} FeedBackFrame;

enum ObjectType
{
    UNKNOWN_ARMOR = 0,
    SMALL_ARMOR = 1,
    BIG_ARMOR = 2,
    RUNE_WING = 3,
    RUNE_CENTRE = 4
};

enum miniPC_Mode
{
    SUSPEND = 0,
    AUTO_AIM,
    HIT_RUNE_MIN,
    HIT_RUNE_MAX,
    NTP_CALIBRATE
};

enum ColorChannels
{
    INVALID_COLOR = 0,
    BLUE = 1,
    RED = 2
};

enum NTP_MSG
{
    PC2STM32_T1 = 1,
    STM2PC_T3,
    PC2STM32_T4,
    NTP_SUCCESS,
    NTP_FAIL,
    NTP_START
};

enum TargetStatus
{
    TargetLost = 0,
    TargetValid,
};

enum AimAssistStatus
{
    TgtLost = 0,
    TgtSwitch,
    TgtConjecture,
};

typedef struct
{
    float T1;
    float T2;
    float T3;
    float T4;
    float deltaT;
    float TransmitDelay;
} NTP_t;

typedef struct
{
    uint8_t Status;
    uint8_t Mode;
    uint8_t LastMode;
    uint8_t miniPC_Online;
    uint8_t PerspectiveEnable;
    ControlFrame CtrlFrame;
    ControlFrameFull CtrlFrameFull;
    FeedBackFrame FdbFrame;
    uint32_t FrameTimeStamp;
    uint8_t dataValid;
    uint8_t FdbFlag;

    float TargetBodyFrame[3];
    float TargetEarthFrame[3];
    uint8_t TargetID;

    float YawPosition;
    float PitchPosition;
    float Distance;

    float TimeConsuming;
    float Frame_dt;
    float FrameDelayToINS;
    float DelayTime;
    uint32_t UpdatePeriodCount;

    float Lpf;

    NTP_t NTP;

    uint8_t LastStatus;
    uint8_t isSpinning;
    uint8_t SpinningTgtValid;
    float TargetLostTime;
    float TrackingTimeStamp;
    uint32_t LostCount;
    uint32_t TrackingCount;

    uint32_t TgtSwitchPeriod_ms;
    uint32_t TgtSwitchTick_ms;

    uint8_t ArmorType;

    mat Ccb, CcbT;
    float Ccb_data[9], CcbT_data[9];
    mat Cbn, CbnT;
    float Cbn_data[9], CbnT_data[9];
    mat Rc;
    float Rc_data[9];
    mat tempMat;
    float tempMat_data[9];
    mat H, HT, M1, M2;
    float H_data[12], HT_data[12], M1_data[12], M2_data[4];
    mat ResErr, ResErrT;
    float ResErr_data[2], ResErrT_data[2];
    KalmanFilter_t TgtMotionEst;

    float BulletVelocity;

    float HorizontalDistance;
    float HorizontalDistance_dot;
    float HorizontalDistance_ddot;
    float HorizontalDistanceLPF;
    float HorizontalDistance_dotLPF;
    float TgtHeight;
    float TgtHeight_dot;
    float TgtHeight_ddot;

    float Position[3];
    float Velocity[3];
    float xOyVelocity;
    float Accel[3];
    float AccLPF;

    float ForwardTime;

    float PreTargetEarthFrame[3];

    float YawVelocity;
    float PitchVelocity;
    float YawAccel;
    float PitchAccel;

    float fire_control_distance;
    uint8_t allow_to_lock;

} AimAssist_t;

typedef struct __packed
{
    uint8_t Nav_SOF;
    uint8_t Mode;
    uint8_t guyong;
    uint8_t Nav_EOF;
    uint8_t FdbFlag;
    uint8_t miniPC_Online;
} Nav_t;

enum
{
    Clockwise,
    Counterclockwise
};

typedef struct
{
    uint8_t Direction;
    uint32_t deltaT;
    float ArmorDistance;
    float CrossProductZ;
    float Velocity;
    float VelocityX;
    float VelocityY;
} SpinningVelocityPriori_t;

enum
{
    NormalSpeedSpinning,
    HighSpeedSpinning
};


typedef struct
{
    float CamX;
    float CamY;
    float CamZ;
    float axis_offset;

    float CameraYaw;
    float CameraPitch;
    float CameraRoll;

    float BulletYaw;
    float BulletPitch;
    float BulletYaw30;
    float BulletPitch30;
    float BulletYawRune;
    float BulletPitchRune;
} OffsetCorrection_t;

typedef struct
{
    uint8_t AbleToShoot;
    uint8_t ShootFreqGain;
    uint8_t UseAccel;
    uint8_t SpinningKeepShooting;
    uint32_t TargetValidTime;
    float MaxForwardTime;
    float SafeForwardTime;
    float MaxFreqGain;
    float PreTargetEarthFrame[3];
    float YawAngle;
    float YawError;
    float BulletShootDelay;
} ShootEvaluation_t;

typedef struct
{
    float Position[3];
    float Velocity[3];
    uint32_t TimeStamp;
    uint8_t TgtStatus;
} TgtPosFrame_t;

#define Tgt_FRAME_LEN 500
typedef struct
{
    TgtPosFrame_t TgtFrame[Tgt_FRAME_LEN];
    uint16_t LatestNum;
} TgtPosBuf_t;

typedef struct __packed
{
    int16_t nowFaceX;
    int16_t nowFaceY;

    int16_t TargetVelocityX;
    int16_t TargetVelocityY;
} NavReceiveFrame_t;

extern AimAssist_t AimAssist;
extern ShootEvaluation_t ShootEvaluation;

extern TgtPosBuf_t TgtPosBuf;
extern Nav_t Nav;
extern uint8_t NavTxFrame[NAV_TX_TO_CHASSIS_BUF_NUM];

void AimAssist_Init(UART_HandleTypeDef *huart);
void AimAssist_Task(void);

#endif
