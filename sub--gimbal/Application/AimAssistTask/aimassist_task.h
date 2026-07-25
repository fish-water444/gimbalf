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

#define ENABLE_CALIBRATION 0

#define AUTOAIM_TASK_PERIOD 1
#define CALIBRATION_RX_BUF_NUM 28
#define AimAssist_RX_BUF_NUM 18
#define AimAssist_TX_BUF_NUM 4

#define NAV_RX_BUF_NUM 15
#define NAV_TX_TO_CHASSIS_BUF_NUM 16

#define X 0
#define Y 1
#define Z 2

enum miniPC_Mode
{
    SUSPEND = 0,
    AUTO_AIM,
    HIT_RUNE_MIN,
    HIT_RUNE_MAX,
    NTP_CALIBRATE
};

enum AimAssistStatus
{
    TgtLost = 0,
    TgtSwitch,
    TgtConjecture,
};

typedef struct
{
    uint8_t Status;
    uint8_t Mode;
    uint8_t miniPC_Online;

    float YawPosition;
    float PitchPosition;
    float YawRange;
    float PitchRange;

    float fire_control_off_distance;
    uint8_t allow_to_lock;

    float Lpf;

    float yaw_motor_current;
    float yaw_dot;
    float pitch_dot;

    uint8_t isLock;
    uint8_t isHero;
    uint8_t isFindTarget;
    float omni_yaw_offset;

    float position_x;
    float position_y;

    UART_HandleTypeDef *AA_USART;
} AimAssist_t;

typedef struct __packed
{
    uint8_t Nav_SOF;
    uint8_t Mode;
    uint8_t guyong;
    uint8_t Nav_EOF;
    uint8_t FdbFlag;
    UART_HandleTypeDef *AA_USART;
    uint8_t miniPC_Online;
} Nav_t;

typedef struct
{
    uint8_t AbleToShoot;
    uint8_t Shoot_Freq;
} ShootEvaluation_t;

extern AimAssist_t AimAssist;
extern ShootEvaluation_t ShootEvaluation;
extern Nav_t Nav;
extern uint8_t NavTxFrame[NAV_TX_TO_CHASSIS_BUF_NUM];

void AimAssist_Init(UART_HandleTypeDef *huart);
void AimAssist_Task(void);

#endif
