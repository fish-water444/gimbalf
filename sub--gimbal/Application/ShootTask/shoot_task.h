#ifndef _SHOOT_TASK_H
#define _SHOOT_TASK_H
#include "main.h"
#include "motor.h"
#include "controller.h"
#include "stdbool.h"


enum Shootmode
{
  NoShooting = 0,
  KeepShooting,
  WaitcShooting, 
};

typedef struct {
    uint8_t Mode;              
    uint8_t isFricOn;         // 摩擦轮是否启动

    float RefFricSpeed;       // 期望摩擦轮转速 (RPM)
    float RefShootFreq;       // 期望射频 (发/秒)

    Motor_t FricMotor[2];     // 左、右摩擦轮
    Motor_t TriggerMotor;     // 拨弹盘电机

    float TriggerSpeed;       // 拨弹盘当前期望转速
    float FricSpeed;
} Shoot_t;

extern Shoot_t Shoot;

void Shoot_Init(void);
void Shoot_Control(void);
float ShootFreqToRPM(float shootfreq);

#endif
