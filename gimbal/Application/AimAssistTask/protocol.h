/*
 * @Author: Vergissmeinnicht zjz0@outlook.com
 * @Date: 2024-00-19 22:36:05
 * @LastEditors: Vergissmeinnicht zjz0@outlook.com
 * @FilePath: Application/protocol.h
 * @Description: Serial Protocol.
 *
 * Copyright (c) 2023 by Zheng Junzhe, All Rights Reserved.
 */

#ifndef SENTRY_GIMBAL_2023_NEW_PROTOCOL_H
#define SENTRY_GIMBAL_2023_NEW_PROTOCOL_H

#define NONE_TEAM 0x00
#define RED_TEAM 0x1F
#define BLUE_TEAM 0xF1

// Serial Communication Protocol
typedef struct
{
  float INSTimeCost;
  float q_w;
  float q_x;
  float q_y;
  float q_z;
} __attribute__((__packed__)) UploadCMD_1;

typedef struct
{
  float INSTimeCost;
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  float accel[3];
  float gyro[3];
} __attribute__((__packed__)) UploadCMD_2;

typedef struct
{
  uint8_t autoaim_status;
  uint8_t my_team;
  uint8_t InvincibleTarget;      // 敌方兵种是否无敌 共8位 第0位为裁判系统读取检测 第1~5位分别对应对方前五号车 第6位为敌方哨兵 第七位奇校验位
  uint8_t AllowToAttackOutpost;  // 是否允许攻击哨站
  uint8_t AllowToAttackEngineer; // 是否允许攻击工程车
} __attribute__((__packed__)) UploadCMD_4;

typedef struct
{
  uint8_t TgtStatus;
  uint8_t TgtLabel;
  uint8_t allow_shoot;
  uint8_t shoot_frequency;
  float Yaw;
  float Pitch;
  uint16_t autoaim_debug;
  uint8_t health_check;
} __attribute__((__packed__)) DownloadCMD_1;

// CMD 71
typedef struct // 0.5~5Hz
{
  int16_t Team;           // 队伍颜色, RED = 0x1F, BLUE = 0xF1
  int16_t DecisionStatus; // 当前决策的模式

  // UWB 位置
  float UWBPosX;
  float UWBPosY;

  // 云台手标记位置
  float AerialPosX;
  float AerialPosY;

  float MoveRatio;
} __attribute__((__packed__)) UPLOAD_PACK_71;

// CMD 72
typedef struct // 10~50Hz
{
  uint16_t current_HP;
  uint16_t stage_remain_time;
  uint16_t remain_bullet;
  uint8_t posture;
  uint8_t current_pos;

  float cruise_begin;
  float cruise_end;
  float cruise_speed;

  uint8_t aim_status;
  uint8_t game_progress;
} __attribute__((__packed__)) UPLOAD_PACK_72;

// CMD 73
typedef struct
{
  int16_t ch1;
  int16_t ch2;
  uint8_t switch_right;
} __attribute__((__packed__)) UPLOAD_PACK_73;

// CMD 71
typedef struct // 50~200Hz
{
  /*
   * 通过这种方式实现:
   * pack.nowFaceX = face.x() * 1000;
   * pack.nowFaceY = face.y() * 1000;
   * face 是 单位向量,
   * 是 当前大yaw朝向方向, 在base_init系下的向量
   */
  int16_t nowFaceX;
  int16_t nowFaceY;

  /*
   * 在base_init系下, 期望的速度方向
   * 单位 毫米/s
   * 注意! 这个并不一定是轨迹输出的期望速度,
   * 而是上位机控制器输出的结果, 当前没有任何实际物理意义
   */
  int16_t targetVelocityX;
  int16_t targetVelocityY;
} __attribute__((packed)) DOWNLOAD_PACK_71;

// CMD 72
typedef struct // 5~20Hz, 通过自瞄转发
{
  uint8_t inPosition; // 是否在位置上, 1为在位置上, 0为不在位置上
} __attribute__((packed)) DOWNLOAD_PACK_72;

// CMD 72
typedef struct // 1~5Hz
{
  int16_t DoSpin;  // 开陀螺, 1为开, 0为关
  int16_t OpenCap; // 开电容, 1为开, 0为关
} __attribute__((packed)) DOWNLOAD_PACK_73;

// Serial Communication Protocol

#endif // SENTRY_GIMBAL_2024_PROTOCOL_H
