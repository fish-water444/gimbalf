#pragma once
#ifndef PROTOCOL_2025_H
#define PROTOCOL_2025_H

#define NONE_TEAM 0x00
#define RED_TEAM 0x1F
#define BLUE_TEAM 0xF1

// Serial Communication Protocol
/**
 * @author Zheng Junzhe
 * @version 3.0.0
 * @brief 基础上传协议，仅包含需要高FPS发送的信息
 * @warning 禁止更改
 */
typedef struct
{
  float INSTimeCost;         // from BMI088_Read to CDC_Send2miniPC_Data Time cost, in milliseconds
  float q_w;                 // Quaternion w
  float q_x;                 // Quaternion x
  float q_y;                 // Quaternion y
  float q_z;                 // Quaternion z
  float trigger_speed;       // 拨弹盘转速 rad/s
  float shoot_residual_heat; // 射击残余热量;
  uint8_t autoaim_mode;      // 自瞄模式 (normal: 0x00, autoaim: 0x01, small_rune: 0x02, big_rune: 0x03)
} __attribute__((__packed__)) UploadCMD_1;

/**
 * @author Zheng Junzhe
 * @version 3.0.0
 * @brief 标定上传协议，仅在标定时使用，不应上场
 * @warning 禁止更改
 */
typedef struct
{
  float INSTimeCost;         // from BMI088_Read to CDC_Send2miniPC_CI_Cali_Data Time cost, in milliseconds
  float q_w;                 // Quaternion w
  float q_x;                 // Quaternion x
  float q_y;                 // Quaternion y
  float q_z;                 // Quaternion z
  float trigger_speed;       // 拨弹盘转速 rad/s
  float shoot_residual_heat; // 射击残余热量;
  float accel[3];            // INS accel
  float gyro[3];             // INS gyro
  uint8_t autoaim_mode;      // 自瞄模式 (normal: 0x00, autoaim: 0x01, small_rune: 0x02, big_rune: 0x03)
} __attribute__((__packed__)) UploadCMD_2;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 裁判系统信息上传协议
 */
typedef struct
{
  uint8_t my_team_color; // 我方队伍颜色 (Red: 0x1F, Blue: 0xF1)
} __attribute__((__packed__)) UploadCMD_3;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 哨兵裁判系统信息上传协议
 */
typedef struct
{
  uint8_t InvincibleTarget;      // 无敌目标,第0位为裁判系统读取检测，1~5位对应对方前5号车，6位为哨兵
  uint8_t AllowToAttackOutpost;  // 允许攻击前哨站
  uint8_t AllowToAttackEngineer; // 允许攻击工程
} __attribute__((__packed__)) UploadCMD_4;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 发射信息上传协议
 */
typedef struct
{
  uint8_t bullet_id;         // 弹丸编号
  uint8_t is_shoot_success;  // 是否发射成功
  uint8_t is_reference_recv; // 裁判系统反馈正常
  float gimbal_pitch;        // 云台Pitch
  float bullet_speed;        // 弹速
  float shoot_delay;         // 发弹延迟
} __attribute__((__packed__)) UploadCMD_5;

/**
 * @author Zheng Junzhe
 * @version 2.0.0
 * @brief 小Yaw电机信息上传协议
 */
typedef struct
{
  float yaw_motor_angle;   // 小Yaw Yaw电机角度
  float pitch_motor_angle; // 小Yaw Pitch电机角度
} __attribute__((__packed__)) UploadCMD_6;

/**
 * @author Zheng Junzhe
 * @version 0.0.1
 * @brief 标定
 */
typedef struct 
{
  float yaw;
  float pitch;
  float accel[3];
  float gyro[3];
} __attribute__((__packed__)) UploadCMD_7;

/**
 * @author Zheng Junzhe
 * @version 1.0.0
 * @brief 电控脱离视觉调pid上传协议
 */
typedef struct
{
  uint8_t mode;
  float center_x;
  float center_y;
  float center_z;
  float pz;
  float r1;
  float r2;
  float dtheta;
  float point_start[2];
  float point_end[2];
  float move_speed_frequency;
} __attribute__((__packed__)) UploadCMD_10;

/**
 * @author Zheng Junzhe
 * @version 3.1.0
 * @brief 自瞄信息下载协议
 */
typedef struct
{
  uint8_t target_state;        // 0: 没有捕获目标, 1: 捕获目标, 使用CV模型, 2：捕获目标, 使用Spin模型
  uint8_t target_label;        // 锁定的目标
  uint8_t allow_shoot;         // 0: 停火， 1: 开火
  float shoot_frequency;       // 射击频率(目前电控不保证该参数被正确执行)
  float yaw;                   // rad
  float yaw_dot;               // rad/s
  float yaw_motor_current;     // A (p.s. 为IMU虚拟理想电机期望电流)
  float pitch;                 // rad
  float pitch_dot;             // rad/s
  float pitch_motor_current;   // A (p.s. 为IMU虚拟理想电机期望电流)
  float yaw_allow_range;       // 开火允许的Yaw范围 rad
  float pitch_allow_range;     // 开火允许的Pitch范围 rad
  uint16_t autoaim_debug_show; // 自瞄调试信息
  uint8_t health_check;        // 健康检测
} __attribute__((__packed__)) DownloadCMD_1;

/**
 * @author Zheng Junzhe
 * @version 2.1.0
 * @brief 哨兵大Yaw控制信息下载协议
 */
typedef struct
{
  uint8_t is_find_target;
  uint8_t is_hero;
  float big_yaw_target_offset;
  float fire_control_off_distance;
} __attribute__((__packed__)) DownloadCMD_2;

/**
 * @author Zheng Junzhe
 * @version 2.1.0
 * @brief 英雄位置(frame: 大Yaw车体中心)
 */
typedef struct
{
  uint8_t is_lock;
  float position_x;
  float position_y;
} __attribute__((__packed__)) DownloadCMD_3;

/**
 * @author Zheng Junzhe
 * @version 0.0.1
 * @brief 标定
 */
typedef struct 
{
  float yaw;
  float pitch;
}__attribute__((__packed__)) DownloadCMD_7;

typedef struct
{
  uint8_t Team;
  float UWBPosX;
  float UWBPosY;
  float AerialPosX;
  float AerialPosY;
  uint8_t DesicionStatus;
} __attribute__((__packed__)) USARTUploadCMD_1;

typedef struct
{
  int16_t nowFaceX;
  int16_t nowFaceY;

  int16_t TargetVelocityX;
  int16_t TargetVelocityY;
} __attribute__((__packed__)) USARTDownloadCMD_1;

#endif
