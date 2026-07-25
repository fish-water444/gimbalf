/**
 * @file    bsp_CAN.c
 * @author  Wang Hongxi
 * @author  Lu Yurui
 * @version V2.0
 * @date    2026/06/06
 * @brief   CAN communication initialization, receive parsing and protocol send
 */
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "main.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "motor.h"
#include "detect_task.h"
#include "aimassist_task.h"
#include "judgement_info.h"
#include "user_lib.h"
#include "judgement_info.h"

#define CAN_TX_WAIT_MAX 10000U

CAN_ErrorInfo_t CAN1_ErrorInfo = {0};
CAN_ErrorInfo_t CAN2_ErrorInfo = {0};

uint8_t FOLLOW_Data_Buf[24] = {0};
uint8_t Robot_Info_Buf[24] = {0};
uint8_t Decision_Info_Buf[24] = {0};
uint8_t DownLoad_Data[8];
uint8_t Aim_Data[24];
uint8_t NAV_Data[8];

uint8_t posture;
float theta, chassis_move_ratio;
int16_t PlanDot[2][2];
int16_t PosDot[2];
uint8_t InPosCount;
uint8_t InPosFlag;
uint8_t InAerialCmdFlag = 0;
uint16_t sentry_HP = 0, base_HP = 0;
uint8_t DataFromAerial[16];

static HAL_StatusTypeDef CAN_Config_Filter(CAN_HandleTypeDef *_hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = filter_bank;
    filter.SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(_hcan, &filter);
}

static CAN_ErrorInfo_t *CAN_Get_ErrorInfo(CAN_HandleTypeDef *_hcan)
{
    if (_hcan == &hcan1)
        return &CAN1_ErrorInfo;
    if (_hcan == &hcan2)
        return &CAN2_ErrorInfo;
    return NULL;
}

HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *_hcan, uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_ErrorInfo_t *error_info = CAN_Get_ErrorInfo(_hcan);
    uint32_t mailbox = 0;
    uint32_t wait_count = 0;
    HAL_StatusTypeDef status;

    if (_hcan == NULL || data == NULL || len > 8U)
        return HAL_ERROR;

    while (_hcan->State != HAL_CAN_STATE_READY &&
           _hcan->State != HAL_CAN_STATE_LISTENING)
    {
        if (++wait_count > CAN_TX_WAIT_MAX)
        {
            if (error_info != NULL)
                error_info->tx_timeout++;
            return HAL_TIMEOUT;
        }
    }

    wait_count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0U)
    {
        if (++wait_count > CAN_TX_WAIT_MAX)
        {
            if (error_info != NULL)
            {
                error_info->tx_timeout++;
                error_info->last_error = HAL_CAN_GetError(_hcan);
            }
            HAL_CAN_AbortTxRequest(_hcan,
                                   CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            return HAL_TIMEOUT;
        }
    }

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    status = HAL_CAN_AddTxMessage(_hcan, &header, (uint8_t *)data, &mailbox);

    if (status != HAL_OK && error_info != NULL)
    {
        error_info->tx_error++;
        error_info->last_error = HAL_CAN_GetError(_hcan);
    }
    return status;
}

void CAN_Device_Init(void)
{
    if (CAN_Config_Filter(&hcan1, 0) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK ||
        CAN_Config_Filter(&hcan2, 14) != HAL_OK ||
        HAL_CAN_Start(&hcan2) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
}

/**
 * @Func	    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief      CAN���߽��ջص����� ���ڽ��յ������
 * @Param	    CAN_HandleTypeDef* _hcan
 * @Retval	    None
 * @Date       2019/11/4
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t RC_Data_Buf[16];
    static uint8_t SLAM_Data_Buf[4];
    static float cruise_omni_offset = 0;
    static uint8_t cruise_flag_local = 0;
    static float cruise_begin_local = 0;
    static float cruise_end_local = 0;
    static float cruise_speed_local = 0;

    if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        CAN_ErrorInfo_t *error_info = CAN_Get_ErrorInfo(_hcan);
        if (error_info != NULL)
            error_info->last_error = HAL_CAN_GetError(_hcan);
        return;
    }

    if (_hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case SUB_YAW_MOTOR_ID:
            if (Gimbal.SubYawMotor.msg_cnt++ <= 50)
                get_moto_offset(&Gimbal.SubYawMotor, rx_data);
            else
                get_moto_info(&Gimbal.SubYawMotor, rx_data);
            break;
        }
    }
    if (_hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case CAN_RC_DATA_Frame_0:
            RC_Data_Buf[0] = rx_data[0];
            RC_Data_Buf[1] = rx_data[1];
            RC_Data_Buf[2] = rx_data[2];
            RC_Data_Buf[3] = rx_data[3];
            RC_Data_Buf[4] = rx_data[4];
            RC_Data_Buf[5] = rx_data[5];
            RC_Data_Buf[6] = rx_data[6];
            RC_Data_Buf[7] = rx_data[7];
            break;
        case CAN_RC_DATA_Frame_1:
            RC_Data_Buf[8] = rx_data[0];
            RC_Data_Buf[9] = rx_data[1];
            RC_Data_Buf[10] = rx_data[2];
            RC_Data_Buf[11] = rx_data[3];
            RC_Data_Buf[12] = rx_data[4];
            RC_Data_Buf[13] = rx_data[5];
            RC_Data_Buf[14] = rx_data[6];
            RC_Data_Buf[15] = rx_data[7];
            Callback_RC_Handle(&remote_control, RC_Data_Buf);
            break;

        case YAW_MOTOR_ID:
            Detect_Hook(GIMBAL_YAW_MOTOR_TOE);
            if (Gimbal.YawMotor.msg_cnt++ <= 50)
                get_RMD_offset(&Gimbal.YawMotor, rx_data);
            else
                get_RMD_info(&Gimbal.YawMotor, rx_data);
            break;
        case CAN_AERIAL_DATA_1:
            DataFromAerial[0] = rx_data[0];
            DataFromAerial[1] = rx_data[1];
            DataFromAerial[2] = rx_data[2];
            DataFromAerial[3] = rx_data[3];
            DataFromAerial[4] = rx_data[4];
            DataFromAerial[5] = rx_data[5];
            DataFromAerial[6] = rx_data[6];
            DataFromAerial[7] = rx_data[7];
            break;
        case CAN_AERIAL_DATA_2:
            DataFromAerial[8] = rx_data[0];
            DataFromAerial[15] = rx_data[7];
            break;
        case 0x503:
            NAV_Data[0] = rx_data[0]; // DesicionStatus
            NAV_Data[2] = rx_data[2]; // TargetX H8
            NAV_Data[3] = rx_data[3]; // TargetX L8
            NAV_Data[4] = rx_data[4]; // TargetY H8
            NAV_Data[5] = rx_data[5]; // TargetY L8
            NAV_Data[6] = rx_data[6]; // IsEnemySentryInvincible
            NAV_Data[7] = rx_data[7]; // PH2
            Gimbal.DecisionPlace = NAV_Data[0];
            map_command.target_position_x = ((float)((NAV_Data[2] << 8) | (NAV_Data[3]))) / 1000.0f;
            map_command.target_position_y = ((float)((NAV_Data[4] << 8) | (NAV_Data[5]))) / 1000.0f;
            break;
        case 0x667:
            DownLoad_Data[0] = rx_data[0];
            DownLoad_Data[1] = rx_data[1];
            DownLoad_Data[2] = rx_data[2];
            DownLoad_Data[3] = rx_data[3];
            DownLoad_Data[4] = rx_data[4];
            DownLoad_Data[5] = rx_data[5];
            DownLoad_Data[6] = rx_data[6];
            DownLoad_Data[7] = rx_data[7];

            cruise_omni_offset = uint16_to_float((DownLoad_Data[0] << 8) | (DownLoad_Data[1]), -PI, PI, 16) * 180 / PI;
            Gimbal.position_x = ((float)((DownLoad_Data[2] << 8) | (DownLoad_Data[3]))) / 1000.0f;
            Gimbal.position_y = ((float)((DownLoad_Data[4] << 8) | (DownLoad_Data[5]))) / 1000.0f;
            Gimbal.isHero = DownLoad_Data[6] >> 2;
            Gimbal.isLock = DownLoad_Data[6] >> 1;
            Gimbal.isTarget = DownLoad_Data[6];

            break;
        case 0x155:
            Aim_Data[0] = rx_data[0];
            Aim_Data[1] = rx_data[1];
            Aim_Data[2] = rx_data[2];
            Aim_Data[3] = rx_data[3];
            Aim_Data[4] = rx_data[4];
            Aim_Data[5] = rx_data[5];
            Aim_Data[6] = rx_data[6];
            Aim_Data[7] = rx_data[7];
            AimAssist.Status = Aim_Data[0];
            AimAssist.allow_to_lock = Aim_Data[1];
            Gimbal.Slope = Aim_Data[5];
            Gimbal.SubYawTorque = 0.022 * ((int16_t)((Aim_Data[3] << 8) | (Aim_Data[4])));

            break;
        case 0x133:
            Robot_Info_Buf[0] = rx_data[0];
            Robot_Info_Buf[1] = rx_data[1];
            Robot_Info_Buf[2] = rx_data[2];
            Robot_Info_Buf[3] = rx_data[3];
            Robot_Info_Buf[4] = rx_data[4];
            Robot_Info_Buf[5] = rx_data[5];
            Robot_Info_Buf[6] = rx_data[6];
            Robot_Info_Buf[7] = rx_data[7];
            Detect_Hook(JUDGE_TOE);
            break;
        case 0x134:
            Robot_Info_Buf[8] = rx_data[0];
            Robot_Info_Buf[9] = rx_data[1];
            Robot_Info_Buf[10] = rx_data[2];
            Robot_Info_Buf[11] = rx_data[3];
            Robot_Info_Buf[12] = rx_data[4];
            Robot_Info_Buf[13] = rx_data[5];
            Robot_Info_Buf[14] = rx_data[6];
            Robot_Info_Buf[15] = rx_data[7];

            robot_state.robot_id = Robot_Info_Buf[0];
            robot_state.shooter_barrel_heat_limit = (uint16_t)((Robot_Info_Buf[1] << 8) | Robot_Info_Buf[2]);
            power_heat_data.shooter_17mm_barrel_heat = (uint16_t)((Robot_Info_Buf[3] << 8) | Robot_Info_Buf[4]);
            shoot_data.initial_speed = (uint16_t)((Robot_Info_Buf[5] << 8) | Robot_Info_Buf[6]) / 10.0f;
            cruise_flag_local = Robot_Info_Buf[8];
            robot_pos.x = ((uint16_t)((Robot_Info_Buf[9] << 8) | Robot_Info_Buf[10]) / 100.0f);
            robot_pos.y = ((uint16_t)((Robot_Info_Buf[12] << 8) | Robot_Info_Buf[13]) / 100.0f);
            Gimbal.ReachFlag = Robot_Info_Buf[14];
            game_status.game_progress = Robot_Info_Buf[15];
            Detect_Hook(JUDGE_TOE);
            break;
        case 0x135:
            Decision_Info_Buf[0] = rx_data[0];
            Decision_Info_Buf[1] = rx_data[1];
            Decision_Info_Buf[2] = rx_data[2];
            Decision_Info_Buf[3] = rx_data[3];
            Decision_Info_Buf[4] = rx_data[4];
            Decision_Info_Buf[5] = rx_data[5];
            Decision_Info_Buf[6] = rx_data[6];
            Decision_Info_Buf[7] = rx_data[7];
            break;
        case 0x136:
            Decision_Info_Buf[8] = rx_data[0];
            Decision_Info_Buf[9] = rx_data[1];
            Decision_Info_Buf[10] = rx_data[2];
            Decision_Info_Buf[11] = rx_data[3];
            Decision_Info_Buf[12] = rx_data[4];
            Decision_Info_Buf[13] = rx_data[5];
            Decision_Info_Buf[14] = rx_data[6];
            Decision_Info_Buf[15] = rx_data[7];

            cruise_begin_local = uint16_to_float((uint16_t)((Decision_Info_Buf[0] << 8) | Decision_Info_Buf[1]), -180.0f, 180.0f, 16);
            cruise_end_local = uint16_to_float((uint16_t)((Decision_Info_Buf[2] << 8) | Decision_Info_Buf[3]), -180.0f, 180.0f, 16);
            chassis_move_ratio = uint16_to_float((uint16_t)((Decision_Info_Buf[4] << 8) | Decision_Info_Buf[5]), 0.0f, 1.0f, 16);
            cruise_speed_local = uint16_to_float((uint16_t)((Decision_Info_Buf[6] << 8) | Decision_Info_Buf[7]), 0.0f, 1.0f, 16);
            
            game_status.stage_remain_time = (uint16_t)((Decision_Info_Buf[8] << 8) | Decision_Info_Buf[9]);
            robot_state.current_HP = (uint16_t)((Decision_Info_Buf[10] << 8) | Decision_Info_Buf[11]);
            posture = Decision_Info_Buf[12];
            sentry_info.SentryPresentPose = Decision_Info_Buf[13];
            projectile_allowance.projectile_allowance_17mm = (uint16_t)((Decision_Info_Buf[14] << 8) | Decision_Info_Buf[15]);
            break;
        }
    }
}

void Send_Robot_Info(CAN_HandleTypeDef *_hcan, int8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed)
{
    uint8_t data[8] = {
        (uint8_t)ID,
        (uint8_t)(heatLimit >> 8), (uint8_t)heatLimit,
        (uint8_t)(heat >> 8), (uint8_t)heat,
        (uint8_t)(bulletSpeed >> 8), (uint8_t)bulletSpeed,
        0};

    CAN_Send_Data(_hcan, 0x430, data, 8);
}

void Send_Gimbal_Info(CAN_HandleTypeDef *_hcan, uint8_t aimassist_online, uint8_t Slope, uint8_t rollover)
{
    uint8_t data[8] = {aimassist_online, Slope, rollover, 0, 0, 0, 0, 0};

    CAN_Send_Data(_hcan, 0x666, data, 8);
}

void Send_Reset_Command(CAN_HandleTypeDef *_hcan)
{
    const uint8_t data[8] = {0};

    CAN_Send_Data(_hcan, CAN_SYSTEM_RESET_CMD, data, 8);
}

void Send_Follow_Data(CAN_HandleTypeDef *_hcan, uint8_t *FOLLOW_Data_Buf)
{
    if (FOLLOW_Data_Buf != NULL)
        CAN_Send_Data(_hcan, 0x250, FOLLOW_Data_Buf, 8);
}

void Send_Follow_Data_2(CAN_HandleTypeDef *_hcan, uint8_t *FOLLOW_Data_Buf)
{
    uint8_t data[8] = {AimAssist.Status, InPositionFlag, 0, 0, 0, 0, 0, 0};

    (void)FOLLOW_Data_Buf;
    CAN_Send_Data(_hcan, 0x251, data, 8);
}

void Send_Aerial_Data(CAN_HandleTypeDef *_hcan)
{
    uint8_t data[8] = {0};

    CAN_Send_Data(_hcan, CAN_AERIAL_DATA_1, DataFromAerial, 8);
    data[0] = DataFromAerial[8];
    data[7] = DataFromAerial[15];
    CAN_Send_Data(_hcan, CAN_AERIAL_DATA_2, data, 8);
}
