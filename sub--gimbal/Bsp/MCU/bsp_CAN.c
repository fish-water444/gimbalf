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
#include "shoot_task.h"
#include "user_lib.h"
#include "judgement_info.h"

#define CAN_TX_WAIT_MAX 10000U

CAN_ErrorInfo_t CAN1_ErrorInfo = {0};
CAN_ErrorInfo_t CAN2_ErrorInfo = {0};

uint8_t FOLLOW_Data_Buf[24] = {0};
uint8_t Robot_Info_Buf[16] = {0};
uint8_t Decision_Info_Buf[16] = {0};
uint8_t FOLLOW_Update = 0;
uint16_t enemy_outpost_HP = 1500;
uint8_t JudgeRxDataValid = 0;
float theta;
int16_t PlanDot[2][2];
int16_t PosDot[2];
uint8_t InPosCount;
uint8_t InPosFlag;
uint8_t DataFromAerial[16];
uint8_t RMUL_NAV[8];
uint8_t ALLRange[8];
uint8_t Gimbal_Info[8];
uint8_t RMUC_NAV[8];

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
 * @Brief       CAN���߽��ջص����� ���ڽ��յ������
 * @Param	    CAN_HandleTypeDef* _hcan
 * @Retval	    None
 * @Date        2019/11/4
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t RC_Data_Buf[16];
    static uint8_t SLAM_Data_Buf[4];

    if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        CAN_ErrorInfo_t *error_info = CAN_Get_ErrorInfo(_hcan);
        if (error_info != NULL)
            error_info->last_error = HAL_CAN_GetError(_hcan);
        return;
    }

    if (_hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case 0x221:
            FOLLOW_Data_Buf[0] = rx_data[0]; // CF_SOF
            FOLLOW_Data_Buf[1] = rx_data[1]; // POSX
            FOLLOW_Data_Buf[2] = rx_data[2]; // POSX
            FOLLOW_Data_Buf[3] = rx_data[3]; // POSY
            FOLLOW_Data_Buf[4] = rx_data[4]; // POSY
            FOLLOW_Data_Buf[5] = rx_data[5]; // YAW
            FOLLOW_Data_Buf[6] = rx_data[6]; // YAW
            FOLLOW_Data_Buf[7] = rx_data[7]; // planX 1
            break;

        case 0x222:
            FOLLOW_Data_Buf[8] = rx_data[0];  // planX 1
            FOLLOW_Data_Buf[9] = rx_data[1];  // planY 1
            FOLLOW_Data_Buf[10] = rx_data[2]; // planY 1
            FOLLOW_Data_Buf[11] = rx_data[3]; // planX 2
            FOLLOW_Data_Buf[12] = rx_data[4]; // planX 2
            FOLLOW_Data_Buf[13] = rx_data[5]; // planY 2
            FOLLOW_Data_Buf[14] = rx_data[6]; // planY 2
            FOLLOW_Data_Buf[15] = rx_data[7]; // Mode
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

            sentry_info.SentryPresentPose = Decision_Info_Buf[13];
            Gimbal.allow_to_attack_outpost = Decision_Info_Buf[14];
            break;

        case 0x666:
            Gimbal_Info[2] = rx_data[2];
            Gimbal.isrollover = Gimbal_Info[2];
            break;
        case 0x503:
            RMUL_NAV[0] = rx_data[0];
            Gimbal.DecisionPlace = RMUL_NAV[0];
            break;
        }
    }

    if (_hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        // �����������ư巢����ң��������
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
            break;
        case YAW_MOTOR_ID:
            if (Gimbal.YawMotor.msg_cnt++ <= 50)
                get_moto_offset(&Gimbal.YawMotor, rx_data);
            else
                get_moto_info(&Gimbal.YawMotor, rx_data);
            Detect_Hook(GIMBAL_YAW_MOTOR_TOE);
            break;
        case FRIC_RM3508_LEFT_ID:
            if (Shoot.FricMotor[0].msg_cnt++ <= 50)
                get_moto_offset(&Shoot.FricMotor[0], rx_data);
            else
                get_moto_info(&Shoot.FricMotor[0], rx_data);
            break;

        case FRIC_RM3508_RIGHT_ID:
            if (Shoot.FricMotor[1].msg_cnt++ <= 50)
                get_moto_offset(&Shoot.FricMotor[1], rx_data);
            else
                get_moto_info(&Shoot.FricMotor[1], rx_data);
            break;
        case PITCH_MOTOR_ID:
            if (Gimbal.PitchMotor.msg_cnt++ <= 50)
                get_moto_offset(&Gimbal.PitchMotor, rx_data);
            else
                get_moto_info(&Gimbal.PitchMotor, rx_data);
            Detect_Hook(GIMBAL_PITCH_MOTOR_TOE);
            break;
        case TRIGGER_MOTOR_ID:
            if (Shoot.TriggerMotor.msg_cnt++ <= 50)
                get_moto_offset(&Shoot.TriggerMotor, rx_data);
            else
                get_moto_info(&Shoot.TriggerMotor, rx_data);
            Detect_Hook(TRIGGER_MOTOR1_TOE);
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
        }
    }
}

void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
    if (rc_data == NULL)
        return;

    CAN_Send_Data(_hcan, CAN_RC_DATA_Frame_0, rc_data, 8);
    CAN_Send_Data(_hcan, CAN_RC_DATA_Frame_1, rc_data + 8, 8);
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

void Send_Contral_Value_Info(CAN_HandleTypeDef *_hcan,uint8_t isHero,uint8_t isFindTarget,uint8_t isLock,float omni_yaw_offset,float position_x,float position_y)
{
    uint8_t data[8] = {0};
    uint8_t Temp = 0u;
    uint16_t TempYawTargetOffset;
    uint16_t Tempposition_x;
    uint16_t Tempposition_y;

    TempYawTargetOffset = float_to_uint16(omni_yaw_offset, -180.0f, 180.0f, 16);
    Tempposition_x = (uint16_t)(1000.0f * position_x);
    Tempposition_y = (uint16_t)(1000.0f * position_y);
    Temp |= isFindTarget;
    Temp |= (isLock << 1);
    Temp |= (isHero << 2);

    data[0] = (uint8_t)(TempYawTargetOffset >> 8);
    data[1] = (uint8_t)TempYawTargetOffset;
    data[2] = (uint8_t)(Tempposition_x >> 8);
    data[3] = (uint8_t)Tempposition_x;
    data[4] = (uint8_t)(Tempposition_y >> 8);
    data[5] = (uint8_t)Tempposition_y;
    data[6] = Temp;
    CAN_Send_Data(_hcan, 0x667, data, 8);
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
    if (FOLLOW_Data_Buf != NULL)
        CAN_Send_Data(_hcan, 0x251, FOLLOW_Data_Buf + 8, 8);
}

void Send_Aerial_Data(CAN_HandleTypeDef *_hcan)
{
    uint8_t data[8] = {0};

    CAN_Send_Data(_hcan, CAN_AERIAL_DATA_1, DataFromAerial, 8);
    data[0] = DataFromAerial[8];
    data[7] = DataFromAerial[15];
    CAN_Send_Data(_hcan, CAN_AERIAL_DATA_2, data, 8);
}

void SendAimRefAngle(CAN_HandleTypeDef *_hcan, float ControlTorque, uint8_t Slope)
{
    uint8_t data[8] = {0};
    uint16_t YawControlTorque;

    YawControlTorque = (uint16_t)ControlTorque;
    data[0] = AimAssist.Status;
    data[1] = AimAssist.allow_to_lock;
    data[3] = (uint8_t)(YawControlTorque >> 8);
    data[4] = (uint8_t)YawControlTorque;
    data[5] = Slope;
    data[6] = set_posture_mode;
    data[7] = debug_posture;
    CAN_Send_Data(_hcan, Ref_Angle_ID, data, 8);
}
