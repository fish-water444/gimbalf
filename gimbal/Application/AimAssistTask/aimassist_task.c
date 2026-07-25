#include "aimassist_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "QuaternionEKF.h"
#include "judgement_info.h"
#include "bsp_usart_idle.h"
#include "user_lib.h"
#include "bsp_CAN.h"
#include "protocol.h"
#include "gimbal_task.h"
#include "ins_task.h"
#include "detect_task.h"
#include "aimassist_task.h"
#include "QuaternionEKF.h"
#include "judgement_info.h"
#include "bsp_usart_idle.h"
#include "user_lib.h"
#include "bsp_CAN.h"
#include "protocol.h"

AimAssist_t AimAssist = {0};
Nav_t Nav = {0};
ShootEvaluation_t ShootEvaluation = {0};

TgtPosBuf_t TgtPosBuf = {0};
OffsetCorrection_t OffsetCorrection;
float NAV2CHASSISTheta;

/**************************** Aim Assist KF Data ******************************/
float TgtMotionEst_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1};
float TgtMotionEst_P[36];
float TgtMotionEst_Pinit[36] = {
    1000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 1000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 1000, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100000};
float TgtMotionEst_Sigma[3] = {10000, 10000, 1000};
float TgtMotionEst_Q[36] = {
    0.1, 0, 0, 0, 0, 0,
    0, 50, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 50, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 50};
float sigmaSqY = 5000;
float sigmaSqTheta = 1.25e-6;
float sigmaSqPhi = 3.5e-6;
float ChiSquare;
float xyPositonError;
float xhat_data_obsv[6];

/*************************** Aim Assist Data Buf ******************************/

ControlFrame CtrlFrameTemp;
ControlFrameFull CtrlFrameTempFull;

NavReceiveFrame_t NavReceiveFrame;
static uint8_t Nav_Rx_Buf[NAV_RX_BUF_NUM];
static uint8_t Nav_Tx_Buf[NAV_TX_BUF_NUM];
uint8_t NavTxFrame[NAV_TX_TO_CHASSIS_BUF_NUM];
/************************** Aim Assist Time Stamp *****************************/
uint32_t AimAssist_DWT_Count = 0;
static float dt = 0, t = 0;
uint32_t AimAssist_DWT_Cost = 0;
float AimAssistCostTime, AimAssisTempTimeStamp;

/******************************** Task Func ***********************************/
void AimAssist_Init(UART_HandleTypeDef *huart)
{
    AimAssist.Mode = AUTO_AIM;
    AimAssist.miniPC_Online = 0;

    USART_IDLE_Init(huart, Nav_Rx_Buf, NAV_RX_BUF_NUM);

    AimAssist.ForwardTime = 0.002f;

    AimAssist.BulletVelocity = 28.5f;

    AimAssist.AccLPF = 0.025;
    AimAssist.HorizontalDistanceLPF = 0.005;
    AimAssist.HorizontalDistance_dotLPF = 0.1;

    OffsetCorrection.CamX = 0;
    OffsetCorrection.CamY = 123;
    OffsetCorrection.CamZ = 93;
    OffsetCorrection.axis_offset = 40;
    OffsetCorrection.CameraYaw = 0;
    OffsetCorrection.CameraPitch = 0.22;
    OffsetCorrection.CameraRoll = 0.12;
    OffsetCorrection.BulletYaw = 0.85;
    OffsetCorrection.BulletPitch = 0.2;
    // ����ʱ��������
    AimAssist.FrameDelayToINS = 0.003f;

    Matrix_Init(&AimAssist.Ccb, 3, 3, (float *)AimAssist.Ccb_data);
    Matrix_Init(&AimAssist.CcbT, 3, 3, (float *)AimAssist.CcbT_data);
    Matrix_Init(&AimAssist.Cbn, 3, 3, (float *)AimAssist.Cbn_data);
    Matrix_Init(&AimAssist.CbnT, 3, 3, (float *)AimAssist.CbnT_data);
    Matrix_Init(&AimAssist.Rc, 3, 3, (float *)AimAssist.Rc_data);
    Matrix_Init(&AimAssist.tempMat, 3, 3, (float *)AimAssist.tempMat_data);
    Matrix_Init(&AimAssist.H, 2, 6, (float *)AimAssist.H_data);
    Matrix_Init(&AimAssist.HT, 6, 2, (float *)AimAssist.HT_data);
    Matrix_Init(&AimAssist.M1, 6, 2, (float *)AimAssist.M1_data);
    Matrix_Init(&AimAssist.M2, 2, 2, (float *)AimAssist.M2_data);
    Matrix_Init(&AimAssist.ResErr, 2, 1, (float *)AimAssist.ResErr_data);
    Matrix_Init(&AimAssist.ResErrT, 1, 2, (float *)AimAssist.ResErrT_data);
}

void AimAssist_Task(void)
{
    static uint16_t SendCount1;
    static uint16_t SendCount2;
    static uint16_t SendCount3;
    dt = DWT_GetDeltaT(&AimAssist_DWT_Count);
    t += dt;
    AimAssisTempTimeStamp = DWT_GetDeltaT(&AimAssist_DWT_Cost);

    if (SendCount1 >= 1000)
    {
        CDCSend2MiniPC_Nav_Data71(robot_pos.x, robot_pos.y, robot_pos.angle, robot_state.robot_id);
        SendCount1 = 0;
    }
    SendCount1++;

    if (SendCount2 >= 200)
    {
        CDCSend2MiniPC_Nav_Data72();
        SendCount2 = 0;
    }
    SendCount2++;

    if (SendCount3 >= 500)
    {
        CDCSend2MiniPc_Nav_Data73();
        SendCount3 = 0;
    }
    SendCount3++;

    NAV2CHASSISTheta = Gimbal.YawMotor.AngleInDegree - INS.Yaw;

    Send_Follow_Data(&hcan1, NavTxFrame);
    Send_Follow_Data_2(&hcan1, NavTxFrame);

    AimAssisTempTimeStamp = DWT_GetDeltaT(&AimAssist_DWT_Cost);
    AimAssistCostTime = AimAssisTempTimeStamp;
    AimAssist.LastMode = AimAssist.Mode;
    AimAssist.LastStatus = AimAssist.Status;
}

void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    if (huart == remote_control.RC_USART)
        Callback_RC_Handle(&remote_control, sbus_rx_buf);
    if (huart == JudgeUSART)
    {
        unpack_fifo_handle(judgement_receive.buf);
        Detect_Hook(JUDGE_TOE);
    }
}
