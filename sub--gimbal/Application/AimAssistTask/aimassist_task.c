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
#include "judgement_info.h"
#include "bsp_usart_idle.h"
#include "user_lib.h"
#include "bsp_CAN.h"
#include "protocol.h"
#include "remote_control.h"

AimAssist_t AimAssist = {0};
Nav_t Nav = {0};
ShootEvaluation_t ShootEvaluation = {0};

static uint8_t Nav_Rx_Buf[NAV_RX_BUF_NUM];
uint8_t NavTxFrame[NAV_TX_TO_CHASSIS_BUF_NUM];

void AimAssist_Init(UART_HandleTypeDef *huart)
{
    AimAssist.AA_USART = huart;
    Nav.AA_USART = huart;
    AimAssist.Mode = AUTO_AIM;
    AimAssist.miniPC_Online = 0;
    USART_IDLE_Init(huart, Nav_Rx_Buf, NAV_RX_BUF_NUM);
}

void AimAssist_Task(void)
{
    static uint8_t lock_active = 0;
    if (AimAssist.Status == TgtLost)
    {
        lock_active = 0;
    }
    else if (!lock_active && AimAssist.fire_control_off_distance < 5.0f)
    {
        lock_active = 1;
    }
    else if (lock_active && AimAssist.fire_control_off_distance > 5.5f)
    {
        lock_active = 0;
    }
    AimAssist.allow_to_lock = lock_active;
}

void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    if (huart == JudgeUSART)
    {
        unpack_fifo_handle(judgement_receive.buf);
        Detect_Hook(JUDGE_TOE);
    }
    if (huart == remote_control.RC_USART)
        Callback_RC_Handle(&remote_control, sbus_rx_buf);
}
