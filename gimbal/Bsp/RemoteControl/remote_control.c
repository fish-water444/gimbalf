/**
 ******************************************************************************
 * @file    remote_control.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2020/12/22
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "remote_control.h"
#include "bsp_CAN.h"
#include "detect_task.h"
#include "bsp_dwt.h"
#include "bsp_usart_idle.h"
#include "bsp_dwt.h"
#include "bsp_usart_idle.h"

RC_Type remote_control = {0};

uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

uint8_t RC_Data_Buffer[16] = {0};

uint32_t RC_DWT_Count = 0;
float RC_dt = 0;
uint8_t RC_Update = 0;

void Remote_Control_Init(UART_HandleTypeDef *huart)
{
    remote_control.RC_USART = huart;
    // 遥控器串口信息初始化 不用深究
    // HAL_UART_Receive_DMA(huart, sbus_rx_buf, RC_FRAME_LENGTH);
    USART_IDLE_Init(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

/**
 * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
 * @Brief  	DR16接收机协议解码程序 详见DT7（遥控器接收机）手册
 * @Param		RC_Type* rc　存储遥控器数据的结构体　　uint8_t* buff　用于解码的缓存
 * @Retval		None
 * @Date
 */
void Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
{
    if (buff == NULL || rc == NULL)
    {
        return;
    }
    memcpy(RC_Data_Buffer, buff, 16);

    rc->UpdateTick = RC_Get_Timeline();

    RC_dt = DWT_GetDeltaT(&RC_DWT_Count);

    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= RC_CH_VALUE_OFFSET;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= RC_CH_VALUE_OFFSET;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= RC_CH_VALUE_OFFSET;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= RC_CH_VALUE_OFFSET;

    rc->switch_left = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->switch_right = (buff[5] >> 4) & 0x0003;

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.press_left = buff[12]; // is pressed?
    rc->mouse.press_right = buff[13];

    rc->key_code = buff[14] | buff[15] << 8; // key borad code

    if (rc->switch_left != 1 && rc->switch_left != 2 && rc->switch_left != 3)
    {
        rc->ch1 = 0;
        rc->ch2 = 0;
        rc->ch3 = 0;
        rc->ch4 = 0;
        rc->switch_left = 0;
        rc->switch_right = 0;

        rc->mouse.x = 0; // x axis
        rc->mouse.y = 0;
        rc->mouse.z = 0;

        rc->mouse.press_left = 0; // is pressed?
        rc->mouse.press_right = 0;

        rc->key_code = 0; // key borad codes
    }

    // if (resetCount < 2000)
    //     Send_RC_Data(&hcan1, buff);
    RC_Update = 1;

    Detect_Hook(RC_TOE);
}

uint8_t is_RC_Lost(void)
{
    if (RC_Get_Timeline() - remote_control.UpdateTick > 30)
        return 1;
    else
        return 0;
}

void Solve_RC_Lost(void)
{
    USART_IDLE_Init(remote_control.RC_USART, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

void Solve_RC_Data_Error(void)
{
    USART_IDLE_Init(remote_control.RC_USART, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

uint8_t RC_Data_is_Error(void)
{
    // 使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (abs(remote_control.ch1) > 1000)
    {
        goto error;
    }
    if (abs(remote_control.ch1) > 1000)
    {
        goto error;
    }
    if (abs(remote_control.ch1) > 1000)
    {
        goto error;
    }
    if (abs(remote_control.ch1) > 1000)
    {
        goto error;
    }
    if (remote_control.switch_left == 0)
    {
        goto error;
    }
    if (remote_control.switch_left == 0)
    {
        goto error;
    }
    return 0;

error:
    remote_control.ch1 = 0;
    remote_control.ch2 = 0;
    remote_control.ch3 = 0;
    remote_control.ch4 = 0;

    remote_control.mouse.x = 0;
    remote_control.mouse.y = 0;
    remote_control.mouse.z = 0;
    remote_control.switch_left = 0;
    remote_control.switch_right = 0;

    remote_control.mouse.x = 0; // x axis
    remote_control.mouse.y = 0;
    remote_control.mouse.z = 0;

    remote_control.mouse.press_left = 0; // is pressed?
    remote_control.mouse.press_right = 0;

    remote_control.key_code = 0; // key borad code
    return 1;
}
