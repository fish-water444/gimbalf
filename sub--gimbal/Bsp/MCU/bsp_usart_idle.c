#include "bsp_usart_idle.h"
#include "main.h"
/**
 ******************************************************************************
 * @file    bsp_usart_idle.c
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/1/5
 * @brief
 ******************************************************************************
 * @attention
 * Usart DMA mode must be "DMA_CIRCULAR" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 ******************************************************************************
 */
#include "bsp_usart_idle.h"

void USART_IDLE_Init(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t dma_buf_num)
{
    // enable the DMA transfer for the receiver request
    // 使能DMA串口接收
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    // enalbe idle interrupt
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    // disable DMA
    // 失效DMA
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
    }

    huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
    // memory buffer 1
    // 内存缓冲区1
    huart->hdmarx->Instance->M0AR = (uint32_t)(rx_buf);
    // data length
    // 数据长度
    huart->hdmarx->Instance->NDTR = dma_buf_num;

    // enable DMA
    // 使能DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void USART_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
    // 不用深究
    uint32_t isrflags = READ_REG(huart->Instance->SR);
    uint32_t cr1its = READ_REG(huart->Instance->CR1);
    uint32_t cr3its = READ_REG(huart->Instance->CR3);

    if (huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }
    else if (huart->Instance->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);

        __HAL_DMA_DISABLE(huart->hdmarx);

        __HAL_DMA_ENABLE(huart->hdmarx);

        USER_UART_RxIdleCallback(huart);
    }

    // 该段代码与接收数据无关，用于保证DMA发送正常进行
    if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
    {
        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

        /* Tx process is ended, restore huart->gState to Ready */
        huart->gState = HAL_UART_STATE_READY;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Tx complete callback*/
        huart->TxCpltCallback(huart);
#else
        /*Call legacy weak Tx complete callback*/
        HAL_UART_TxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    }
}

/**
 * @brief  Rx Transfer idle callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
__weak void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}
