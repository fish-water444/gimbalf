#include "low_pass_filter.h"
#include "main.h"
#include <math.h>
/**
 ******************************************************************************
 * @file    low_pass_filter.c
 * @author  Sihan Cheng
 * @version V1.0
 * @date    2024/12/15
 * @brief   Standard Low-Pass-Filter(LPF) for signal noise or Inertia link
 *                  (低通滤波器用来抑制高频噪声or添加惯性环节)
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "low_pass_filter.h"
Lpf_t LowPassFilterParam = {1};
void LowPassFilter_Init(Lpf_t *lpf, float frequency)
{
    lpf->wc = 2 * PI * frequency; // wc = 2*pi*fc ，fc为信号频率，单位Hz
    lpf->first_flag = 1;
}

inline float LowPassFilter(float input, Lpf_t *lpf, float Ts) // Ts为采样时间，wc为信号的角频率
{
    float alpha = lpf->wc * Ts / (lpf->wc * Ts + 1);

    float output = 0;    // 当前滤波值
    if (lpf->first_flag) // 判断是否为第一次进行滤波
    {
        lpf->first_flag = 0;
        lpf->lastOutput = input; // 赋初值
    }

    output = lpf->lastOutput + alpha * (input - lpf->lastOutput); // 更新滤波值
    lpf->lastOutput = output;                                     // 更新上一次滤波值

    return output;
}


/**
  * @brief        一阶低通滤波
  * @param[in]    输入
  * @param[in]    上一次滤波输入
  * @param[in]    lpf系数
  * @param[in]    dt      
  */
float First_Order_LPF(float value, float value_last, float lpf, float dt)
{
    value = value*dt / (lpf + dt) + value_last*lpf / (lpf + dt);
    return value;
}
