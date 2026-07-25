#ifndef __LOW_Pass_Filter_H
#define __LOW_Pass_Filter_H

#ifndef PI // ! PI
#define PI 3.141593f
#endif

typedef struct lpfParam
{
    float wc;
    float lastOutput;
    char first_flag;
} Lpf_t;

extern Lpf_t LowPassFilterParam;
void LowPassFilter_Init(Lpf_t *lpf, float frequency);
float LowPassFilter(float input, Lpf_t *lpf, float Ts);
float First_Order_LPF(float value, float value_last, float lpf, float dt);
#endif
