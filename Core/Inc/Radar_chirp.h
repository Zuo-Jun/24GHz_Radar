#ifndef RADAR_CHIRP_H
#define RADAR_CHIRP_H

// chirp驱动：定时器ISR驱动ADF4153A逐步扫频
#include "stm32f4xx_hal.h"
#include "ADF4153A.h"
#include "Radar_config.h"
#include <stdint.h>
#include <stdbool.h>

// IO采样对：每步采集一对I/Q值
typedef struct {
    int16_t i;   /* I路：ADC1结果 - 2048（去直流后） */
    int16_t q;   /* Q路：ADC2结果 - 2048（去直流后） */
} IQ_Sample_t;

// chirp方向
typedef enum {
    CHIRP_UP   = 0,   /* 上扫：RADAR_F_START_HZ → RADAR_F_STOP_HZ */
    CHIRP_DOWN = 1,   /* 下扫：RADAR_F_STOP_HZ  → RADAR_F_START_HZ */
} ChirpDir_t;

// chirp状态（由定时器ISR维护）
typedef struct {
    ChirpDir_t  dir;   // 扫频方向
    uint8_t     step;  // 当前步数索引(0到CHIRP_STEPS-1)
    bool        done;  // 采样完成标志。false代表进行中，true代表本次chirp全部完成采样
    IQ_Sample_t iq_buf[ADC_SAMPLES_PER_CHIRP];  // I/Q采样数据缓冲区
} ChirpState_t;

extern ChirpState_t g_chirp;

void Chirp_Precompute(void); /* 预计算频率查找表，Init时调用一次 */
void Chirp_Start(ChirpDir_t dir); /* 设置初始频率、启动ADC DMA、启动TIM2 */

void Chirp_WaitDone(void); /* 等待chirp完成（阻塞） */
void Chirp_StepISR(void); /* 定时器ISR中调用 */

/* 使用流程：
 * Chirp_Start(CHIRP_UP);       // 启动扫频
 * Chirp_WaitDone();            // 阻塞等待完成←用到done标志
 * Process_FFT(g_chirp.iq_buf); // 处理数据 
*/

#endif


