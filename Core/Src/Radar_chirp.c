#include "Radar_chirp.h"
#include "stm32f4xx_hal_tim.h"

/* ============================================================
 * ADC1为主ADC,采I路;ADC2为从ADC,采Q路
 * Mode:开启Dual Regular Simultaneous Mode
 *  	让ADC1和ADC2在完全相同的时刻同时启动转换，保证I路和Q路采样之间没有时间差。
 * 		时间差哪怕只有几百纳秒，90°的相位关系会被破坏，FFT分离正负频率的能力变差。
 * 		不能用两次独立的HAL_ADC_Start()来采两路，因为两次调用之间会有us级的延迟。
 *
 * DMA Access Mode：开启DMA Access Mode 2
 * 		Dual模式下两个ADC的结果被自动打包成一个32位值放入ADC->CDR寄存器：
 *		高16位是ADC2结果（Q路），低16位是ADC1结果（I路）。
 *		用DMA把这个32位值搬到内存最高效，避免ISR里两次读寄存器的时间开销。
 *
 * Delay between 2 sampling phase：选择5 Cycles
 *		这个参数控制ADC1和ADC2采样相位的对齐精度，设最小值即可。
 *
 * ADC配置-ADC_Settings
 * Clock Prescaler:选择PCLK2 divided by 4,得到21MHz,不超过36MHz上限
 *
 * Resolution:选择12 bits(15 ADC cycles),保持最高精度,不降低
 *
 * Data Alignment:选择Right alignment。12位结果放低12位,高4为为0,范围为0到4095
 *
 * 开启ADC1的DMA传输:Normal Mode,Direction为Peripheral To Memory，使能Memory Increment,
 * Data Width选择Word(32位);ADC2不需要开启DMA,二者共用一个DMA,避免ISR里两次读寄存器的时间开销;
 * 使能ADC1的DMA Continuous Requests:Dual模式下DMA需要持续搬运打包后的32位结果,每次触发产生一次DMA请求
 * ADC Continuous Conversion Mode：Enable,否则DMA长度为4时可能只完成1次转换
 * 
 * Sampling Time选84 Cycle。ADC的一次完整转换分为两个阶段：
 * 		1、CubeMX中设置的为采样时间,这段时间ADC内部的采样保持电容对输入信号充电,时间
 *		越长充电越完整,对高阻抗信号源越友好
 *		2、12 bit分辨率固定需要15个ADC时钟周期,这是逐次逼近型ADC的硬件决定的,不可改变.
 *		两段时间相加就是总周期数,转换时间=总周期数/ADC时钟频率,ADC采样率就是转换时间的倒数
 *
 * 注意:
 *   CHIRP_STEP_US、ADC_FS_HZ、RADAR_TC_S等配置必须按真实TIM2周期同步修改。
 * ============================================================ */
 
 /* ============================================================
 * TIM2配置
 * Clock Source选择Internal Clock
 * TIM2挂载APB1,Timer clock为84MHz,令Prescaler=83,计数频率=1MHz;
 * Counter Period建议先设为39,对应40μs步进周期,对应CHIRP_STEP_US,后期根据实测优化
 * TIM2 Channel 1配置为Output Compare No Output
 * TIM2 CH1 Pulse:建议先设为15,对应频率写入后约15 us启动ADC,后期根据实测优化
 *
 * 设计目标：
 *   1. TIM2 Update中断:负责保存上一频点数据，并写入下一频点ADF4153A频率;
 *   2. TIM2 CH1 Compare中断:在频率写入后延迟一段时间,再启动ADC DMA
 *   3. ADC DMA一次采集CHIRP_ADC_AVG_COUNT组I/Q,完成后取平均,作为该频率点数据
 *   4. 避免“刚写完频率就立即采样”的问题
 * ============================================================ */
 
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1; /* 主ADC，采I路*/
extern ADC_HandleTypeDef hadc2; /* 从ADC，采Q路*/

#define CHIRP_ADC_AVG_COUNT      4U     /* 每个频点采4组I/Q，取平均 */
#define CHIRP_ADC_START_DELAY_US 15U    /* 写完频率后等待15us再启动ADC，避免“刚写完频率就立即采样”的问题 */

ChirpState_t g_chirp = {
    .dir   = CHIRP_UP,
    .step  = 0,
    .done  = false,
    .iq_buf = {{0, 0}}
};
 
/* -------- 查找表 -------- */
static uint32_t s_INT_up [CHIRP_STEPS];
static uint32_t s_FRAC_up[CHIRP_STEPS];
static uint32_t s_INT_dn [CHIRP_STEPS];
static uint32_t s_FRAC_dn[CHIRP_STEPS];
 
/* DMA一次采集多组ADC->CDR打包数据：
 * CDR格式：[31:16]=ADC2结果(Q)，[15:0]=ADC1结果(I) */
static volatile uint32_t s_iq_raw[CHIRP_ADC_AVG_COUNT];
 
/* -------- ADC采样同步标志 -------- */
static volatile bool s_dma_done = false;     /* ADC DMA已完成，并完成平均 */
static volatile bool s_adc_start_en = false; /* 允许TIM2 CH1触发ADC */
/* ADC平均后的临时结果，由TIM2 Update中断写入g_chirp.iq_buf */
static volatile int16_t s_i_avg = 0;
static volatile int16_t s_q_avg = 0;
/* 调试用：统计采样未完成次数，若该值增加，说明TIM2周期太短或ADC/DMA未按预期完成 */
static volatile uint32_t s_sample_miss_count = 0U;

// 将VCO目标频率 → ADF N分频参数
static void FreqToND(double vco_hz, uint32_t *INT_out, uint32_t *FRAC_out)
{
    double f_adf  = vco_hz / (double)BGT_PRESCALER_DIV;
    double N_real = f_adf  / RADAR_FPFD_HZ;
    uint32_t INT  = (uint32_t)N_real;
    uint32_t FRAC = (uint32_t)((N_real - (double)INT) * (double)RADAR_MOD + 0.5);
    if (FRAC >= (uint32_t)RADAR_MOD) { FRAC = 0U; INT++; }
    *INT_out  = INT;
    *FRAC_out = FRAC;
}

// Init时调用一次，方便运行时直接根据索引查值
void Chirp_Precompute(void)
{
    double f_step = RADAR_BW_HZ / (double)(CHIRP_STEPS - 1);
    for (int i = 0; i < CHIRP_STEPS; i++) {
        FreqToND(RADAR_F_START_HZ + f_step * i,&s_INT_up[i], &s_FRAC_up[i]);
        FreqToND(RADAR_F_STOP_HZ  - f_step * i,&s_INT_dn[i], &s_FRAC_dn[i]);
    }
}
 
/* 写入指定step的ADF频率 */
static void Chirp_SetStepFrequency(uint8_t step)
{
    if (g_chirp.dir == CHIRP_UP) {
        ADF4153A_SetFrequency(s_INT_up[step], s_FRAC_up[step], RADAR_MOD);
    } else {
        ADF4153A_SetFrequency(s_INT_dn[step], s_FRAC_dn[step], RADAR_MOD);
    }
}

// 设起始频率 → 等PLL锁定 → 启动TIM2 Update/Compare
void Chirp_Start(ChirpDir_t dir)
{
    g_chirp.dir  = dir;
    g_chirp.step = 0;
    g_chirp.done = false;

    s_dma_done     = false;
    s_adc_start_en = true; /* 允许TIM2 CH1触发ADC */

    s_i_avg = 0;
    s_q_avg = 0;
    s_sample_miss_count = 0U;

    /* 先停止可能残留的定时器和DMA */
    HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);

    /* 设置起始频率 */
    Chirp_SetStepFrequency(g_chirp.step);
    /* 起始频率跳变较大，保守等待PLL锁定 */
    HAL_Delay(1);

    /* 设置比较点:计数器到达CHIRP_ADC_START_DELAY_US时启动ADC采样 */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CHIRP_ADC_START_DELAY_US);
    /* 清除定时器标志 */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    /* 启动定时器，等待中断触发 */
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
}
 
// 等待本次chirp完成，不用标志位就只能死等固定时间或轮询step
void Chirp_WaitDone(void)
{
    while (!g_chirp.done) {
        __WFI(); // 等待中断唤醒
    }
}
 
/* ADC DMA完成回调:对CHIRP_ADC_AVG_COUNT组I/Q求平均 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        int32_t sum_i = 0;
        int32_t sum_q = 0;

        for (uint32_t n = 0; n < CHIRP_ADC_AVG_COUNT; n++) {
            uint32_t raw = s_iq_raw[n];
            // CDR[15:0]  = ADC1结果 = I路(0~4095)
            // CDR[31:16] = ADC2结果 = Q路(0~4095)
            int32_t i_raw = (int32_t)( raw        & 0xFFFFU);
            int32_t q_raw = (int32_t)((raw >> 16) & 0xFFFFU);

            sum_i += i_raw;
            sum_q += q_raw;
        }

        /* 平均后减2048去直流偏置,转换为有符号值(-2048~+2047) */
        s_i_avg = (int16_t)((sum_i / (int32_t)CHIRP_ADC_AVG_COUNT) - 2048);
        s_q_avg = (int16_t)((sum_q / (int32_t)CHIRP_ADC_AVG_COUNT) - 2048);

        s_dma_done = true;

        /* 连续转换模式下，DMA完成后停止ADC，避免继续转换 */
        HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    }
}

/* TIM2 CH1 Compare中断:频率写入并延迟后,启动ADC DMA */
static void Chirp_CompareISR(void)
{
    if (!g_chirp.done && s_adc_start_en) {
        s_dma_done = false;

        /* 避免上一次DMA残留，然后重新启动ADC DMA */
        (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)s_iq_raw, CHIRP_ADC_AVG_COUNT);
    }
}

/* TIM2 Update中断：保存上一点数据，写入下一步频率 */
void Chirp_UpdateISR(void)
{
    /* 1. 保存当前step的ADC平均数据 */
    if (s_dma_done) {
        g_chirp.iq_buf[g_chirp.step].i = s_i_avg;
        g_chirp.iq_buf[g_chirp.step].q = s_q_avg;
        s_dma_done = false;
    } else {
        /* 如果进入下一步时上一点ADC还没完成，说明时序设计不够。
         * 这里先保留为0并计数，后续可改为置错误标志。 */
        g_chirp.iq_buf[g_chirp.step].i = 0;
        g_chirp.iq_buf[g_chirp.step].q = 0;
        s_sample_miss_count++;
    }
 
    /* 2. 判断是否完成 */
    if (g_chirp.step >= (CHIRP_STEPS - 1U)) {
        s_adc_start_en = false;

        HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
        HAL_TIM_Base_Stop_IT(&htim2);
        HAL_ADCEx_MultiModeStop_DMA(&hadc1);

        g_chirp.done = true;
        return;
    }

    /* 3. 写入下一步频率 */
    g_chirp.step++;
    s_dma_done = false;
    s_adc_start_en = true;

    Chirp_SetStepFrequency(g_chirp.step);

    /* 4. 下一次TIM2 CH1 Compare到来后启动ADC */
}
 
/* TIM2 Update回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        Chirp_UpdateISR();
    }
}

/* TIM2 Output Compare回调 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) {
        Chirp_CompareISR();
    }
}