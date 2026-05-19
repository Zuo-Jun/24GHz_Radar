#include "Radar_chirp.h"

/* ============================================================
 * ADC1为主ADC，采I路；ADC2为从ADC，采Q路
 * ADC配置-ADCs_Common_Settings
 * Mode：开启Dual Regular Simultaneous Mode
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
 * Clock Prescaler：选择PCLK2 divided by 4，得到21MHz，不超过36MHz上限
 *
 * Resolution：选择12 bits(15 ADC cycles)，保持最高精度，不降低
 *
 * Data Alignment：选择Right alignment。12位结果放低12位，高4为为0，范围为0到4095。
 *
 * 开启ADC1的DMA传输，Direction为Peripheral To Memory，Data Width选择Word(32位)
 * 		ADC2不需要开启DMA，二者共用一个DMA，避免ISR里两次读寄存器的时间开销。
 *
 * 使能ADC1的DMA Continuous Requests：Dual模式下DMA需要持续搬运打包后的32位结果，
 * 		每次触发产生一次DMA请求
 * 
 * ADC配置-ADC_Regular_ConversionMode
 * Number Of Conversion选择1。只有一个信号，不需要多通道扫描。
 *
 * ADC1的External Trigger Conversion Source选择Regular Conversion launched by software
 * 		ISR里调用HAL_ADCEx_MultiModeStart_DMA()同时启动ADC1和ADC2。
 * 
 * Sampling Time选84 Cycle。ADC的一次完整转换分为两个阶段：
 * 		1、CubeMX中设置的为采样时间，这段时间ADC内部的采样保持电容对输入信号充电，时间
 *		越长充电越完整，对高阻抗信号源越友好。
 *		2、12 bit分辨率固定需要15个ADC时钟周期，这是逐次逼近型ADC的硬件决定的，不可改变。
 *		两段时间相加就是总周期数，转换时间=总周期数/ADC时钟频率，ADC采样率就是转换时间的倒数
 * ============================================================ */
 
 /* ============================================================
 * TIM2配置
 * Clock Source选择Internal Clock
 * TIM2挂载APB1，Timer clock为84MHz，令Prescaler=83，让计时器时钟降到1MHz
 * Counter Period=12，周期=(12+1)×1us=13us，对应CHIRP_STEP_US。
 * ============================================================ */
 
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1; /* 主ADC，采I路*/
extern ADC_HandleTypeDef hadc2; /* 从ADC，采Q路*/

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
 
/* 每次ADC转换后DMA自动填入，DMA把ADC->CDR搬到s_iq_raw
 * CDR格式：[31:16]=ADC2结果(Q)，[15:0]=ADC1结果(I) */
static volatile uint32_t s_iq_raw = 0; 
 
/* -------- DMA是否完成标志（ISR和回调之间同步） -------- */
static volatile bool s_dma_done = false;
 

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
 

// 设起始频率 → 等PLL锁定 → 启动DMA → 启动TIM2
void Chirp_Start(ChirpDir_t dir)
{
    g_chirp.dir  = dir;
    g_chirp.step = 0;
    g_chirp.done = false;
    s_dma_done   = false;
 
    /* 根据chirp方向设置初始频率 */
    if (dir == CHIRP_UP) {
        ADF4153A_SetFrequency(s_INT_up[0], s_FRAC_up[0], RADAR_MOD);
    } else {
        ADF4153A_SetFrequency(s_INT_dn[0], s_FRAC_dn[0], RADAR_MOD);
    }
 
    /* 等PLL锁定到起始频率（从上一chirp末尾跳到起始，最坏频差250MHz）
     * 环路带宽20kHz，锁定约需0.5ms，用1ms保守等待 */
    HAL_Delay(1);
 
    /* 启动ADC双通道DMA（每次ISR触发一次转换，DMA自动搬运1个word）
     * HAL_ADCEx_MultiModeStart_DMA：
     *   - 启动ADC1为主机，ADC2为从机，同步触发模式
     *   - DMA把ADC->CDR搬到s_iq_raw（1个uint32_t）
     *   - 每次转换完成DMA产生TC中断 → 调用回调 */
    HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t *)&s_iq_raw,1);  /* 每次搬1个word */
 
    /* 启动TIM2中断定时器 */
    HAL_TIM_Base_Start_IT(&htim2);
}
 
// 告知应用层chirp完成，不用标志位就只能死等固定时间或轮询step
void Chirp_WaitDone(void)
{
    while (!g_chirp.done) {
        __WFI(); // 等待中断唤醒
    }
}
 
// ADC->CDR 32位数据已就绪，从ADC外设通过DMA传输到Memory
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        s_dma_done = true;
    }
}

void Chirp_StepISR(void)
{
    uint8_t step = g_chirp.step;
 
    /* 1. 读取并存储本步的IQ数据
     *    CDR[15:0]  = ADC1结果 = I路（0~4095）
     *    CDR[31:16] = ADC2结果 = Q路（0~4095）
     *    减2048去直流偏置，转换为有符号值（-2048~+2047） */
    if (s_dma_done) {
        uint32_t raw = s_iq_raw;
		/*
		* 目前这里采用的是硬编码去掉直流偏置，可换为动态减均值。
		* 对每帧数据先算再均值再整体相减，自适应地消除硬件误差，但需额外累加运算
		*/
        g_chirp.iq_buf[step].i = (int16_t)( raw        & 0xFFFFU) - 2048;
        g_chirp.iq_buf[step].q = (int16_t)((raw >> 16) & 0xFFFFU) - 2048;
        s_dma_done = false;
    }
 
    /* 2. 判断是否还有下一步 */
	if(step<CHIRP_STEPS-1){
		// 还有下一步，写写一步的ADF频率，重启DMA
		uint8_t next = step + 1;
		
		if (g_chirp.dir == CHIRP_UP) {
        ADF4153A_SetFrequency(s_INT_up[next], s_FRAC_up[next], RADAR_MOD);
		} else {
			ADF4153A_SetFrequency(s_INT_dn[next], s_FRAC_dn[next], RADAR_MOD);
		}
 
	/* 3. 停止旧DMA → 重启新DMA。HAL的MultiMode DMA是单次模式（length=1）
	 * 转换完成后DMA自动停止，不会自动重新触发。*/
		HAL_ADCEx_MultiModeStop_DMA(&hadc1);
		s_dma_done = false; // 避免因时延问题导致的误判，确保每次ADC启动时标志为假
		HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t *)&s_iq_raw,1);
		
		g_chirp.step = next;
	}
    else{
		// step=CHIRP_STEPS-1，结束本次chirp
        HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        HAL_TIM_Base_Stop_IT(&htim2);
        g_chirp.done = true;
		/* 此后Chirp_WaitDone()里的while循环会退出
         * 调用方负责决定下一步做什么（启动下扫chirp、或切换到下一个测量阶段） */
    }
}
 
// 定时器中断处理函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        Chirp_StepISR();
    }
}

