#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

/* ============================================================
 *  所有频率/硬件参数统一从ADF4153A.h引用，不重复定义。
 *  此文件只定义雷达信号参数和DSP参数。
 * ============================================================ */
 
#include "ADF4153A.h" /* ADF_REFIN_HZ, BGT_PRESCALER_DIV, ADF4153A_CalcFpfd/Fres/RFout */

// 物理常数 光速
#define SPEED_OF_LIGHT      299792458.0

// 雷达RF参数
#define RADAR_FC_HZ         24125000000.0   /* 中心频率 24.125GHz */
#define RADAR_F_START_HZ    24000000000.0   /* 扫频起始 24.000GHz */
#define RADAR_F_STOP_HZ     24250000000.0   /* 扫频终止 24.250GHz */
#define RADAR_BW_HZ         (RADAR_F_STOP_HZ - RADAR_F_START_HZ) /* 带宽 B = 250MHz */

// PLL参数，注意：Fpfd上限32MHz
#define RADAR_REFIN_DBL     0               /* D=0，不倍频 */
#define RADAR_R             1               /* 参考分频R=1 */
#define RADAR_PRESCALER     0               /* 0=4/5模式（RFin≤2GHz）INT实际值为60，比最小值31大 */
/* ============================================================
 *  分辨率是指能设置的最小频率步进。FRAC每次+1，VCO频率变化：
 *  ΔFvco= Fpfd/MOD×BGT_DIV。这个ΔFvco是chirp的每步频率增量，与带宽B决定了每Chirp的步长CHIRP_STEPS
 *  MOD数值范围为2到4095，选择125的原因有二，其一是可以被25MHz整除，没有量化误差
 *  其二是因为125不能被2、3、6整除，ADF4153A芯片手册P18明确说这种MOD值可以避免subfractional杂散。
 *  RADAR_MOD可以设置为其他值，只要能被25MHz整除，且CHIRP_STEPS够用覆盖250MHz带宽即可。
 * ============================================================ */
#define RADAR_MOD           125             /* 小数分母，决定步进精度 */

// 编译期计算Fpfd（单位Hz）
#define RADAR_FPFD_HZ       ((double)ADF_REFIN_HZ * (1.0 + RADAR_REFIN_DBL) / RADAR_R)
// VCO端每步频率增量
#define RADAR_F_STEP_VCO_HZ (RADAR_FPFD_HZ / RADAR_MOD * (double)BGT_PRESCALER_DIV)
	
/* ============================================================
 *  Chirp时序参数
 *
 *  Tc不是随意定的，它同时约束了三件事，需要三者之间取平衡
 *  1、距离分辨率只取决于带宽B，Dres=c/2B,Tc改变不影响距离测量精度
 *  2、Tc决定了速度测量能力。对于不同距离物体，Vmax=λ/4Tc
 *  已知波长λ约为12.44mm，Tc越短，Vmax越大。但Tc为1ms时，Vmax为3.11m/s≈11.2km/h
 *  测河道水速小于3m/s情况下，Tc=1ms够用，但要测量行驶车辆速度不够，需要降额
 *  3、Tc决定了单次chirp内每步间隔CHIRP_STEP_US，进而决定了SPI速度要求
 *  每步间隔越短，SPI需要在更短的时间内完成24位传输，这是硬件的硬性限制，所以Tc不能无限缩短。
 * 
 *  CHIRP_STEPS：带宽/步进 = 250MHz/3.2MHz = 78.125，取79步
 *  79步实际覆盖 79×3.2MHz = 252.8MHz（比250MHz多1%，可接受）
 *
 *  CHIRP_STEP_US：每步间隔=13μs（定时器TIM2周期）
 *  时间分配：SPI写R1+R0约5μs → PLL锁定约5μs → ADC采样约3μs
 *  实际Tc = 79×13μs = 1.027ms（误差2.7%，可接受)
 * ============================================================ */
#define RADAR_TC_S          0.001           /* 单次chirp时间 Tc = 1ms */
#define RADAR_PLL_SETTLE_S  0.001           /* Chirp_Start() HAL_Delay(1), PLL settle time */
#define RADAR_WATER_PHASE_DT_S (2.0 * (RADAR_TC_S + RADAR_PLL_SETTLE_S)) /* UP phase-to-phase interval in UP/DOWN water-speed mode */

#define CHIRP_STEPS         ((int)(RADAR_BW_HZ/RADAR_F_STEP_VCO_HZ+0.5))  /* 每次chirp的频率步数 */
#define CHIRP_STEP_US       RADAR_TC_S*1e6/CHIRP_STEPS   /* 每步间隔 = Tc/steps ≈ 12.7μs，取13μs */


// 帧结构参数
#define N_CHIRPS_WATER_V    16  /* 水速测量：16对三角波（上+下各16次，共32ms） */
#define N_CHIRPS_CAR        4   /* 车速测量：4对三角波取平均，够用且快 */


/* ============================================================
 *  ADC参数
 *
 *  IF最大频率推导（测10m目标）：
 *    回波延时τ= 2×10m/c=67ns
 *    IFmax=S×τ= (B/Tc)×τ= (250MHz/1ms)×67ns = 16.7kHz
 *    奈奎斯特最低采样率fs=2×16.7kHz=33.4kHz→取40ksps留余量
 *
 *  ksps = kilo-samples per second（每秒千次采样），ADC速率单位
 *  SNR = 信号功率/噪声功率，通常用dB表示，dB值越大越好
 *  过采样改善SNR的原理是：ADC的量化噪声均匀分布在0到fs/2的频带内，有用信号只占其中很窄的一段
 *  （比如0到16.7kHz）。采样率越高，噪声被稀释到更宽的频带，落在有用信号带宽内的噪声就越少。
 *
 *  选200ksps的理由：
 *    1、过采样因子OSR = 实际采样率/奈奎斯特最低采样率 = 200ksps/33.4ksps ≈ 6
 *    SNR（信噪比）改善 = 10×log10(OSR) = 10×log10(6) ≈ 7.8dB
 *    2、STM32F4 ADC最高2.4Msps，200ksps轻松实现
 *    每步13μs内完成一次转换（200ksps→每次5μs<13μs）
 * ============================================================ */
#define ADC_FS_HZ           200000          /* ADC采样率 200ksps */
#define ADC_SAMPLES_PER_CHIRP CHIRP_STEPS   /* 每步采1点，共79点 */


// 物理量计算
#define RADAR_WAVELENGTH    (SPEED_OF_LIGHT / RADAR_FC_HZ)  /* 雷达波长λ≈12.44mm */
#define RADAR_S             (RADAR_BW_HZ / RADAR_TC_S)      /* 调频斜率 250GHz/s */

#define RADAR_DRANGE        (SPEED_OF_LIGHT / (2.0 * RADAR_BW_HZ)) /* 距离分辨率=0.60m */
/* 数值上等于RADAR_DRANGE，意义是FFT第k个bin对应距离k×RANGE_PER_BIN */
#define RANGE_PER_BIN       RADAR_DRANGE
/* 水速相位差分最大不模糊速度=λ/(4Δt) */
#define V_MAX_SINGLE_PAIR   (RADAR_WAVELENGTH / (4.0 * RADAR_WATER_PHASE_DT_S))


#endif 

