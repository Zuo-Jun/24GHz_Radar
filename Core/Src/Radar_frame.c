#include "Radar_frame.h"
#include "Radar_chirp.h"
#include "arm_math.h"
#include <string.h>
#include <math.h>
    
#define FFT_SIZE        128

/* CFAR恒虚警率检测(根据环境自适应)
 * 问题背景：
 *		若背景噪声功率均匀且已知，设一个固定门限即可。但现实中噪声/杂波功率随距离、方位、时间变化。
 *		固定门限会导致噪声强的地方虚警爆炸(有真有假)，噪声弱的地方漏检严重(真的可能都认为是假的)。
 * 		以自动感应水龙头举例，假设感应距离固定50cm。小孩手短(信号弱)→够不到50cm→不出水(漏检)
 *		大人正常洗手→刚好50cm→正常出水(检测成功)；但有人走过/衣服晃动(噪声强)→也触达50cm→乱出水(虚警)
 *
 * 单元平均CFAR具体步骤：
 *		1、滑窗：以待检距离单元(CUT)为中心，左右各取N个参考单元(CFAR_REF)，
 *		中间留几个保护单元(CFAR_GUARD)隔开，防止目标能量泄漏污染估计。
 *		RRRRRRRRGGCGGRRRRRRRR
 *		2、估计噪声：对2N个参考单元的功率求平均，得到noise_avg，这就是当前位置的局部噪声/杂波功率估计。
 * 		3、设置门限：放大因子CFAR_THRESHOLD乘以noise_avg
 *		4、判决：若CUT的功率sig大于设置的门限，则判为目标；否则判为噪声。
 *
 * 关于GUARD、REF和THRESHOLD:
 *		CFAR_GUARD保护单元是给加窗后的"变胖目标"留的隔离带，不让它污染参考单元的噪声估计，
 *		宽度为加窗后的主瓣宽度向上取整；
 *		CFAR_REF参考单元是噪声统计的样本数，REF越大，SNR损失越小，但计算量大
 *		CFAR_THRESHOLD由选定的虚警率决定，选的过大则会发生漏检；选的过小则会发生虚警 
 *
 *  算法          噪声估计方式           	核心思路			适用场景
 * CA-CFAR   所有参考单元取均值			最简单，统计效率最高	均匀场景
 * GO-CFAR   取左右两侧均值中的较大值	宁可漏检也不虚警		杂波边缘(防虚警爆炸)
 * SO-CFAR   取左右两侧均值中的较小值	宁可虚警也不漏检		杂波边缘(防漏检)
 * OS- CFAR  参考单元排序，取第k大		对少量强干扰样本不敏感	多目标密集场景
*/
#define CFAR_GUARD      1       // 每侧保护单元数,用于隔离目标主瓣,避免目标能量进入噪声估计
#define CFAR_REF        6       // 每侧参考单元数
#define CFAR_THRESHOLD  13.0f   // 门限系数(10×log10(13)≈11dB)
#define CFAR_EPSILON	1e-30f  // 除零保护
#define CFAR_MIN_BIN        2   // 跳过DC和极低频杂散,避免bin 0/1的直流泄漏影响检测
#define CFAR_MIN_REF_TOTAL  4   // 至少需要的参考单元总数,用于保证噪声估计至少有一定样本数,避免边缘处参考单元过少导致门限不稳定

/**
 * @param[in] mag : 输入幅度谱数组，长度为len，通常为FFT_SIZE/2
 * @param[in] len : 输入幅度谱长度
 *
 * @return int : 检测到的最佳目标bin索引，-1表示未检测到目标
 *
 * @note 边缘自适应CA-CFAR：
 *       - 常规CA-CFAR要求CUT左右两侧都有完整参考窗，因此近距离bin会被跳过。
 *       - 本函数允许边缘bin参与检测：当左侧参考窗不足时，只使用可用的左侧参考单元
 *         和右侧参考单元估计噪声；右边缘同理。
 *       - 若多个bin超过门限，返回信噪比最高的bin，而不是第一个超过门限的bin。
 */
static int cfar_detect(const float *mag, int len)
{
    int   best_bin   = -1;      // 最佳目标bin
    float best_ratio = 0.0f;    // 最佳目标的功率信噪比

    /* refine_peak()需要访问bin-1和bin+1。因此这里最多检测到len-2，避免后续峰值插值越界 */
    int max_bin = len - 2;

    /* 从CFAR_MIN_BIN开始检测，跳过DC附近的直流残留、泄漏和慢变杂波。
     * 与传统CFAR不同，这里不要求 i >= CFAR_GUARD + CFAR_REF，
     * 因此近距离目标不会被左侧参考窗边界直接屏蔽。*/
    for (int i = CFAR_MIN_BIN; i <= max_bin; i++) {
        float noise = 0.0f;
        int ref_count = 0;

        /* 左参考窗: [i - CFAR_GUARD - CFAR_REF, i - CFAR_GUARD - 1]
         * 右参考窗: [i + CFAR_GUARD + 1,   i + CFAR_GUARD + CFAR_REF]*/
        int left_start  = i - CFAR_GUARD - CFAR_REF;
        int left_end    = i - CFAR_GUARD - 1;
        int right_start = i + CFAR_GUARD + 1;
        int right_end   = i + CFAR_GUARD + CFAR_REF;

        /* 边缘bin没有完整参考窗时，只裁剪到数组有效范围内 */
        if (left_start < 0) left_start = 0;
        if (left_end >= len) left_end = len - 1;
        if (right_start < 0) right_start = 0;
        if (right_end >= len) right_end = len - 1;

        /* 累加左侧可用参考单元的功率 */
        for (int k = left_start; k <= left_end; k++) {
            noise += mag[k] * mag[k];
            ref_count++;
        }

        /* 累加右侧可用参考单元的功率。 */
        for (int k = right_start; k <= right_end; k++) {
            noise += mag[k] * mag[k];
            ref_count++;
        }

        /* 边缘处如果可用参考单元太少，噪声估计会非常不稳定。此时跳过该CUT，不做检测。*/
        if (ref_count < CFAR_MIN_REF_TOTAL) {
            continue;
        }

        float noise_avg = noise / (float)ref_count; // 局部平均噪声功率
        float sig = mag[i] * mag[i];                // CUT信号功率

        /* CA-CFAR判决:目标功率>门限系数*局部噪声平均功率 */
        if (sig > CFAR_THRESHOLD * noise_avg) {
            float snr_ratio = sig / (noise_avg + CFAR_EPSILON);

            /* 如果多个bin过门限，选择功率信噪比最高的bin */
            if (snr_ratio > best_ratio) {
                best_ratio = snr_ratio;
                best_bin = i;
            }
        }
    }

    return best_bin;
}

/* 离散采样本质是对连续信号进行时域截断，相当于信号乘上了一个矩形窗，造成频谱旁瓣高(-13dB)，容易掩盖弱目标
 * 窗函数的本质是对真实采集到的信号样本做加权
 *
 * cosf是余弦函数的单精度浮点版本，嵌入式/DSP常用，速度快
 * cos是双精度浮点数，高精度计算，但运算慢
*/
static float s_window[CHIRP_STEPS];
static bool  s_window_init = false;

static void init_window(void)
{
    if (s_window_init) return;
    for (int i = 0; i < CHIRP_STEPS; i++) {
 /* hanning窗适用于多目标检测，需要低旁瓣的场景
 *		优点：旁瓣-31dB，衰减快，计算简单
 * 		缺点：主瓣宽为2 bin，分辨率下降*/		
        s_window[i]=0.5f*(1.0f-cosf(2.0f*PI*(float)i/(float)(CHIRP_STEPS - 1)));
		
/* hamming窗适用于单目标、需要极低第一旁瓣的场景
 *		优点：第一旁瓣更低(-41dB)
 * 		缺点：旁瓣衰减慢（拖尾长），主瓣宽为2 bin，可能掩盖邻近弱目标*/
//		s_window[i] = 0.54f-0.46f*cosf(2.0f*PI*(float)i/(float)(CHIRP_STEPS - 1));
		
/* hamming窗适用于强干扰环境、弱目标在强目标旁边的场景
 *		优点：旁瓣-58dB，非常低
 * 		缺点：主瓣宽为3 bin，分辨率最差*/		
//		s_window[i] = 0.42f-0.50f*cosf(2.0f*PI*(float)i/(float)(CHIRP_STEPS - 1))
//				 +0.08f*cosf(4.0f*PI*(float)i/(float)(CHIRP_STEPS-1));
    }
    s_window_init = true;
}

/* ============================================================
 *  Range-FFT核心（复数输入，含方向修正）
 *
 *  dir参数的作用：
 *    上扫(CHIRP_UP)：拍频为正，FFT后能量在bin 0~63（正频率），Q不变
 *    下扫(CHIRP_DOWN)：拍频为负，FFT后能量在bin 64~127（负频率）
 *      → 对Q取反（即复数取共轭）：负频率翻转到正频率
 *      → 后续cfar_detect和bin_to_range无需修改
 *
 *  IQ输入已在chirp.c去直流（减2048），范围-2048~+2047
 *  加窗 → 零填充 → 复数FFT → 正频率幅度谱
 * ============================================================ */
static void run_range_fft(const IQ_Sample_t *iq_buf,ChirpDir_t dir,
                           float *mag,float *fft_buf){
    init_window();
    memset(fft_buf, 0, sizeof(float) * 2 * FFT_SIZE);

    float q_sign = (dir == CHIRP_UP) ? 1.0f : -1.0f;
 
    for (int i = 0; i < CHIRP_STEPS; i++) {
        float w = s_window[i];
        fft_buf[2 * i]     = (float)iq_buf[i].i * w;
        fft_buf[2 * i + 1] = (float)iq_buf[i].q * w * q_sign;
    }
    /* 剩余bin（79~127）已被memset清零，即零填充 */
 
    arm_cfft_instance_f32 inst;
    arm_cfft_init_f32(&inst, FFT_SIZE);
    arm_cfft_f32(&inst, fft_buf, 0, 1);            /* 正向FFT，位反转使能 */
    arm_cmplx_mag_f32(fft_buf, mag, FFT_SIZE / 2); /* 取正频率幅度（64个bin） */
}

/* ============================================================
 *  三点抛物线插值：亚bin精度峰值定位
 *
 *  原理：假设峰值附近的幅度曲线是抛物线
 *    y0=mag[bin-1], y1=mag[bin], y2=mag[bin+1]
 *    抛物线顶点偏移 = 0.5×(y0-y2)/(y0-2y1+y2)
 *
 *  效果：距离误差从最大0.5bin×0.234m/bin≈0.12m
 *        降到约0.05bin×0.234m/bin≈0.012m
 * ============================================================ */
static float refine_peak(const float *mag, int bin, int len)
{
    if (bin <= 0 || bin >= len - 1) return (float)bin;
    float y0 = mag[bin - 1], y1 = mag[bin], y2 = mag[bin + 1];
    return (float)bin + 0.5f * (y0 - y2) / (y0 - 2.0f * y1 + y2);
}
 
/* ============================================================
 *  目的：bin序号 → 距离（米）
 *
 *  距离 d = f_beat × c × Tc / (2B)=f_beat × c / (2S)
 *
 *  目前设置参数：ADC_FS=200kHz，FFT_SIZE=128，S=250GHz/s
 *    每bin对应频率 = 200000/128 = 1562.5Hz
 *    每bin对应距离 = 1562.5 × 3e8 / (2×250e9) ≈ 0.234m
 * ============================================================ */
static float bin_to_range(float bin)
{
    float f_beat = bin * (float)ADC_FS_HZ / (float)FFT_SIZE;
    return f_beat * (float)SPEED_OF_LIGHT / (2.0f * (float)RADAR_S);
}
 
/* ============================================================
 *  DSP_WaterLevel：单次上扫 → 水面距离
 *  内部固定使用CHIRP_UP（水位只用上扫），不需要外部传dir
 * ============================================================ */
static bool calc_WaterLevel(const IQ_Sample_t *iq_buf, float *range_m, float *snr)
{
    static float fft_buf[2 * FFT_SIZE];
    static float mag    [FFT_SIZE/2];
 
    run_range_fft(iq_buf, CHIRP_UP, mag, fft_buf);
    int   peak_bin = cfar_detect(mag, FFT_SIZE/2);
    if (peak_bin < 0) return false;  /* 未检测到目标 */
 
    *range_m = bin_to_range(refine_peak(mag, peak_bin, FFT_SIZE/2));
 
    /* SNR估计：峰值幅度/低频段噪声均值（bin 1~9）
     * 用低频段而不是全局平均，避免远端目标拉高噪声底 */
//	if(snr!=NULL){
//		float noise = 0.0f;
//		for (int i = 1; i < 10; i++) noise += mag[i];
//		noise /= 9.0f;
//		*snr = 20.0f*log10f(mag[peak_bin]/ (noise + CFAR_EPSILON));
//	}
    return true;
}
 
/* ============================================================
 *  DSP_ExtractPeakComplex：提取目标峰处的复数FFT系数
 *
 *  用途：水速测量时，每次chirp调用此函数得到一个复数，
 *        然后对N个复数取相位，传给DSP_WaterVelocity做差分
 *
 *  复数包含相位信息：
 *    相位 φ =atan2(imag, real)
 *    相邻chirp相位差 Δφ=4π×δd/λ（δd=目标移动距离）
 *    速度 v =λ×Δφ/(4π×Tc)
 *
 *  dir参数：水速测量固定传CHIRP_UP，下扫chirp不用于水速
 * ============================================================ */
static bool ExtractPeakComplex(const IQ_Sample_t *iq_buf,ChirpDir_t dir,
                              float *real_out,float *imag_out,float *range_m_out)
{
    static float fft_buf[2 * FFT_SIZE];
    static float mag    [FFT_SIZE/2];
 
    run_range_fft(iq_buf, dir, mag, fft_buf);
    int bin = cfar_detect(mag, FFT_SIZE/2);
    if (bin < 0) {
        *real_out    = 0.0f;
        *imag_out    = 0.0f;
        *range_m_out = -1.0f;   /* -1表示未检测到目标 */
        return false;
    }
 
    /* 返回该bin处的复数FFT系数
     * fft_buf是实虚交织存储：[real0, imag0, real1, imag1, ...] */
    *real_out    = fft_buf[2 * bin];
    *imag_out    = fft_buf[2 * bin + 1];
    *range_m_out = bin_to_range((float)bin);
	return true;
}
 
/* ============================================================
 *  DSP_WaterVelocity：相位差分法估计水面流速
 *
 *  输入：N次连续上扫chirp各自的距离峰相位（弧度）
 *  输出：水面流速（m/s），正值=水流靠近雷达
 *
 *  原理：
 *    相邻chirp之间目标移动δd
 *    相位变化 Δφ = 4π×δd/λ
 *    速度 v = λ×Δφ/(4π×Tc)
 *
 *  相位展开（while循环）：
 *    消除2π跳变，确保Δφ在[-π,π]范围内
 *    最大不模糊速度 = λ/(4Tc) ≈ 3.11m/s（满足水速<3m/s的要求）
 *
 *  粗过滤（|v|<5m/s）：
 *    去除因相位噪声引起的极端异常值，5m/s略大于最大期望值3.11m/s
 * ============================================================ */
static float calc_WaterVelocity(const float *phase_up_seq, int N)
{
    if (N < 2) return 0.0f;
    float vel_sum = 0.0f;
    int   valid   = 0;
 
    for (int i = 1; i < N; i++) {
        float dp = phase_up_seq[i] - phase_up_seq[i - 1];
        /* 相位展开：限制在[-π, π] */
        while (dp > PI) dp -= 2.0f * PI;
        while (dp < -PI) dp += 2.0f * PI;
 
        float v = (float)RADAR_WAVELENGTH*dp/(4.0f*PI*(float)RADAR_WATER_PHASE_DT_S);
        if (fabsf(v) < 3.5f) {
            vel_sum += v;
            valid++;
        }
    }
    return (valid > 0) ? (vel_sum / (float)valid) : 0.0f;
}
 
/* ============================================================
 *  DSP_CarRangeVelocity：三角波对联立方程解车距和车速
 *
 *  物理方程（FMCW三角波）：
 *   发射信号f_Tx(t)=f_start+(B/Tc)*t，t∈[0,Tc]
 *   τ=2R/c, f_D=-2v/λ=-2v·fc/c （远离时频率降低）
 *   目标回波f_Rx(t)=f_Tx(t-τ)+f_d=f_start+(B/Tc)*(t-τ)+f_D, 
 *   拍频f_beat=|f_Tx(t)-f_Rx(t)|=|(B/Tc)×τ+f_D|=|(2B/cTc)×R+(2fc/c)×v|
 * 
 *    上扫拍频：f_up   =(2B/cTc)×R+(2fc/c)×v
 *    下扫拍频：f_down =(2B/cTc)×R-(2fc/c)×v
 *    （下扫斜率为负，速度项符号翻转）
 *
 *  联立求解：
 *    R=c×Tc×(f_up+f_down)/(4B)
 *    v=c×(f_up-f_down)/(4×fc)
 *
 *  有效性过滤：
 *    R>0.5m（太近可能是雷达自身反射）
 *    R<50m（超出合理测量范围）
 *    |v|<40m/s（约144km/h，超出则认为解算错误）
 *
 *  注意：三角波单对最大不模糊速度=λ/(4Tc)≈3.11m/s≈11.2km/h
 *    测量超速车辆时结果会模糊，需要缩短Tc或多对解模糊
 * ============================================================ */
static bool calc_CarRangeVelocity(const IQ_Sample_t *iq_up,const IQ_Sample_t *iq_dn,
								float *range_m, float *velocity_mps)
{
    static float fft_up[2 * FFT_SIZE], fft_dn[2 * FFT_SIZE];
    static float mag_up[FFT_SIZE / 2], mag_dn[FFT_SIZE / 2];
 
    run_range_fft(iq_up, CHIRP_UP,   mag_up, fft_up);
    run_range_fft(iq_dn, CHIRP_DOWN, mag_dn, fft_dn);
 
    int bin_up = cfar_detect(mag_up, FFT_SIZE/2);
    int bin_dn = cfar_detect(mag_dn, FFT_SIZE/2);
    if (bin_up < 0 || bin_dn < 0) return false;
 
    /* bin → 拍频（Hz） */
    float f_res = (float)ADC_FS_HZ / (float)FFT_SIZE;
    float f_up  = refine_peak(mag_up, bin_up, FFT_SIZE / 2)*f_res;
    float f_dn  = refine_peak(mag_dn, bin_dn, FFT_SIZE / 2)*f_res;
 
    /* 联立方程解R和v */
    float R = (float)SPEED_OF_LIGHT*(float)RADAR_TC_S*(f_up + f_dn)/(4.0f*(float)RADAR_BW_HZ);
    float v = (float)SPEED_OF_LIGHT*(f_up-f_dn)/(4.0f*(float)RADAR_FC_HZ);
 
	if (R<=0.5f||R>=50.0f||fabsf(v)>=40.0f) return false;
    *range_m = R;
	*velocity_mps=v;
    return true;
}

/* ============================================================
 *  Radar_MeasureFrame
 *
 *  帧内各阶段的chirp调度：
 *    每次Chirp_Start()设好方向、等PLL锁定、启动DMA和TIM2
 *    Chirp_WaitDone()阻塞直到79步全部完成（TIM2停止、done=true）
 *    然后调用DSP处理g_chirp.iq_buf里的数据
 *    下一次Chirp_Start()会重置状态，可以安全复用iq_buf
 * ============================================================ */
void Radar_MeasureFrame(RadarFrame_t *frame)
{
    memset(frame, 0, sizeof(RadarFrame_t));

    /* ====================================================
     *  阶段1：水位（单次上扫chirp）
     *  只需要一次上扫，Range-FFT找水面距离峰
     * ==================================================== */
    Chirp_Start(CHIRP_UP);
    Chirp_WaitDone();
	if(calc_WaterLevel(g_chirp.iq_buf ,&frame->water_level_m,&frame->water_level_snr)) frame->water_level_valid=true;

    /* ====================================================
     *  阶段2：水速（N_CHIRPS_WATER_V对三角波，相位差分法）
     *
     *  每对执行上扫+下扫，从上扫提取水面峰的复数系数
     *  积累N个相位值后做差分估计速度
     *
     *  为什么要配套下扫：
     *    保持三角波波形，让PLL在chirp间隔内有对称的频率跳变
     *    实际上水速计算只用上扫相位，下扫数据在这里丢弃
     * ==================================================== */
    float phase_up[N_CHIRPS_WATER_V];
    int phase_valid = 0;

    for (int i = 0; i < N_CHIRPS_WATER_V; i++) {
        float re, im, rng;

        /* 上扫：提取水面距离峰的复数 → 计算相位 */
        Chirp_Start(CHIRP_UP);
        Chirp_WaitDone();

        if(ExtractPeakComplex(g_chirp.iq_buf, CHIRP_UP, &re, &im, &rng)) {
            phase_up[phase_valid] = atn2f(im, re);   /* 相位范围 [-π, π] */
            phase_valid++;
        } 

        /* 下扫：维持三角波结构，数据暂不使用 */
        Chirp_Start(CHIRP_DOWN);
        Chirp_WaitDone();
    }
    if (phase_valid >= 2) {
        frame->water_velocity_mps = calc_WaterVelocity(phase_up, phase_valid);
        frame->water_velocity_valid = true;
    } else {
        frame->water_velocity_mps = 0.0f;
        frame->water_velocity_valid = false;
    }

    /* ====================================================
     *  阶段3：车速/车距（N_CHIRPS_CAR对三角波，联立方程法）
     *
     *  每对：上扫保存 → 下扫处理 → 联立解R和v
     *  多对结果取平均，抑制单次噪声
     *
     *  注意：最大不模糊速度≈11.2km/h，测量更快目标时结果会模糊
     * ==================================================== */
    // static IQ_Sample_t iq_up_buf[ADC_SAMPLES_PER_CHIRP];
    // float sum_range = 0.0f, sum_vel = 0.0f;
    // int   valid_pairs = 0;

    // for (int i = 0; i < N_CHIRPS_CAR; i++) {
    //     /* 上扫：保存IQ数据 */
    //     Chirp_Start(CHIRP_UP);
    //     Chirp_WaitDone();
    //     memcpy(iq_up_buf, g_chirp.iq_buf, sizeof(iq_up_buf));
    //     /* 下扫：立刻处理，与刚保存的上扫联立 */
    //     Chirp_Start(CHIRP_DOWN);
    //     Chirp_WaitDone();

	// 	float R=0.0f, V=0.0f;
    //     if(calc_CarRangeVelocity(iq_up_buf, g_chirp.iq_buf, &R, &V)){
	// 		sum_range +=R;
    //         sum_vel   +=V;
    //         valid_pairs++;
	// 	}
    // }
    // if (valid_pairs > 0) {
    //     frame->car_range_m      = sum_range/(float)valid_pairs;
    //     frame->car_velocity_mps = sum_vel/(float)valid_pairs;
    //     frame->car_valid        = true;
    // }
}
