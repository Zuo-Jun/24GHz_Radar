#include "ADF4153A.h"

/* -------- 影子寄存器，只保留R2 -------- */
uint32_t ADF4153A_R2_Shadow = 0U;

/* 模块级缓存Init时的不变参数，避免SetFrequency时重传整个cfg */
static ADF_MuxoutMode_t s_muxout    = MUXOUT_DLOCK_DIGITAL;
static uint8_t          s_prescaler = 1U;
static uint8_t          s_R         = 1U;
static uint8_t          s_resync_en = 0U;

/* Init完成后由ADF4153A_Init内部调用，更新缓存 */
static void CacheR1Params(const ADF4153A_Config_t *cfg)
{
    s_muxout    = cfg->muxout;
    s_prescaler = cfg->prescaler;
    s_R         = cfg->R;
    s_resync_en = cfg->resync_en;
}

/* ============================================================
 *  ADF4153A软件SPI实现
 *
 *  引脚：PB8=CLK，PB9=DATA(MOSI)，LE=PA4
 *  时序依据：ADF4153A手册 Figure 2 / Table 2
 *
 *  协议规则：
 *    1. LE默认高电平（空闲态）
 *    2. LE拉低，至少等t1=20ns，才能开始发CLK
 *    3. DATA在CLK上升沿前至少t2=10ns就要稳定（setup time）
 *    4. DATA在CLK上升沿后至少保持t3=10ns（hold time）
 *    5. 数据从MSB（DB23）开始，LSB（DB0）最后发
 *    6. 数据在CLK上升沿被锁入移位寄存器
 *    7. 24位全部发完，等t6=10ns后LE拉高
 *    8. LE上升沿把移位寄存器内容载入对应寄存器（DB1:DB0决定R0~R3）
 *    9. LE高电平至少保持t7=20ns
 *
 *  STM32F405@168MHz：
 * 	  一周期=1/168MHz≈6ns
 *    一条GPIO操作指令约6~8ns
 *    一个__NOP()约6ns
 *    HAL_Delay(1)约1ms（用于1ms级等待）
 *    所有时序要求最小10~25ns，GPIO操作本身就满足，无需额外插NOP
 * ============================================================ */
static void ADF_SPI_Write24(uint32_t word24)
{
    /* LE拉低，开始一次24bit传输，等t1 */
    ADF_LE_LOW();
    __NOP(); __NOP(); __NOP();  

    for (int8_t bit = 23; bit >= 0; bit--) 
	{
        if ((word24 >> bit) & 0x1U) ADF_DATA_HIGH();
        else ADF_DATA_LOW();
		/* 在CLK上升沿之前，DATA必须稳定t2≥10ns */
        __NOP(); __NOP();  

        ADF_CLK_HIGH();
		 /* t4（CLK高电平时间）≥25ns，t3（DATA保持时间）≥10ns：
          * t3完全包含在t4内，DATA在整个CLK高电平期间保持不变*/
        __NOP(); __NOP(); __NOP(); __NOP();  
        ADF_CLK_LOW();
        /* CLK低电平t5≥25ns（当前bit的t5=下一bit的DATA操作+NOP） */
    }
    /* 等t6≥10ns（最后一个CLK下降沿到LE上升沿） */
    __NOP(); __NOP();  
    /* LE上升沿：数据载入寄存器 */
    ADF_LE_HIGH();
    /* 保持LE高电平t7≥20ns，让芯片完成锁存 */
    __NOP(); __NOP(); __NOP(); 
}

/* ============================================================
 *  内部寄存器构建（static，外部不可见）
 *  R0 位域：
 *    [23]      FASTLOCK
 *    [22:14]   INT[8:0]
 *    [13:2]    FRAC[11:0]
 *    [1:0]     00
 * ============================================================ */
static uint32_t BuildR0(uint8_t fastlock, uint32_t INT, uint32_t FRAC)
{
    return (((uint32_t)fastlock  & 0x1U)   << 23)
         | ((INT                 & 0x1FFU) << 14)
         | ((FRAC                & 0xFFFU) <<  2)
         | 0x0U;
}

/* R1 位域：
 *    [23]      LOAD_CTRL
 *    [22:20]   MUXOUT[2:0]
 *    [19]      0（保留）
 *    [18]      PRESCALER
 *    [17:14]   R[3:0]
 *    [13:2]    MOD[11:0]
 *    [1:0]     01
 */
static uint32_t BuildR1(uint8_t load_ctrl, ADF_MuxoutMode_t muxout,
                         uint8_t prescaler, uint8_t R, uint32_t MOD)
{
    return (((uint32_t)load_ctrl & 0x1U) << 23)
         | (((uint32_t)muxout    & 0x7U) << 20)
         /* DB19 保留，写0 */
         | (((uint32_t)prescaler & 0x1U) << 18)
         | (((uint32_t)R         & 0xFU) << 14)
         | ((MOD                 & 0xFFFU) <<  2)
         | 0x1U;
}

/* R2 位域：
 *    [15:12]   RESYNC[3:0]
 *    [11]      REFIN_DBL
 *    [10:7]    CP[3:0]
 *    [6]       PD_POL
 *    [5]       LDP
 *    [4]       POWER_DOWN
 *    [3]       CP_3STATE
 *    [2]       CNT_RESET
 *    [1:0]     10
 */
static uint32_t BuildR2(uint8_t resync, uint8_t refin_doubler,
                         uint8_t cp_current, uint8_t pd_polarity,
                         uint8_t ldp, uint8_t power_down,
                         uint8_t cp_three_state, uint8_t counter_reset)
{
    return (((uint32_t)resync         & 0xFU) << 12)
         | (((uint32_t)refin_doubler  & 0x1U) << 11)
         | (((uint32_t)cp_current     & 0xFU) <<  7)
         | (((uint32_t)pd_polarity    & 0x1U) <<  6)
         | (((uint32_t)ldp            & 0x1U) <<  5)
         | (((uint32_t)power_down     & 0x1U) <<  4)
         | (((uint32_t)cp_three_state & 0x1U) <<  3)
         | (((uint32_t)counter_reset  & 0x1U) <<  2)
         | 0x2U;
}

/* R3 位域：
 *    [10]      0（保留，必须写0）
 *    [9:6]     T8..T5（5位噪声值的高4位）
 *    [5:3]     0（保留，必须写0）
 *    [2]       T1（5位噪声值的低1位）
 *    [1:0]     11
 */
static uint32_t BuildR3(ADF_NoiseSpur_t noise_spur)
{
    uint8_t ns = (uint8_t)noise_spur & 0x1FU;
    return ((uint32_t)((ns >> 1) & 0xFU) <<  6) 
         | ((uint32_t)( ns       & 0x1U) <<  2)  
         | 0x3U;
}

/* ============================================================
 *  ADF4153A_Init
 *  严格按手册 Page17 "Initialization Sequence" 六步执行：
 *    1. 写R3全零（清除测试模式）
 *    2. 写R3（选择噪声/毛刺模式）
 *    3. 写R2，CNT_RESET=1（保持计数器复位，配置其他参数）
 *    4. 写R1（MOD, R, MUXOUT, PRESCALER）
 *    5. 写R0（INT, FRAC）
 *    6. 写R2，CNT_RESET=0（释放复位，PLL开始锁定）
 * ============================================================ */
void ADF4153A_Init(const ADF4153A_Config_t *cfg)
{
    uint32_t r1, r2;

    /* 步骤1：R3全零清除 */
    ADF_SPI_Write24(0x000003U);

    /* 步骤2：R3写入噪声模式 */
    ADF_SPI_Write24(BuildR3(cfg->noise_spur));

    /* 步骤3：R2，CNT_RESET=1，配置其余所有控制位 */
    r2 = BuildR2(
        cfg->resync,
        cfg->refin_doubler,
        cfg->cp_current,
        cfg->pd_polarity,
        cfg->ldp,
        0U,   /* power_down=0 */
        0U,   /* cp_three_state=0 */
        1U    /* counter_reset=1，暂时保持复位 */
    );
    ADF4153A_R2_Shadow = r2;  /* 保存R2影子，供PowerDown使用 */
    ADF_SPI_Write24(r2);

    /* 步骤4：R1，LOAD_CTRL由cfg->resync_en决定 */
    r1 = BuildR1(
        cfg->resync_en,   /* 0=正常；1=装载相位重同步延迟 */
        cfg->muxout,
        cfg->prescaler,
        cfg->R,
        cfg->MOD
    );
    ADF_SPI_Write24(r1);

    /* 步骤5：R0 */
    ADF_SPI_Write24(BuildR0(cfg->fastlock_en, cfg->INT, cfg->FRAC));

    /* 步骤6：R2，CNT_RESET=0，释放复位，PLL开始锁定 */
    r2 = BuildR2(
        cfg->resync,
        cfg->refin_doubler,
        cfg->cp_current,
        cfg->pd_polarity,
        cfg->ldp,
        0U, 0U,
        0U    /* counter_reset=0 */
    );
    ADF4153A_R2_Shadow = r2;  /* 更新影子 */
    ADF_SPI_Write24(r2);

    /* 缓存R1不变参数，供SetFrequency使用 */
    CacheR1Params(cfg);
}

/* ============================================================
 *  ADF4153A_SetFrequency()动态修改频率，只写R1和R0，R2/R3保持不变。
 *
 *  注意MOD双缓冲机制（手册要求）：
 *    修改MOD必须先写R1（新MOD装入缓冲），再写R0（触发生效）。
 *    即使MOD没变也要按此顺序，因此始终先R1后R0。
 * ============================================================ */
void ADF4153A_SetFrequency(uint32_t INT, uint32_t FRAC, uint32_t MOD)
{
    /* 先写R1（将新MOD装入双缓冲），保持其他R1参数不变 */
    ADF_SPI_Write24(BuildR1(s_resync_en, s_muxout, s_prescaler, s_R, MOD));
    /* 再写R0（触发MOD生效并更新INT/FRAC） */
    ADF_SPI_Write24(BuildR0(0U, INT, FRAC));
}

/* ============================================================
 *  ADF4153A_WaitLock
 *  轮询MUXOUT GPIO，等待数字锁定检测拉高
 *  前提：MUXOUT已配置为MUXOUT_DLOCK_DIGITAL
 *  锁定判定：连续24个PFD周期（LDP=0）相位误差<15ns → 拉高
 *  timeout_ms取决于环路带宽，一般取500ms作为安全值，如果500ms内还没锁，说明硬件有问题。
 *  实时调试可以缩短到50ms，如果连续测试都能在50ms内锁定，说明系统工作正常。
 * ============================================================ */
bool ADF4153A_WaitLock(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (HAL_GPIO_ReadPin(ADF_MUXOUT_GPIO_Port, ADF_MUXOUT_Pin) == GPIO_PIN_SET) {
            return true;
        }
        HAL_Delay(1U);
    }
    return false;
}

/* ============================================================
 *  ADF4153A_PowerDown
 *  只修改R2的DB4（POWER_DOWN位），其他位保持不变。
 *  依赖ADF4153A_R2_Shadow记录的当前R2值。
 *  在软件断电模式下，部件仍保留寄存器中所有信息，仅当电源切断时，内容丢失
 * ============================================================ */
void ADF4153A_PowerDown(bool enable)
{
    if (enable) {
        ADF4153A_R2_Shadow |=  (1U << 4);
    } else {
        ADF4153A_R2_Shadow &= ~(1U << 4);
    }
    ADF_SPI_Write24(ADF4153A_R2_Shadow);
}


/* Fpfd = Frefin × (1 + D) / R */
double ADF4153A_CalcFpfd(double refin_hz, uint8_t doubler, uint8_t R)
{
    return refin_hz * (1.0 + (double)doubler) / (double)R;
}

/* Fres = Fpfd / MOD（RF输出端的通道分辨率） */
double ADF4153A_CalcFres(double refin_hz, uint8_t doubler, uint8_t R, uint32_t MOD)
{
    return ADF4153A_CalcFpfd(refin_hz, doubler, R) / (double)MOD;
}

/* ADF RF输入端频率 = Fpfd × (INT + FRAC/MOD)
 * BGT VCO实际频率 = 上述结果 × BGT_PRESCALER_DIV */
double ADF4153A_CalcRFout(double refin_hz, uint8_t doubler, uint8_t R,
                           uint32_t MOD, uint32_t INT, uint32_t FRAC)
{
    double fpfd = ADF4153A_CalcFpfd(refin_hz, doubler, R);
    return fpfd * ((double)INT + (double)FRAC / (double)MOD);
}

