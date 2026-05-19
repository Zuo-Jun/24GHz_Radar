/**
 ******************************************************************************
 * @file    pga113.c
 * @brief   PGA113 零漂移可编程增益放大器驱动实现
 *          针对 STM32F405RGT6 (168MHz) 优化，单工只写模式
 *
 * @SPI 时序实现细节
 *
 *  PGA113 使用 SPI Mode 0,0（CPOL=0，CPHA=0）：
 *    - SCLK 空闲为低电平
 *    - DIO 数据在 SCLK 低电平期间准备（MCU输出），SCLK 上升沿被芯片采样
 *    - 每次传输固定 16 位（MSB 先发），CS 上升沿时数据锁存生效
 *    - 若 CS 低电平期间时钟数不是 16 的整数倍，芯片不响应任何操作
 *      （此特性可用于 SPI 接口同步复位）
 *
 *  时序图（SPI Mode 0,0）：
 *
 *  CS   ─┐                               ┌─────
 *        └───────────────────────────────┘
 *              ↑tCSSC              ↑tSCCS  ↑tCSH
 *
 *  SCLK  ___┌─┐ ┌─┐ ┌─┐ ... ┌─┐___
 *           └─┘ └─┘ └─┘     └─┘
 *           ↑tLO↑tHI（各≥100ns）
 *
 *  DIO  ──X D15 X D14 X ... X D0 X──
 *           ↑数据在SCLK低时变化，上升沿采样
 *
 *  NOP延时分析（STM32F405 @ 168MHz）：
 *    单次NOP          ≈ 6ns
 *    GPIO BSRR写入    ≈ 12~18ns（2~3个AHB周期）
 *    I/O实际翻转      ≈ 6ns
 *    单次GPIO操作合计 ≈ 18~24ns
 *
 *  各时序余量验证：
 *    tSU  (≥10ns)  = GPIO操作本身 ≥ 18ns，已满足，无需额外NOP
 *    tHD  (≥10ns)  = GPIO操作本身 ≥ 18ns，已满足，无需额外NOP
 *    tHI  (≥100ns) = SCLK_HIGH后 HALF_CLK_DELAY(16×6=96ns) + 下一个GPIO(18ns) ≥ 114ns ✓
 *    tLO  (≥100ns) = SCLK_LOW后  HALF_CLK_DELAY(16×6=96ns) + 下一个GPIO(18ns) ≥ 114ns ✓
 *    tCSSC(≥10ns)  = CS_LOW后    CS_HOLD_DELAY(4×6=24ns)   + GPIO(18ns)       ≥ 42ns  ✓
 *    tSCCS(≥10ns)  = 最后SCLK边沿后 CS_HOLD_DELAY ≥ 42ns                              ✓
 *    tCSH (≥40ns)  = CS_HIGH后   CS_HOLD_DELAY(4×6=24ns)   + GPIO(18ns)       ≥ 42ns  ✓
 *
 * @版本    V2.0
 * @日期    2026-03
 ******************************************************************************
 */

#include "pga113.h"

/* ============================================================
 * 私有函数：发送16位命令字
 * ============================================================ */

/**
 * @brief  通过软件 SPI 发送16位命令（MSB 先发）
 * @param  cmd  要发送的16位命令字
 *
 * @时序说明（SPI Mode 0,0）：
 *   每个 bit 的操作序列：
 *     1. 在 SCLK 低电平期间将当前 bit 输出到 DIO
 *        （GPIO操作≈18~24ns，满足 tSU ≥ 10ns）
 *     2. 延时半个时钟周期（HALF_CLK_DELAY ≈ 96ns）
 *        确保 SCLK 低电平持续 ≥ 100ns（tLO 要求）
 *     3. SCLK 拉高（上升沿，芯片在此采样 DIO）
 *        DIO 数据保持到 SCLK 上升沿后（GPIO操作≈18ns，满足 tHD ≥ 10ns）
 *     4. 延时半个时钟周期（HALF_CLK_DELAY ≈ 96ns）
 *        确保 SCLK 高电平持续 ≥ 100ns（tHI 要求）
 *     5. SCLK 拉低，准备下一个 bit
 *
 *   完整 SCLK 周期 ≈ 2×(18ns + 96ns) = 228ns，频率 ≈ 4.4MHz < 10MHz ✓
 *
 * @note   调用此函数前 CS 必须已为低电平
 *         此函数不操作 CS，CS 由调用者负责
 */
static void PGA113_Write16(uint16_t cmd)
{
    uint8_t i;

    for (i = 0U; i < 16U; i++)
    {
        /*
         * Step 1：SCLK 处于低电平期间，输出当前 bit（MSB 先发）
         *   DIO 在此变化，直到 SCLK 上升沿后保持（tSU + tHD 均由GPIO操作满足）
         */
        if ((cmd & 0x8000U) != 0U)
        {
            PGA113_DIO_HIGH();
        }
        else
        {
            PGA113_DIO_LOW();
        }
        cmd <<= 1U;

        /*
         * Step 2：SCLK 低电平保持延时
         *   目的：满足 tLO ≥ 100ns（手册 Note3：不小于 1/fSCLK_max）
         *   GPIO写入DIO的操作已耗时≈18ns，
         *   再加 HALF_CLK_DELAY(96ns)，总低电平时间 ≥ 114ns ✓
         */
        PGA113_HALF_CLK_DELAY();

        /*
         * Step 3：SCLK 上升沿
         *   芯片在此边沿采样 DIO 数据
         *   DIO 在上升沿后保持（下一个 GPIO 操作前），满足 tHD ≥ 10ns
         */
        PGA113_SCLK_HIGH();

        /*
         * Step 4：SCLK 高电平保持延时
         *   目的：满足 tHI ≥ 100ns
         *   SCLK_HIGH写入≈18ns，再加 HALF_CLK_DELAY(96ns)，总高电平 ≥ 114ns ✓
         */
        PGA113_HALF_CLK_DELAY();

        /*
         * Step 5：SCLK 下降沿，准备下一个 bit
         */
        PGA113_SCLK_LOW();

        /* 注意：SCLK_LOW 到下一次 DIO 输出之间无需额外延时，
         *       因为下一次循环的 GPIO 操作本身已满足 tSU ≥ 10ns */
    }
    /* 循环结束时 SCLK 为低电平，符合 SPI Mode 0,0 空闲状态 */
}

void PGA113_Init(void)
{ 
    /*   等待PGA113完成上电复位（POR）
     *   手册：POR power-up time=40μs（DVDD≥2V 后）
     *   此处等待 1ms，远大于40μs，确保可靠*/
    HAL_Delay(1U);

    /* 发送NOP，复位SPI接口到已知状态
     *   手册（Section 8.6.1）：
     *   "This condition provides a way to quickly reset the SPI interface
     *    to a known starting condition for data synchronization."
     *   发送完整16位NOP（0x0000）使接口同步
     */
    PGA113_CS_LOW();
    PGA113_CS_HOLD_DELAY();           /* tCSSC ≥ 10ns */
    PGA113_Write16(PGA113_CMD_NOP);   /* 发送16位 NOP */
    PGA113_CS_HOLD_DELAY();           /* tSCCS ≥ 10ns */
    PGA113_CS_HIGH();
    PGA113_CS_HOLD_DELAY();           /* tCSH  ≥ 40ns */

    /*   写入初始配置
     *   默认使用CH1，增益×1（可根据实际测试结果调整）
     *   两片PGA113共用CS，此次写入同时配置两片
     */
    PGA113_SetGainChannel(PGA113_GAIN_1, PGA113_CH_CH1);
}

/**
 * @brief  设置增益和输入通道
 * @param  gain    增益选择
 * @param  channel 通道选择
 *
 * @命令字构造（手册 Table 3）：
 *   cmd = 0x2A00 | ((gain & 0x07) << 4) | (channel & 0x0F)
 *
 *   例：增益×20（0x04），通道CH1（0x01）
 *   cmd = 0x2A00 | (0x04 << 4) | 0x01 = 0x2A41
 *   二进制：0010 1010 | 0100 0001
 *           ^^^^^^^^   ^^^^ ^^^^
 *           写标识符   G3-G0 CH3-CH0
 *
 * @SPI 传输过程：
 *   CS 拉低 → 延时(tCSSC) → 发送16位命令 → 延时(tSCCS) → CS 拉高（锁存）→ 延时(tCSH)
 *
 * @note   两片 PGA113 共用同一 CS，调用一次同时配置两片
 *         增益/通道切换完成时间：200ns（手册 Electrical Characteristics）
 */
void PGA113_SetGainChannel(PGA113_Gain_t gain, PGA113_Channel_t channel)
{
    /* 构造16位写命令字 */
    uint16_t cmd = (uint16_t)(PGA113_CMD_WRITE_BASE
                              | (((uint16_t)gain    & 0x07U) << 4U)
                              |  ((uint16_t)channel & 0x0FU));

    /* CS 拉低，开始 SPI 传输 */
    PGA113_CS_LOW();
    PGA113_CS_HOLD_DELAY();    /* 满足 tCSSC ≥ 10ns（CS↓到第一个SCLK边沿）*/

    /* 发送16位命令（MSB先发，时序由 PGA113_Write16 保证）*/
    PGA113_Write16(cmd);

    /* 满足 tSCCS ≥ 10ns（最后SCLK边沿到CS↑）*/
    PGA113_CS_HOLD_DELAY();

    /* CS 拉高，命令在此上升沿锁存生效 */
    PGA113_CS_HIGH();

    /* 满足 tCSH ≥ 40ns（CS高电平保持时间）*/
    PGA113_CS_HOLD_DELAY();
    PGA113_CS_HOLD_DELAY();

    /*
     * 等待增益/通道切换完成
     * 手册：channel/gain select time = 0.2μs = 200ns
     * CS_HOLD_DELAY × 2 ≈ 84ns，不足200ns
     * 此处用 NOP 补足（共约33个NOP ≈ 200ns）
     * 实际中 HAL 调用和函数调用开销通常已超过 200ns，可酌情省略
     */
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); /* 33 × 6ns ≈ 198ns，满足 200ns 切换时间 */
}

/**
 * @brief  进入软件关断模式
 *
 * @命令字（手册 Table 3）：
 *   SDN_EN = 0xE1F1
 *   D15..D8 = 1110 0001
 *   D7..D0  = 1111 0001
 *
 * @关断特性：
 *   - 关断总电流（模拟+数字）≤ 4μA（SCLK 空闲，典型值）
 *   - VOUT 在 2μs 后进入高阻态
 *   - RF 和 RI 在关断期间仍连接于 VOUT 与 VREF 之间
 *   - 唤醒：发送 SDN_DIS（0xE100）或任意有效写命令
 */
void PGA113_Shutdown(void)
{
    PGA113_CS_LOW();
    PGA113_CS_HOLD_DELAY();
    PGA113_Write16(PGA113_CMD_SDN_EN); /* 发送 0xE1F1 */
    PGA113_CS_HOLD_DELAY();
    PGA113_CS_HIGH();
    PGA113_CS_HOLD_DELAY();
    PGA113_CS_HOLD_DELAY();

    /*
     * 等待关断生效：Disable time = 2μs（VOUT 进入高阻态）
     * 约 2000ns / 6ns = 334个NOP，此处用 HAL_Delay 替代（精度足够）
     * 实际上 PGA113_Shutdown() 调用后通常不会立即操作 VOUT，
     * 函数调用开销本身已接近 2μs，此延时可按需保留或省略
     */
    /* delay_us(3); */ /* 如有 us 级延时函数，推荐替换此处 */
    volatile uint32_t cnt = 350U;
    while (cnt-- > 0U) { __NOP(); }
}

/**
 * @brief  退出软件关断，恢复上一次有效配置
 *
 * @命令字（手册 Table 3）：
 *   SDN_DIS = 0xE100
 *
 * @唤醒特性：
 *   - Enable time = 4μs（从关断到正常工作）
 *   - 恢复到最近一次有效写操作的增益和通道配置
 *   - 若从未写过，则恢复 POR 默认：增益=1，通道=VCAL/CH0
 */
void PGA113_WakeUp(void)
{
    PGA113_CS_LOW();
    PGA113_CS_HOLD_DELAY();
    PGA113_Write16(PGA113_CMD_SDN_DIS); /* 发送 0xE100 */
    PGA113_CS_HOLD_DELAY();
    PGA113_CS_HIGH();
    PGA113_CS_HOLD_DELAY();
    PGA113_CS_HOLD_DELAY();

    /*
     * 等待唤醒完成：Enable time = 4μs
     * 约 4000ns / 6ns = 667个NOP
     */
    /* delay_us(5); */ /* 如有 us 级延时函数，推荐替换此处 */
    volatile uint32_t cnt = 700U;
    while (cnt-- > 0U) { __NOP(); }
}
