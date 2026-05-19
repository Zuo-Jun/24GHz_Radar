/**
 ******************************************************************************
 * @file    pga113.h
 * @brief   PGA113 零漂移可编程增益放大器驱动头文件
 *          针对 STM32F405RGT6 (168MHz) 优化，单工只写模式
 *
 * @硬件平台
 *   MCU   : STM32F405RGT6
 *   SYSCLK: 168 MHz
 *   单次 NOP 指令耗时：1/168MHz ≈ 6 ns
 *
 * @芯片说明
 *   PGA113 是 Texas Instruments 生产的零漂移可编程增益放大器，
 *   本项目使用两片 PGA113（均为 PGA113AIDGSR，VSSOP-10 封装），
 *   共用同一组软件模拟 SPI 总线（SCLK / DIO / CS），
 *   片选信号相同，两片配置相同，协同将中频信号放大至
 *   ADC 最佳采样范围。
 *
 *   芯片特性（Scope Gains）：
 *     - 增益：×1 / ×2 / ×5 / ×10 / ×20 / ×50 / ×100 / ×200
 *     - 2路模拟输入 MUX + 4路内部校准通道
 *     - 输入失调电压典型值 ±25 μV，零漂 0.35 μV/°C
 *     - 3线 SPI（SCLK / DIO / CS），SPI Mode 0,0
 *     - 软件关断，关断电流 ≤ 4 μA
 *
 * @在板作用
 *   两片 PGA113 协同对来自射频前端的中频模拟信号进行可编程放大，
 *   确保信号幅度匹配 ADC 的最优采样范围，提高有效分辨率和 SNR。
 *   增益在初始化时一次性写入，运行期间保持不变。
 *
 * @引脚连接
 *   PGA113 Pin   | MCU 引脚 | 功能说明
 *   -------------|----------|----------------------------------
 *   CS   (Pin 9) | PB11     | 片选（低有效，上升沿锁存命令）
 *   SCLK (Pin 7) | PB9      | SPI 时钟（软件模拟）
 *   DIO  (Pin 8) | PB8      | 数据输入（仅写，固定为推挽输出）
 *   AVDD (Pin 1) | +5V      | 模拟电源，0.1μF 陶瓷电容旁路至GND
 *   DVDD (Pin10) | +3.3V    | 数字电源，0.1μF 陶瓷电容旁路至GND
 *   GND  (Pin 6) | GND      | 地
 *   VREF (Pin 4) | GND      | 输出参考（接GND，单端配置）
 *   VOUT (Pin 5) | ADC输入  | 放大后的中频模拟输出
 *   VCAL/CH0 (P3)| 校准基准 | 建议接 ADC 参考电压
 *   CH1  (Pin 2) | 中频信号 | 模拟输入通道1
 *
 * @GPIO 配置要求（在 CubeMX 或手动初始化中设置）
 *   CS   (PB11)：GPIO_MODE_OUTPUT_PP，GPIO_SPEED_FREQ_VERY_HIGH，初始 高电平
 *   SCLK (PB9) ：GPIO_MODE_OUTPUT_PP，GPIO_SPEED_FREQ_VERY_HIGH，初始 低电平
 *   DIO  (PB8) ：GPIO_MODE_OUTPUT_PP，GPIO_SPEED_FREQ_VERY_HIGH，初始 低电平
 *                单工只写，DIO 全程固定为推挽输出，无需切换方向
 *                芯片内部 DIO 有 10μA 下拉电流源，无需外部下拉电阻
 *   注：GPIO_SPEED_FREQ_VERY_HIGH 对应 STM32F4 的 100MHz I/O速度
 *
 *   STM32F405 GPIO 操作耗时（VERY_HIGH速度，AHB总线）：
 *     GPIO BSRR寄存器写入：约2~3个AHB周期≈12~18ns
 *     I/O实际电平翻转（含压摆）：≈6ns（轻负载）
 *     合计单次GPIO操作：≈18~24ns
 *
 *   关于tHI/tLO的手册说明（Note 3）：
 *     原文："tHI and tLO must not be less than 1/SCLK(maximum)"
 *     即tHI/tLO≥1/10MHz=100ns
 *     这是SCLK占空比不超过50%@10MHz 的约束，
 *     软件SPI中每个电平状态都需要持续至少100ns。
 *
 * @单工只写模式说明
 *   本驱动仅实现写操作，不实现ReadBack。
 *   SPI总线与ADF4153A等芯片共用，ADF4153A需频繁修改寄存器以改变输出频率，
 *   PGA113增益在初始化时一次性写入即可。两片PGA113共用同一CS，写操作会同时配置两片。
 *******************************************************************************/

#ifndef __PGA113_H
#define __PGA113_H

#include "stm32f4xx_hal.h"   

#define PGA113_CS_GPIO_PORT     GPIOB
#define PGA113_CS_GPIO_PIN      GPIO_PIN_11   /* PB11 — 片选 CS */

#define PGA113_SCLK_GPIO_PORT   GPIOB
#define PGA113_SCLK_GPIO_PIN    GPIO_PIN_9    /* PB9  — 时钟 SCLK */

#define PGA113_DIO_GPIO_PORT    GPIOB
#define PGA113_DIO_GPIO_PIN     GPIO_PIN_8    /* PB8  — 数据 DIO（只写）*/

/* ============================================================
 * GPIO快速操作宏（直接写BSRR寄存器，原子操作，比HAL更快）
 *   BSRR[15:0]=置位（SET）
 *   BSRR[31:16]=清零（RESET）
 * ============================================================ */
#define PGA113_CS_LOW()    (PGA113_CS_GPIO_PORT->BSRR   = (uint32_t)PGA113_CS_GPIO_PIN   << 16U)
#define PGA113_CS_HIGH()   (PGA113_CS_GPIO_PORT->BSRR   = (uint32_t)PGA113_CS_GPIO_PIN)
#define PGA113_SCLK_LOW()  (PGA113_SCLK_GPIO_PORT->BSRR = (uint32_t)PGA113_SCLK_GPIO_PIN << 16U)
#define PGA113_SCLK_HIGH() (PGA113_SCLK_GPIO_PORT->BSRR = (uint32_t)PGA113_SCLK_GPIO_PIN)
#define PGA113_DIO_LOW()   (PGA113_DIO_GPIO_PORT->BSRR  = (uint32_t)PGA113_DIO_GPIO_PIN  << 16U)
#define PGA113_DIO_HIGH()  (PGA113_DIO_GPIO_PORT->BSRR  = (uint32_t)PGA113_DIO_GPIO_PIN)

/* ============================================================
 * SPI 时序延时宏（STM32F405 @ 168MHz，单次NOP ≈ 6ns）
 *
 * PGA113_HALF_CLK_DELAY()：
 *   用途：SCLK 高/低电平保持期间（tHI / tLO）
 *   要求：≥ 100ns（手册 Note3：不小于 1/fSCLK_max）
 *   实现：16个NOP×6ns = 96ns，加上GPIO写操作≈18ns，合计≥114ns ✓
 *
 * PGA113_CS_HOLD_DELAY()：
 *   用途：CS 相关时序（tCSH / tCSSC / tSCCS）
 *   要求：tCSH ≥ 40ns，tCSSC/tSCCS ≥ 10ns
 *   实现：4个NOP×6ns = 24ns，加上GPIO写操作≈18ns，合计≈42ns ✓
 * ============================================================ */

/** SCLK 半周期延时，保证 tHI/tLO ≥ 100ns */
#define PGA113_HALF_CLK_DELAY() do {    \
    __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); \
} while(0)

/** CS 建立/保持延时，满足 tCSH/tCSSC/tSCCS */
#define PGA113_CS_HOLD_DELAY() do {     \
    __NOP(); __NOP(); __NOP(); __NOP(); \
} while(0)

#define PGA113_CMD_WRITE_BASE   0x2A00U   /* 写命令基地址，低8位填入G[3:0]|CH[3:0] */
#define PGA113_CMD_NOP          0x0000U   /* NOP，用于SPI同步复位  */
#define PGA113_CMD_SDN_DIS      0xE100U   /* 退出软件关断（恢复上次配置） */
#define PGA113_CMD_SDN_EN       0xE1F1U   /* 进入软件关断（低功耗） */

typedef enum {
    PGA113_GAIN_1   = 0x00U,  
    PGA113_GAIN_2   = 0x01U,  
    PGA113_GAIN_5   = 0x02U,  
    PGA113_GAIN_10  = 0x03U,  
    PGA113_GAIN_20  = 0x04U,  
    PGA113_GAIN_50  = 0x05U,  
    PGA113_GAIN_100 = 0x06U,  
    PGA113_GAIN_200 = 0x07U,  
} PGA113_Gain_t;

typedef enum {
    PGA113_CH_VCAL_CH0 = 0x00U,  /* CH0/VCAL校准基准输入    */
    PGA113_CH_CH1      = 0x01U,  /* CH1信号输入（中频信号）  */
    PGA113_CH_CAL1     = 0x0CU,  /* 内部校准：接GND         */
    PGA113_CH_CAL2     = 0x0DU,  /* 内部校准：接0.9×VCAL    */
    PGA113_CH_CAL3     = 0x0EU,  /* 内部校准：接0.1×VCAL    */
    PGA113_CH_CAL4     = 0x0FU,  /* 内部校准：接VREF        */
} PGA113_Channel_t;

/**
 * @brief  初始化PGA113 GPIO，复位SPI接口，写入默认配置
 * @note   调用前需在MX_GPIO_Init中完成GPIOB时钟使能
 *         函数内部等待1ms确保POR（40μs）完成
 *        上电后芯片默认状态：增益=1，通道=VCAL/CH0 */
void PGA113_Init(void);

/**
 * @brief  设置增益和输入通道
 * @param  gain    增益，见PGA113_Gain_t
 * @param  channel 通道，见PGA113_Channel_t
 * @note   两片PGA113共用CS，调用一次同时配置两片 */
void PGA113_SetGainChannel(PGA113_Gain_t gain, PGA113_Channel_t channel);

/**
 * @brief  使PGA113进入软件关断模式（低功耗）
 * @note   关断电流≤4μA（典型值）
 *         唤醒方式：发送SDN_DIS命令或任意有效写命令 */
void PGA113_Shutdown(void);

/* 退出软件关断模式，恢复上一次有效配置 */
void PGA113_WakeUp(void);

#endif 
