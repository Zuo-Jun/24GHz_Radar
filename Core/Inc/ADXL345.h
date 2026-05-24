/**
 ******************************************************************************
 * @file    ADXL345.h
 * @brief   ADXL345 三轴数字加速度计驱动头文件
 *          针对 STM32F405RGT6 (168MHz) 优化，软件I2C接口
 *
 * @硬件平台
 *   MCU   : STM32F405RGT6
 *   SYSCLK: 168 MHz
 *
 * @芯片说明
 *   ADXL345 是 Analog Devices 生产的低功耗三轴MEMS加速度计，
 *   主要特性：
 *   - 测量范围：±2g/±4g/±8g/±16g（可编程）
 *   - 分辨率：10位（固定），13位（全分辨率模式，±16g时）
 *   - 数据速率：0.1Hz ~ 3200Hz（可编程）
 *   - 接口：I2C（最高400kHz）
 *   - 内置32级FIFO
 *   - 多种中断功能：单击、双击、自由落体、活动/静止检测
 *   - 低功耗：待机模式0.1μA，测量模式40μA@100Hz
 *   - 工作电压：2.0V ~ 3.6V
 *
 * @在板作用
 *   用于检测雷达系统的振动或倾斜角度，可作为系统状态监测
 *   或在移动设备中进行运动检测。
 *
 * @引脚连接（I2C模式）
 *   ADXL345 Pin   | MCU 引脚 | 功能说明
 *   --------------|----------|----------------------------------
 *   VCC           | +3.3V    | 电源
 *   GND           | GND      | 地
 *   CS            | +3.3V    | I2C模式（必须拉高）
 *   SDO           | GND/3.3V | I2C地址选择（GND:0x53, 3.3V:0x1D）
 *   SDA           | PC1(GPIO)| 软件I2C数据线（开漏/上拉）
 *   SCL           | PC0(GPIO)| 软件I2C时钟线（开漏/上拉）
 *   INT1          | 可选     | 中断输出1
 *   INT2          | 可选     | 中断输出2
 *
 * @I2C通信说明
 *   - 使用GPIO软件模拟I2C，不依赖HAL硬件I2C模块
 *   - PC0=SCL、PC1=SDA，均配置为开漏输出并开启上拉，推荐外接4.7kΩ上拉到3.3V
 *   - 7位从地址：0x53（SDO=GND）或 0x1D（SDO=VCC）
 *   - 写操作格式：[START][从地址+W][寄存器地址][数据][STOP]
 *   - 读操作格式：[START][从地址+W][寄存器地址][Sr][从地址+R][数据][STOP]
 *   - 支持多字节读写，地址自动递增
 ******************************************************************************
 */

#ifndef __ADXL345_H
#define __ADXL345_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 *  Data = 最终从DATAX/Y/Z读到的数字输出
 *  Sensitivity = 每LSB对应的加速度值，单位为g/LSB，取决于量程和分辨率设置
 *  Offset = 长期固定偏差，静止时不该有但一直存在的偏移
 *  Noise = 随机波动，静止时也会一跳一跳的小变化
 * ============================================================ */

/* ============================================================
 *  Measurement/StandBy:决定芯片是否进行测量。StandBy模式不进行测量，功耗最低
 *  Sleep:是Measurement体系下的一种低采样睡眠状态
 *  Low Power:是BW_RATE里的低功耗采样方式,通过降低内部采样率节省功耗,但会带来稍大的噪声
 *  Auto Sleep:是"根据静止/活动事件，自动进入或退出Sleep"的机制
 *             设备运动时正常测量，设备长时间静止时自动降低功耗，检测到活动后再自动恢复正常数据速率
 * ============================================================ */

/* ============================================================
 *  寄存器地址定义
 * ============================================================ */
/* Reset Value: 11100101B 可读 固定返回0xE5*/
#define ADXL345_REG_DEVID           0x00U   /* 设备ID(固定值0xE5) */

/* Reset Value: 00001010B 可读可写
 * D7:D5=保留,填0
 * D4(LOW_POWER): 置1进入低功耗模式(降低功耗但增加噪声)，0正常模式
 * D[3:0](Rate): 输出数据速率,详细参考下面的速率枚举定义ADXL345_DataRate_t */
#define ADXL345_REG_BW_RATE         0x2CU   /* 带宽和数据速率和低功耗模式 */

/* Reset Value: 00000000B 可读可写
 * D7:D6=保留,填0
 * D5(Link):默认为0,此时Activity检测和Inactivity检测可以共同工作
 *          置1且Activity和Inactivity都使能时,会推迟Activity的启动,直到先检测到Inactivity;
 *          检测到Activity后，又开始Inactivity检测，从而把活动和静止检测“串联”起来,而不是同时独立工作。
 * D4(Auto_Sleep):Link置0,AUTO SLEEP机制无法工作;Link置1,Auto_Sleep也置1开启AUTO SLEEP机制。
 *                在检测到Inactivity后会自动进入Sleep模式,自动禁止Inactivity,使能Activity
 *                检测到Activity后会自动切回原始数据速率,自动禁止Activity,使能Inactivity
 * D3(Measure):默认为StandBy模式,置1进入Measurement模式
 * D2(Sleep):默认为正常工作模式,置1进入Sleep模式。Sleep模式会抑制DATA_READY中断，停止向FIFO传输数据，并将采样率切换到Wakeup指定的采样率。
 *           即使DATA_READY中断被抑制，数据寄存器仍会以Wakeup设置的采样率进行更新
 * D1:D0(Wakeup):只在Sleep模式下有意义,用来决定Sleep状态下的低速唤醒/采样频率。00=8Hz, 01=4Hz, 10=2Hz, 11=1Hz */
#define ADXL345_REG_POWER_CTL       0x2DU   /* 电源控制(测量模式、休眠、自动休眠) */

/* Reset Value: 00000000B 可读可写
 * The DATA_FORMAT register controls the presentation of data to Register 0x32 through Register 0x37.
 * All data, except that for the ±16g range, must be clipped to avoid rollover.
 * rollover可理解为数字码值回绕，类似:+511再加1→-512。为了避免码值从正最大突然翻到负值，芯片把输出限制在最大/最小可表示范围内，就是clipped
 * 16g是ADXL345最大量程，动态范围最大;对于较小量程，如果输入加速度超过该量程，数字码值更容易超出当前有效位能表示的范围。
 * 目前我只使用了倾角测量功能，正常情况下每个轴不会超过1g
 *
 * D7(SELF_TEST):默认为正常工作模式,置1为自检模式，会导致输出数据发生偏移，用于测试传感器功能
 * D6(SPI):置1为3线SPI模式，0为4线SPI模式(默认)，I2C模式忽略此位
 * D5(INT_INVERT):置1中断设置为低电平有效，默认为0(中断设置为高电平有效)
 * D4=保留, 填0
 * D3(FULL_RES):置1为全分辨率模式，比例因子固定为4mg/LSB，量程和分辨率随Range位自动调整；置0为10位模式(默认)，比例因子和量程由Range位设置
 * D2(Justify):置1为左对齐(MSB)，0为右对齐有符号数(默认)
 * D[1:0](Range): 量程枚举值，详细参考下面的量程枚举定义ADXL345_Range_t */
#define ADXL345_REG_DATA_FORMAT     0x31U   /* 数据格式(分辨率、量程、数据对齐方式) */

/* Register 0x32 to Register 0x37(数据寄存器):Reset Value: 00000000B 只读
 * the measured output for each axis is expressed in LSBs
 * 建议对这6个寄存器执行多字节读取，以防止在连续读取寄存器之间数据发生变化*/
#define ADXL345_REG_DATAX0          0x32U   /* X轴数据低字节 */
#define ADXL345_REG_DATAX1          0x33U   /* X轴数据高字节 */
#define ADXL345_REG_DATAY0          0x34U   /* Y轴数据低字节 */
#define ADXL345_REG_DATAY1          0x35U   /* Y轴数据高字节 */
#define ADXL345_REG_DATAZ0          0x36U   /* Z轴数据低字节 */
#define ADXL345_REG_DATAZ1          0x37U   /* Z轴数据高字节 */

/* Register 0x1E to Register 0x20(偏移校准寄存器):Reset Value: 00000000B 可读可写
 * 8位都用来存储用户设置的的偏移调整值(二进制补码格式)，比例因子为15.6mg/LSB
 * 偏移寄存器中存储的值会自动添加到加速度数据中，并将结果存储在输出数据寄存器中*/
#define ADXL345_REG_OFSX            0x1EU   /* X轴偏移校准 */
#define ADXL345_REG_OFSY            0x1FU   /* Y轴偏移校准 */
#define ADXL345_REG_OFSZ            0x20U   /* Z轴偏移校准 */

/* ============================================================
 *  用户敲击检测(TAP Detection)，可触发单击/双击中断
 *  以下内容详见手册APPLICATIONS INFORMATION章节的TAP DETECTION部分
 *  
 *  1、当只使用单击检测时，LATENT和WINDOW寄存器可以置0，因为单击检测只需要判断一次加速度冲击是否超过THRESH_TAP，以及持续时间是否小于DUR
 *  2、当使用双击检测时，LATENT和WINDOW不能置0。LATENT表示从第一次敲击结束，即加速度回落到THRESH_TAP以下之后，到第二次敲击检测窗口开始之间的等待时间。
 *     在该时间段内出现的加速度冲击不会被检测为第二次敲击，用于避免第一次敲击后的机械回弹、余振或回波被误判。
 *     WINDOW表示Latency结束后允许第二次敲击开始的时间范围。第二次敲击必须在该窗口内开始，但不要求在窗口结束前完成;同时，只要其持续时间满足DUR的限制，仍可被判定为有效双击。
 *
 *  3、如果仅使用单击检测功能，当加速度低于THRESH_TAP且时间未超过DUR时，就会触发单击检测中断
 *  4、如果同时使用单击和双击检测功能，如果第二次敲击有效，则触发双击检测中断;如果第二次敲击无效，则触发单击检测中断。
 *     三种Double Tap失效的情况分别是：
 *          Suppress置位后，Latency期间出现超过阈值的冲击(Figure 47.);
 *          Winodw刚开始时已经检测到超过阈值(Figure 48.);
 *          第二次敲击超过DUR限制(Figure 48.)
 *
 *  DUR、LATENT、WINDOW、THRESH_TAP寄存器的值会影响单击和双击事件的检测，建议根据实际应用需求进行调整和测试，以获得最佳性能
 *  手册推荐初值：THRESH_TAP>0x30(0x30*62.5mg=3g)，DUR>0x10(10ms)，LATENT>0x10(20ms)，WINDOW>0x40(80ms)
 * ============================================================ */
/* Register 0x1D and Register 0x21 to Register 0x23:Reset Value: 00000000B 可读可写
 * 8位无符号数，比例因子为62.5mg/LSB，如果启用了Single/Double Tap中断，置0可能会导致异常行为*/
#define ADXL345_REG_THRESH_TAP      0x1DU   /* 敲击阈值 */
/* 8位无符号数，比例因子为625μs/LSB，0会禁止单/双击功能 */
#define ADXL345_REG_DUR             0x21U   /* 敲击持续时间 */
/* 8位无符号数，比例因子为1.25ms/LSB，0会禁止双击功能 */
#define ADXL345_REG_LATENT          0x22U   /* 敲击潜伏时间*/
/* 8位无符号数，比例因子为1.25ms/LSB，0会禁止双击功能 */
#define ADXL345_REG_WINDOW          0x23U   /* 敲击窗口 */
/* Reset Value: 00000000B 可读可写
 * D[7:4]=保留,填0
 * D3(Suppress):置1可抑制双击检测
 * D2(TAP_X enable)、D1(TAP_Y enable)、D0(TAP_Z enable):哪一位置1，哪一位对应的轴参与敲击检测 */
#define ADXL345_REG_TAP_AXES        0x2AU   /* 敲击轴使能 */
/* Reset Value: 00000000B 只读
 * D7=保留，填0
 * D[6:4](ACT_X/Y/Z source)、D[2:0](TAP_X/Y/Z source):表示参与TAP/ACTIVITY事件的第一个轴，置1代表参与
 * 当有新数据可用时，这些位不会被清除，而是会被新数据覆盖。应在清除中断之前读取ACT_TAP_STATUS寄存器的值。
 * D3(Asleep):仅当设备配置为Auto sleep时，此位才会切换。置1代表设备处于Sleep模式 */
#define ADXL345_REG_ACT_TAP_STATUS  0x2BU   /* 活动/敲击状态 */


/* ============================================================
 *  ACTIVITY/INACTIVITY
 * ============================================================ */
/* Register 0x24 to Register 0x27:Reset Value: 00000000B 可读可写
 * 8位无符号数,比例因子为62.5mg/LSB,如果开启中断，置0会导致异常行为 */
#define ADXL345_REG_THRESH_ACT      0x24U   /* 活动阈值 */
/* 8位无符号数,比例因子为62.5mg/LSB,如果开启中断，置0会导致异常行为 */
#define ADXL345_REG_THRESH_INACT    0x25U   /* 静止阈值 */
/* 8位无符号数,比例因子为1s/LSB,最大值为255s */
#define ADXL345_REG_TIME_INACT      0x26U   /* 静止时间 */
/* D7(ACT ac/dc),D3(INACT ac/dc):默认为直流模式,置1为交流模式
 * DC模式下，将当前加速度幅值与THRESH_ACT/INACT中存储的值直接比较，确定是否触发活动/静止事件;
 * AC模式下进行活动检测时,开始时的加速度值被用作参考值，如果当前加速度值与参考值的差值超过THRESH_ACT中存储的值,则触发活动中断;
 * AC模式下进行静止检测时,如果当前加速度值与参考值的差值在TIME_INACT设定的时间内一直小于THRESH_INACT中存储的值,则触发静止中断。
 * D[6:4](ACT_X/Y/Z enable),D[2:0](INACT_X/Y/Z enable):设置为1时,对应的轴参与检测;设置为0时,所选轴不参与检测。如果所有轴均置0，则该功能将被禁用。*/
#define ADXL345_REG_ACT_INACT_CTL   0x27U   /* 活动/静止检测控制 */


 /* FREE_FALL(检测设备是否处于自由下落状态，用于防震保护或掉落报警)
  * Register 0x28 to Register 0x29: Reset Value: 00000000B 可读可写
  * 当自由下落中断开启时，THRESH_FF和TIME_FF任意一个置0都会导致异常行为
  * 8位无符号数，比例因子为62.5mg/LSB,推荐值为300mg到600mg(0x05 to 0x09) */
#define ADXL345_REG_THRESH_FF       0x28U   /* 自由落体阈值 */
/* 8位无符号数，比例因子为5ms/LSB,推荐值为100ms到350ms(0x14 to 0x46) */
#define ADXL345_REG_TIME_FF         0x29U   /* 自由落体时间 */


/* ============================================================
 *  INTERRUPTS
 *  中断功能通过两种方式锁存和清除：
 *      对于数据相关的中断，通过读取数据寄存器(地址0x32到地址0x37)直到中断条件不再有效;
 *      对于其余中断，则通过读取INT_SOURCE寄存器来清除。
 * D7(DATA_READY):当有新数据可用时置1;当无新数据可用时置0
 * D6(SINGLE_TAP)、D5(DOUBLE_TAP)
 * D4(Activity):当任何参与轴上的加速度大于THRESH_ACT中存储的值时,被置位，
 * D3(Inactivity):当所有参与轴上的加速度小于THRESH_INACT中存储的值，且持续时间超过TIME_INACT中指定的时间时置1。 
 * D2(FREE_FALL):当所有轴的加速度小于THRESH_FF中存储的值，且持续时间超过TIME_FF中指定的时间时置1
 * D1(Watermark):当FIFO中的样本数等于存储在FIFO_CTL[4:0]中的值时，被置位。读取FIFO时，会自动置0，FIFO的内容恢复到低于存储在Samples中的值。
 * D0(Overrun):在旁路模式下,当新数据替换DATA寄存器中未读取的数据时,置1;在所有其他模式下，当FIFO被填满时置1。读取FIFO的内容后，Overrun位会自动清除。
 * ============================================================ */
/* Reset Value: 00000000B 可读可写 
*  位设置为1可启用相应的事件中断,而设置为0则禁用该事件中断
*  The DATA_READY, watermark, and overrun bits enable only the interrupt output; the functions are always enabled. */ 
#define ADXL345_REG_INT_ENABLE      0x2EU   /* 中断使能 */
/* Reset Value: 00000000B 可读可写
 * 任何设置为0的位都会将其对应的中断发送到INT1引脚;而设置为1的位则会将其对应的中断发送到INT2引脚
 * 给定引脚的所有选定中断都会进行逻辑或运算*/
#define ADXL345_REG_INT_MAP         0x2FU   /* 中断映射到INT1/INT2引脚 */
/* Reset Value: 00000010B 只读
 * 置1的位说明其对应事件已被触发,而置0的位说明其对应事件未被触发 */
#define ADXL345_REG_INT_SOURCE      0x30U   /* 中断源状态*/

/* ============================================================
 *  FIFO
 * ============================================================ */
/* Reset Value: 00000000B 可读可写
 * D[7:6](FIFI_MODE)
 * D5(Trigger):置0时触发模式的触发事件与INT1引脚相关联;置1时触发事件与INT2引脚相关联
 * D[4:0](Samples) */
#define ADXL345_REG_FIFO_CTL        0x38U   /* FIFO控制 */

/* Reset Value: 00000000B 只读
 * D7(FIFO_TRIG)
 * D6=保留,填0
 * D[5:0](Entries)*/
#define ADXL345_REG_FIFO_STATUS     0x39U   /* FIFO状态 */


/* ============================================================
 *  常量定义
 * ============================================================ */
#define ADXL345_DEVICE_ID           0xE5U   /* 设备ID期望值 */
/* An alternate I2C address of 0x53 (followed by the R/W bit) 
 * can be chosen by grounding the ALT ADDRESS pin (Pin 12).
 * This translates to 0xA6 for a write and 0xA7 for a read. */
#define ADXL345_I2C_ADDR_LOW        0x53U   /* I2C地址（SDO=GND），7位地址 */
/* With the ALT ADDRESS pin high, the 7-bit I2C address for the device is 0x1D, 
 * followed by the R/W bit. This translates to 0x3A for a write and 0x3B for a read. */
#define ADXL345_I2C_ADDR_HIGH       0x1DU   /* I2C地址（SDO=VCC），7位地址 */

#define ADXL345_TIMEOUT_MS          100U    /* I2C通信超时时间 */

/* ============================================================
 *  数据速率枚举(BW_RATE寄存器D[3:0])
 *  The ADXL345 automatically modulates its power consumption in proportion to its output data rate.
 *  低功耗模式(BW_RATE D[4]=1)时,手册推荐数据速率范围从12.5Hz到400Hz，比非低功率模式节省功率，但代价是噪声略微增加
 * ============================================================ */
typedef enum {
    ADXL345_RATE_0_10   = 0x00U,   /* 0.10 Hz */
    ADXL345_RATE_0_20   = 0x01U,   /* 0.20 Hz */
    ADXL345_RATE_0_39   = 0x02U,   /* 0.39 Hz */
    ADXL345_RATE_0_78   = 0x03U,   /* 0.78 Hz */
    ADXL345_RATE_1_56   = 0x04U,   /* 1.56 Hz */
    ADXL345_RATE_3_13   = 0x05U,   /* 3.13 Hz */
    ADXL345_RATE_6_25   = 0x06U,   /* 6.25 Hz */
    ADXL345_RATE_12_5   = 0x07U,   /* 12.5 Hz */
    ADXL345_RATE_25     = 0x08U,   /* 25 Hz */
    ADXL345_RATE_50     = 0x09U,   /* 50 Hz */
    ADXL345_RATE_100    = 0x0AU,   /* 100 Hz(默认) */
    ADXL345_RATE_200    = 0x0BU,   /* 200 Hz */
    ADXL345_RATE_400    = 0x0CU,   /* 400 Hz */
    ADXL345_RATE_800    = 0x0DU,   /* 800 Hz */
    ADXL345_RATE_1600   = 0x0EU,   /* 1600 Hz */
    ADXL345_RATE_3200   = 0x0FU,   /* 3200 Hz */
} ADXL345_DataRate_t;

/* ============================================================
 *  量程枚举(DATA_FORMAT寄存器D[1:0])
 *  在全分辨率模式下，比例因子保持4mg/LSB;在10位模式下，比例因子随量程变化
 * ============================================================ */
typedef enum {
    ADXL345_RANGE_2G   = 0x00U,   /* ±2g(默认)，10位模式：4mg/LSB */
    ADXL345_RANGE_4G   = 0x01U,   /* ±4g，10位模式：7.8mg/LSB */
    ADXL345_RANGE_8G   = 0x02U,   /* ±8g，10位模式：15.6mg/LSB */
    ADXL345_RANGE_16G  = 0x03U,   /* ±16g，10位模式：31.2mg/LSB */
} ADXL345_Range_t;

/* ============================================================
 *  加速度数据结构体
 *  每个轴的原始数据为16位有符号整数
 * ============================================================ */
typedef struct {
    int16_t x;    /* X轴加速度原始值 */
    int16_t y;    /* Y轴加速度原始值 */
    int16_t z;    /* Z轴加速度原始值 */
} ADXL345_AccelData_t;

/* ============================================================
 *  配置结构体
 * ============================================================ */
typedef struct {
    ADXL345_DataRate_t  data_rate;      /* 数据输出速率 */
    ADXL345_Range_t     range;          /* 量程 */
    bool                full_res;       /* 全分辨率模式：true=13位，false=10位 */
    bool                low_power;      /* 低功耗模式（降低功耗但增加噪声）*/
    int8_t              offset_x;       /* X轴偏移校准值 */
    int8_t              offset_y;       /* Y轴偏移校准值 */
    int8_t              offset_z;       /* Z轴偏移校准值 */
    bool                addr_high;      /* I2C地址选择：true=0x1D(SDO=VCC), false=0x53(SDO=GND) */
} ADXL345_Config_t;


/**
 * @brief  初始化ADXL345
 * @param  cfg 配置参数结构体指针
 * @retval true: 成功, false: 失败（设备ID不匹配或I2C通信失败）
 * @note   初始化后会自动进入测量模式
 */
bool ADXL345_Init(const ADXL345_Config_t *cfg);

/**
 * @brief  读取设备ID
 * @retval 设备ID（期望值0xE5）
 */
uint8_t ADXL345_GetDeviceID(void);

/**
 * @brief  读取三轴加速度数据
 * @param  data 数据存储结构体指针
 * @retval true: 成功, false: 失败
 */
bool ADXL345_ReadAccel(ADXL345_AccelData_t *data);

/**
 * @brief  读取X轴加速度
 * @retval X轴加速度原始值
 */
int16_t ADXL345_ReadX(void);

/**
 * @brief  读取Y轴加速度
 * @retval Y轴加速度原始值
 */
int16_t ADXL345_ReadY(void);

/**
 * @brief  读取Z轴加速度
 * @retval Z轴加速度原始值
 */
int16_t ADXL345_ReadZ(void);

/**
 * @brief  将原始加速度值转换为g值
 * @param  raw 原始加速度值
 * @param  range 量程设置
 * @param  full_res 是否全分辨率模式
 * @retval 加速度值（单位：g）
 * @note   全分辨率：4mg/LSB
 *         10位模式：range=2G时4mg/LSB，range=4G时7.8mg/LSB...
 */
float ADXL345_RawToG(int16_t raw, ADXL345_Range_t range, bool full_res);

/**
 * @brief  设置数据速率
 * @param  rate 数据速率枚举值
 * @param  low_power 是否低功耗模式
 */
void ADXL345_SetDataRate(ADXL345_DataRate_t rate, bool low_power);

/**
 * @brief  设置量程和分辨率模式
 * @param  range 量程枚举值
 * @param  full_res 是否全分辨率模式
 */
void ADXL345_SetRange(ADXL345_Range_t range, bool full_res);

/**
 * @brief  设置偏移校准
 * @param  x X轴偏移值
 * @param  y Y轴偏移值
 * @param  z Z轴偏移值
 * @note   偏移比例因子：15.6mg/LSB
 */
void ADXL345_SetOffset(int8_t x, int8_t y, int8_t z);

/**
 * @brief  进入测量模式
 */
void ADXL345_StartMeasurement(void);

/**
 * @brief  进入待机模式
 */
void ADXL345_StopMeasurement(void);

/**
 * @brief  进入休眠模式（比待机更低功耗）
 */
void ADXL345_EnterSleep(void);

/**
 * @brief  软件复位
 * @note   复位后需等待1ms
 */
void ADXL345_SoftReset(void);

#endif 

