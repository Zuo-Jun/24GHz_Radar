/**
 ******************************************************************************
 * @file    ADXL345.h
 * @brief   ADXL345 三轴数字加速度计驱动头文件
 *          针对 STM32F405RGT6 (168MHz) 优化，I2C接口
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
 *   SDA           | PC1(I2C1)| I2C数据线
 *   SCL           | PC0(I2C1)| I2C时钟线
 *   INT1          | 可选     | 中断输出1
 *   INT2          | 可选     | 中断输出2
 *
 * @I2C通信说明
 *   - 支持标准模式（100kHz）和快速模式（400kHz）
 *   - 7位从地址：0x53（SDO=GND）或 0x1D（SDO=VCC）
 *   - 写操作格式：[START][从地址+W][寄存器地址][数据][STOP]
 *   - 读操作格式：[START][从地址+W][寄存器地址][Sr][从地址+R][数据][STOP]
 *   - 支持多字节读写，地址自动递增
 *
 * @寄存器说明
 *   地址    名称            功能描述
 *   0x00    DEVID           设备ID（固定值0xE5）
 *   0x1D    THRESH_TAP      敲击阈值（比例因子：62.5mg/LSB）
 *   0x1E    OFSX            X轴偏移校准（比例因子：15.6mg/LSB）
 *   0x1F    OFSY            Y轴偏移校准
 *   0x20    OFSZ            Z轴偏移校准
 *   0x21    DUR             敲击持续时间（比例因子：625μs/LSB）
 *   0x22    LATENT          敲击潜伏时间（比例因子：1.25ms/LSB）
 *   0x23    WINDOW          敲击窗口时间（比例因子：1.25ms/LSB）
 *   0x24    THRESH_ACT      活动阈值（比例因子：62.5mg/LSB）
 *   0x25    THRESH_INACT    静止阈值（比例因子：62.5mg/LSB）
 *   0x26    TIME_INACT      静止时间（比例因子：1s/LSB）
 *   0x27    ACT_INACT_CTL   活动/静止检测控制
 *   0x28    THRESH_FF       自由落体阈值（比例因子：62.5mg/LSB，推荐值：5~9）
 *   0x29    TIME_FF         自由落体时间（比例因子：5ms/LSB）
 *   0x2A    TAP_AXES        敲击轴使能
 *   0x2B    ACT_TAP_STATUS  活动/敲击状态（只读）
 *   0x2C    BW_RATE         数据速率和低功耗模式
 *   0x2D    POWER_CTL       电源控制（测量模式、休眠、自动休眠）
 *   0x2E    INT_ENABLE      中断使能
 *   0x2F    INT_MAP         中断映射到INT1/INT2引脚
 *   0x30    INT_SOURCE      中断源状态（只读）
 *   0x31    DATA_FORMAT     数据格式（分辨率、量程、对齐方式）
 *   0x32    DATAX0          X轴数据低字节
 *   0x33    DATAX1          X轴数据高字节
 *   0x34    DATAY0          Y轴数据低字节
 *   0x35    DATAY1          Y轴数据高字节
 *   0x36    DATAZ0          Z轴数据低字节
 *   0x37    DATAZ1          Z轴数据高字节
 *   0x38    FIFO_CTL        FIFO控制
 *   0x39    FIFO_STATUS     FIFO状态（只读）
 ******************************************************************************
 */

#ifndef __ADXL345_H
#define __ADXL345_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 *  寄存器地址定义
 * ============================================================ */
#define ADXL345_REG_DEVID           0x00U   /* 设备ID，期望值0xE5 */
#define ADXL345_REG_THRESH_TAP      0x1DU   /* 敲击阈值 */
#define ADXL345_REG_OFSX            0x1EU   /* X轴偏移 */
#define ADXL345_REG_OFSY            0x1FU   /* Y轴偏移 */
#define ADXL345_REG_OFSZ            0x20U   /* Z轴偏移 */
#define ADXL345_REG_DUR             0x21U   /* 敲击持续时间 */
#define ADXL345_REG_LATENT          0x22U   /* 敲击潜伏时间 */
#define ADXL345_REG_WINDOW          0x23U   /* 敲击窗口 */
#define ADXL345_REG_THRESH_ACT      0x24U   /* 活动阈值 */
#define ADXL345_REG_THRESH_INACT    0x25U   /* 静止阈值 */
#define ADXL345_REG_TIME_INACT      0x26U   /* 静止时间 */
#define ADXL345_REG_ACT_INACT_CTL   0x27U   /* 活动/静止控制 */
#define ADXL345_REG_THRESH_FF       0x28U   /* 自由落体阈值 */
#define ADXL345_REG_TIME_FF         0x29U   /* 自由落体时间 */
#define ADXL345_REG_TAP_AXES        0x2AU   /* 敲击轴使能 */
#define ADXL345_REG_ACT_TAP_STATUS  0x2BU   /* 活动/敲击状态（只读）*/
#define ADXL345_REG_BW_RATE         0x2CU   /* 带宽和数据速率 */
#define ADXL345_REG_POWER_CTL       0x2DU   /* 电源控制 */
#define ADXL345_REG_INT_ENABLE      0x2EU   /* 中断使能 */
#define ADXL345_REG_INT_MAP         0x2FU   /* 中断映射 */
#define ADXL345_REG_INT_SOURCE      0x30U   /* 中断源（只读）*/
#define ADXL345_REG_DATA_FORMAT     0x31U   /* 数据格式 */
#define ADXL345_REG_DATAX0          0x32U   /* X轴数据低字节 */
#define ADXL345_REG_DATAX1          0x33U   /* X轴数据高字节 */
#define ADXL345_REG_DATAY0          0x34U   /* Y轴数据低字节 */
#define ADXL345_REG_DATAY1          0x35U   /* Y轴数据高字节 */
#define ADXL345_REG_DATAZ0          0x36U   /* Z轴数据低字节 */
#define ADXL345_REG_DATAZ1          0x37U   /* Z轴数据高字节 */
#define ADXL345_REG_FIFO_CTL        0x38U   /* FIFO控制 */
#define ADXL345_REG_FIFO_STATUS     0x39U   /* FIFO状态（只读）*/

/* ============================================================
 *  常量定义
 * ============================================================ */
#define ADXL345_DEVICE_ID           0xE5U   /* 设备ID期望值 */
#define ADXL345_I2C_ADDR_LOW        0x53U   /* I2C地址（SDO=GND），7位地址 */
#define ADXL345_I2C_ADDR_HIGH       0x1DU   /* I2C地址（SDO=VCC），7位地址 */
#define ADXL345_TIMEOUT_MS          100U    /* I2C通信超时时间 */

/* ============================================================
 *  数据速率枚举（BW_RATE寄存器低4位）
 *  比例因子：当LOW_POWER=0时，带宽=数据速率/2
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
    ADXL345_RATE_25     = 0x08U,   /* 25 Hz（默认）*/
    ADXL345_RATE_50     = 0x09U,   /* 50 Hz */
    ADXL345_RATE_100    = 0x0AU,   /* 100 Hz（常用）*/
    ADXL345_RATE_200    = 0x0BU,   /* 200 Hz */
    ADXL345_RATE_400    = 0x0CU,   /* 400 Hz */
    ADXL345_RATE_800    = 0x0DU,   /* 800 Hz */
    ADXL345_RATE_1600   = 0x0EU,   /* 1600 Hz */
    ADXL345_RATE_3200   = 0x0FU,   /* 3200 Hz */
} ADXL345_DataRate_t;

/* ============================================================
 *  量程枚举（DATA_FORMAT寄存器D1:D0）
 *  在全分辨率模式下，精度保持4mg/LSB
 *  在10位模式下，精度随量程变化
 * ============================================================ */
typedef enum {
    ADXL345_RANGE_2G   = 0x00U,   /* ±2g（默认），10位模式：4mg/LSB */
    ADXL345_RANGE_4G   = 0x01U,   /* ±4g，10位模式：7.8mg/LSB */
    ADXL345_RANGE_8G   = 0x02U,   /* ±8g，10位模式：15.6mg/LSB */
    ADXL345_RANGE_16G  = 0x03U,   /* ±16g，10位模式：31.2mg/LSB */
} ADXL345_Range_t;

/* ============================================================
 *  FIFO模式枚举（FIFO_CTL寄存器D7:D6）
 * ============================================================ */
typedef enum {
    ADXL345_FIFO_BYPASS    = 0x00U,   /* 旁路模式（默认）*/
    ADXL345_FIFO_FIFO      = 0x01U,   /* FIFO模式，收集数据直到满或触发 */
    ADXL345_FIFO_STREAM    = 0x02U,   /* 流模式，持续采样，新数据覆盖旧数据 */
    ADXL345_FIFO_TRIGGER   = 0x03U,   /* 触发模式，触发事件后保存samples */
} ADXL345_FifoMode_t;

/* ============================================================
 *  中断类型枚举
 * ============================================================ */
typedef enum {
    ADXL345_INT_OVERRUN   = 0x01U,   /* 数据覆盖中断 */
    ADXL345_INT_WATERMARK = 0x02U,   /* FIFO水印中断 */
    ADXL345_INT_FREEFALL  = 0x04U,   /* 自由落体中断 */
    ADXL345_INT_INACTIVITY= 0x08U,   /* 静止中断 */
    ADXL345_INT_ACTIVITY  = 0x10U,   /* 活动中断 */
    ADXL345_INT_DOUBLETAP = 0x20U,   /* 双击中断 */
    ADXL345_INT_SINGLETAP = 0x40U,   /* 单击中断 */
    ADXL345_INT_DATAREADY = 0x80U,   /* 数据就绪中断 */
} ADXL345_IntType_t;

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
    ADXL345_FifoMode_t  fifo_mode;      /* FIFO模式 */
    uint8_t             fifo_samples;   /* FIFO采样数（0-31），触发水印中断 */
    int8_t              offset_x;       /* X轴偏移校准值 */
    int8_t              offset_y;       /* Y轴偏移校准值 */
    int8_t              offset_z;       /* Z轴偏移校准值 */
    bool                addr_high;      /* I2C地址选择：true=0x1D(SDO=VCC), false=0x53(SDO=GND) */
} ADXL345_Config_t;

/* ============================================================
 *  API 函数声明
 * ============================================================ */

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
 * @brief  配置FIFO
 * @param  mode FIFO模式
 * @param  samples 触发采样数（0-31）
 */
void ADXL345_ConfigFifo(ADXL345_FifoMode_t mode, uint8_t samples);

/**
 * @brief  读取FIFO状态
 * @retval FIFO中的数据条目数（0-32）
 */
uint8_t ADXL345_GetFifoStatus(void);

/**
 * @brief  配置活动检测
 * @param  threshold 活动阈值（比例因子：62.5mg/LSB）
 * @param  ac_dc 0=直流耦合，1=交流耦合
 * @param  axes_en 使能检测的轴（bit0=X, bit1=Y, bit2=Z）
 */
void ADXL345_ConfigActivity(uint8_t threshold, bool ac_dc, uint8_t axes_en);

/**
 * @brief  配置静止检测
 * @param  threshold 静止阈值（比例因子：62.5mg/LSB）
 * @param  time 静止时间（比例因子：1s/LSB）
 * @param  ac_dc 0=直流耦合，1=交流耦合
 * @param  axes_en 使能检测的轴（bit0=X, bit1=Y, bit2=Z）
 */
void ADXL345_ConfigInactivity(uint8_t threshold, uint8_t time, bool ac_dc, uint8_t axes_en);

/**
 * @brief  配置自由落体检测
 * @param  threshold 阈值（推荐5-9，比例因子：62.5mg/LSB）
 * @param  time 时间（比例因子：5ms/LSB）
 */
void ADXL345_ConfigFreeFall(uint8_t threshold, uint8_t time);

/**
 * @brief  配置单击检测
 * @param  threshold 敲击阈值（比例因子：62.5mg/LSB）
 * @param  duration 持续时间（比例因子：625μs/LSB）
 * @param  axes_en 使能检测的轴（bit0=X, bit1=Y, bit2=Z）
 */
void ADXL345_ConfigSingleTap(uint8_t threshold, uint8_t duration, uint8_t axes_en);

/**
 * @brief  配置双击检测
 * @param  threshold 敲击阈值
 * @param  duration 持续时间
 * @param  latent 潜伏时间（比例因子：1.25ms/LSB）
 * @param  window 窗口时间（比例因子：1.25ms/LSB）
 * @param  axes_en 使能检测的轴
 */
void ADXL345_ConfigDoubleTap(uint8_t threshold, uint8_t duration,
                              uint8_t latent, uint8_t window, uint8_t axes_en);

/**
 * @brief  使能中断
 * @param  int_type 中断类型（可或多个中断类型）
 * @param  map_int1 true: 映射到INT1, false: 映射到INT2
 */
void ADXL345_EnableInterrupt(ADXL345_IntType_t int_type, bool map_int1);

/**
 * @brief  禁用中断
 * @param  int_type 中断类型
 */
void ADXL345_DisableInterrupt(ADXL345_IntType_t int_type);

/**
 * @brief  读取中断源
 * @retval 中断源状态（各中断位）
 */
uint8_t ADXL345_GetInterruptSource(void);

/**
 * @brief  清除中断
 * @note   读取INT_SOURCE寄存器自动清除中断
 */
void ADXL345_ClearInterrupt(void);

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

