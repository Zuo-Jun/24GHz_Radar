/**
 ******************************************************************************
 * @file    ADXL345.c
 * @brief   ADXL345 三轴数字加速度计驱动源文件
 *          使用I2C接口与STM32F405RGT6通信
 *
 * @实现说明
 *   - 使用HAL库的I2C函数进行通信
 *   - 支持7位从地址：0x53（SDO=GND）或 0x1D（SDO=VCC）
 *   - 支持多字节读写，地址自动递增
 *   - I2C时钟频率由CubeMX配置，推荐使用400kHz（快速模式）
 ******************************************************************************
 */

#include "ADXL345.h"

/* ============================================================
 *  模块级变量
 * ============================================================ */
/* 当前配置缓存，用于计算g值转换 */
static ADXL345_Range_t s_current_range = ADXL345_RANGE_2G;
static bool s_full_resolution = false;

/* I2C 8位写地址（7位地址左移1位）*/
static uint8_t s_i2c_addr = (ADXL345_I2C_ADDR_LOW << 1);

/* I2C句柄声明（需在main.c或CubeMX生成文件中定义并初始化）*/
extern I2C_HandleTypeDef hi2c1;

/* ============================================================
 *  底层I2C通信函数
 * ============================================================ */

/**
 * @brief  I2C写一个字节
 * @param  addr 寄存器地址
 * @param  data 要写入的数据
 * @retval HAL状态
 */
static HAL_StatusTypeDef ADXL345_I2C_WriteByte(uint8_t addr, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, s_i2c_addr, addr,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, ADXL345_TIMEOUT_MS);
}

/**
 * @brief  I2C读一个字节
 * @param  addr 寄存器地址
 * @retval 读取的数据
 */
static uint8_t ADXL345_I2C_ReadByte(uint8_t addr)
{
    uint8_t data = 0U;
    HAL_I2C_Mem_Read(&hi2c1, s_i2c_addr, addr,
                     I2C_MEMADD_SIZE_8BIT, &data, 1, ADXL345_TIMEOUT_MS);
    return data;
}

/**
 * @brief  I2C多字节读
 * @param  addr 起始寄存器地址
 * @param  buf 数据缓冲区
 * @param  len 要读取的字节数
 * @retval HAL状态
 * @note   地址自动递增，适合读取加速度数据
 */
static HAL_StatusTypeDef ADXL345_I2C_ReadMulti(uint8_t addr, uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, s_i2c_addr, addr,
                            I2C_MEMADD_SIZE_8BIT, buf, len, ADXL345_TIMEOUT_MS);
}

/**
 * @brief  I2C多字节写
 * @param  addr 起始寄存器地址
 * @param  buf 数据缓冲区
 * @param  len 要写入的字节数
 */
static void ADXL345_I2C_WriteMulti(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c1, s_i2c_addr, addr,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, ADXL345_TIMEOUT_MS);
}

/* ============================================================
 *  API 函数实现
 * ============================================================ */

/**
 * @brief  初始化ADXL345
 * @param  cfg 配置参数结构体指针
 * @retval true: 成功, false: 失败
 */
bool ADXL345_Init(const ADXL345_Config_t *cfg)
{
    uint8_t id;
    uint8_t data_format;
    uint8_t bw_rate;
    uint8_t power_ctl;
    uint8_t fifo_ctl;
    uint8_t offset_data[3];

    /* 根据配置设置I2C地址 */
    s_i2c_addr = (cfg->addr_high ? ADXL345_I2C_ADDR_HIGH : ADXL345_I2C_ADDR_LOW) << 1;

    /* 读取设备ID，验证通信是否正常 */
    id = ADXL345_I2C_ReadByte(ADXL345_REG_DEVID);
    if (id != ADXL345_DEVICE_ID) {
        return false;  /* 通信失败或芯片不在线 */
    }

    /* 软件复位，确保寄存器处于已知状态 */
    ADXL345_I2C_WriteByte(ADXL345_REG_POWER_CTL, 0xD7U);  /* 写入0xD7触发软件复位 */
    HAL_Delay(2);  /* 等待复位完成（手册要求>1ms）*/

    /* 配置数据格式寄存器
     * D7: SELF_TEST  = 0（自测关闭）
     * D6: SPI        = 0（I2C模式忽略此位）
     * D5: INT_INVERT = 0（中断高有效）
     * D4: 0（保留）
     * D3: FULL_RES   = cfg->full_res
     * D2: JUSTIFY    = 0（右对齐，有符号数）
     * D1:D0 RANGE    = cfg->range
     */
    data_format = (uint8_t)(cfg->range & 0x03U);
    if (cfg->full_res) {
        data_format |= 0x08U;  /* FULL_RES = 1 */
    }
    ADXL345_I2C_WriteByte(ADXL345_REG_DATA_FORMAT, data_format);

    /* 缓存配置用于后续g值转换 */
    s_current_range = cfg->range;
    s_full_resolution = cfg->full_res;

    /* 配置带宽和数据速率寄存器
     * D4: LOW_POWER  = cfg->low_power
     * D3:D0 RATE     = cfg->data_rate
     */
    bw_rate = (uint8_t)(cfg->data_rate & 0x0FU);
    if (cfg->low_power) {
        bw_rate |= 0x10U;  /* LOW_POWER = 1 */
    }
    ADXL345_I2C_WriteByte(ADXL345_REG_BW_RATE, bw_rate);

    /* 配置偏移校准 */
    offset_data[0] = (uint8_t)cfg->offset_x;
    offset_data[1] = (uint8_t)cfg->offset_y;
    offset_data[2] = (uint8_t)cfg->offset_z;
    ADXL345_I2C_WriteMulti(ADXL345_REG_OFSX, offset_data, 3);

    /* 配置FIFO
     * D7:D6 FIFO_MODE = cfg->fifo_mode
     * D5 TRIGGER      = 0（触发引脚INT1）
     * D4:D0 SAMPLES   = cfg->fifo_samples
     */
    fifo_ctl = (uint8_t)((cfg->fifo_mode & 0x03U) << 6) | (cfg->fifo_samples & 0x1FU);
    ADXL345_I2C_WriteByte(ADXL345_REG_FIFO_CTL, fifo_ctl);

    /* 配置电源控制寄存器，进入测量模式
     * D7: AUTO_SLEEP = 0（禁用自动休眠）
     * D6: LINK       = 0（禁用活动/静止联动）
     * D5: MEASURE    = 1（测量模式）
     * D4: SLEEP      = 0（正常模式，不休眠）
     * D3:D0 WAKEUP   = 0000（唤醒频率8Hz，休眠模式下有效）
     */
    power_ctl = 0x08U;  /* MEASURE = 1 */
    ADXL345_I2C_WriteByte(ADXL345_REG_POWER_CTL, power_ctl);

    return true;
}

/**
 * @brief  读取设备ID
 */
uint8_t ADXL345_GetDeviceID(void)
{
    return ADXL345_I2C_ReadByte(ADXL345_REG_DEVID);
}

/**
 * @brief  读取三轴加速度数据
 * @param  data 数据存储结构体指针
 * @retval true: 成功, false: 失败
 */
bool ADXL345_ReadAccel(ADXL345_AccelData_t *data)
{
    uint8_t buf[6];

    /* 连续读取6字节（DATAX0~DATAZ1），地址自动递增 */
    if (ADXL345_I2C_ReadMulti(ADXL345_REG_DATAX0, buf, 6) != HAL_OK) {
        return false;
    }

    /* 组合16位数据（小端格式：低字节在前）*/
    data->x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    data->y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    data->z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    return true;
}

/**
 * @brief  读取X轴加速度
 */
int16_t ADXL345_ReadX(void)
{
    uint8_t buf[2];
    ADXL345_I2C_ReadMulti(ADXL345_REG_DATAX0, buf, 2);
    return (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
}

/**
 * @brief  读取Y轴加速度
 */
int16_t ADXL345_ReadY(void)
{
    uint8_t buf[2];
    ADXL345_I2C_ReadMulti(ADXL345_REG_DATAY0, buf, 2);
    return (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
}

/**
 * @brief  读取Z轴加速度
 */
int16_t ADXL345_ReadZ(void)
{
    uint8_t buf[2];
    ADXL345_I2C_ReadMulti(ADXL345_REG_DATAZ0, buf, 2);
    return (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
}

/**
 * @brief  将原始加速度值转换为g值
 * @note   全分辨率模式：固定4mg/LSB
 *         10位模式：根据量程不同，分辨率不同
 *           ±2g  → 4mg/LSB    (256 LSB/g)
 *           ±4g  → 7.8mg/LSB  (128 LSB/g)
 *           ±8g  → 15.6mg/LSB (64 LSB/g)
 *           ±16g → 31.2mg/LSB (32 LSB/g)
 */
float ADXL345_RawToG(int16_t raw, ADXL345_Range_t range, bool full_res)
{
    float scale;

    if (full_res) {
        /* 全分辨率模式：固定4mg/LSB */
        scale = 0.004f;  /* 4mg = 0.004g */
    } else {
        /* 10位模式：根据量程确定分辨率 */
        switch (range) {
            case ADXL345_RANGE_2G:
                scale = 0.004f;    /* 4mg/LSB */
                break;
            case ADXL345_RANGE_4G:
                scale = 0.0078f;   /* 7.8mg/LSB */
                break;
            case ADXL345_RANGE_8G:
                scale = 0.0156f;   /* 15.6mg/LSB */
                break;
            case ADXL345_RANGE_16G:
            default:
                scale = 0.0312f;   /* 31.2mg/LSB */
                break;
        }
    }

    return (float)raw * scale;
}

/**
 * @brief  设置数据速率
 */
void ADXL345_SetDataRate(ADXL345_DataRate_t rate, bool low_power)
{
    uint8_t bw_rate = (uint8_t)(rate & 0x0FU);
    if (low_power) {
        bw_rate |= 0x10U;
    }
    ADXL345_I2C_WriteByte(ADXL345_REG_BW_RATE, bw_rate);
}

/**
 * @brief  设置量程和分辨率模式
 */
void ADXL345_SetRange(ADXL345_Range_t range, bool full_res)
{
    uint8_t data_format = ADXL345_I2C_ReadByte(ADXL345_REG_DATA_FORMAT);

    /* 清除原有量程和全分辨率位 */
    data_format &= ~0x0BU;  /* 清除D3(FULL_RES), D1:D0(RANGE) */

    /* 设置新的量程和分辨率 */
    data_format |= (uint8_t)(range & 0x03U);
    if (full_res) {
        data_format |= 0x08U;
    }

    ADXL345_I2C_WriteByte(ADXL345_REG_DATA_FORMAT, data_format);

    /* 更新缓存 */
    s_current_range = range;
    s_full_resolution = full_res;
}

/**
 * @brief  设置偏移校准
 */
void ADXL345_SetOffset(int8_t x, int8_t y, int8_t z)
{
    uint8_t offset_data[3];
    offset_data[0] = (uint8_t)x;
    offset_data[1] = (uint8_t)y;
    offset_data[2] = (uint8_t)z;
    ADXL345_I2C_WriteMulti(ADXL345_REG_OFSX, offset_data, 3);
}

/**
 * @brief  配置FIFO
 */
void ADXL345_ConfigFifo(ADXL345_FifoMode_t mode, uint8_t samples)
{
    uint8_t fifo_ctl = (uint8_t)((mode & 0x03U) << 6) | (samples & 0x1FU);
    ADXL345_I2C_WriteByte(ADXL345_REG_FIFO_CTL, fifo_ctl);
}

/**
 * @brief  读取FIFO状态
 */
uint8_t ADXL345_GetFifoStatus(void)
{
    return ADXL345_I2C_ReadByte(ADXL345_REG_FIFO_STATUS) & 0x3FU;
}

/**
 * @brief  配置活动检测
 * @param  threshold 活动阈值（比例因子：62.5mg/LSB，推荐值4~10）
 * @param  ac_dc 0=直流耦合（比较当前值与阈值）
 *               1=交流耦合（比较当前值与参考值的偏差）
 * @param  axes_en 使能检测的轴（bit0=X, bit1=Y, bit2=Z）
 */
void ADXL345_ConfigActivity(uint8_t threshold, bool ac_dc, uint8_t axes_en)
{
    /* 设置活动阈值 */
    ADXL345_I2C_WriteByte(ADXL345_REG_THRESH_ACT, threshold & 0x7FU);

    /* 配置活动检测控制
     * D7: ACT_ACDC   = ac_dc
     * D6: ACT_X_EN   = axes_en bit0
     * D5: ACT_Y_EN   = axes_en bit1
     * D4: ACT_Z_EN   = axes_en bit2
     */
    uint8_t act_ctl = (ac_dc ? 0x80U : 0x00U) | ((axes_en & 0x07U) << 4);
    ADXL345_I2C_WriteByte(ADXL345_REG_ACT_INACT_CTL, act_ctl);
}

/**
 * @brief  配置静止检测
 */
void ADXL345_ConfigInactivity(uint8_t threshold, uint8_t time, bool ac_dc, uint8_t axes_en)
{
    /* 设置静止阈值 */
    ADXL345_I2C_WriteByte(ADXL345_REG_THRESH_INACT, threshold & 0x7FU);

    /* 设置静止时间 */
    ADXL345_I2C_WriteByte(ADXL345_REG_TIME_INACT, time);

    /* 配置静止检测控制（低4位）*/
    uint8_t ctl = ADXL345_I2C_ReadByte(ADXL345_REG_ACT_INACT_CTL);
    ctl &= 0xF0U;  /* 清除低4位 */
    ctl |= (ac_dc ? 0x08U : 0x00U) | (axes_en & 0x07U);
    ADXL345_I2C_WriteByte(ADXL345_REG_ACT_INACT_CTL, ctl);
}

/**
 * @brief  配置自由落体检测
 * @param  threshold 阈值（推荐5-9，对应300mg~600mg）
 * @param  time 时间（推荐20~50，对应100ms~250ms）
 */
void ADXL345_ConfigFreeFall(uint8_t threshold, uint8_t time)
{
    ADXL345_I2C_WriteByte(ADXL345_REG_THRESH_FF, threshold & 0x7FU);
    ADXL345_I2C_WriteByte(ADXL345_REG_TIME_FF, time & 0xFFU);
}

/**
 * @brief  配置单击检测
 */
void ADXL345_ConfigSingleTap(uint8_t threshold, uint8_t duration, uint8_t axes_en)
{
    /* 敲击阈值 */
    ADXL345_I2C_WriteByte(ADXL345_REG_THRESH_TAP, threshold);

    /* 敲击持续时间（最大值127）*/
    ADXL345_I2C_WriteByte(ADXL345_REG_DUR, duration & 0x7FU);

    /* 配置敲击轴使能
     * D7: TAP_SUPPRESS = 0（不抑制双重敲击检测）
     * D6: TAP_X_EN     = axes_en bit0
     * D5: TAP_Y_EN     = axes_en bit1
     * D4: TAP_Z_EN     = axes_en bit2
     */
    uint8_t tap_axes = (axes_en & 0x07U) << 4;
    ADXL345_I2C_WriteByte(ADXL345_REG_TAP_AXES, tap_axes);
}

/**
 * @brief  配置双击检测
 */
void ADXL345_ConfigDoubleTap(uint8_t threshold, uint8_t duration,
                               uint8_t latent, uint8_t window, uint8_t axes_en)
{
    /* 单击参数设置 */
    ADXL345_ConfigSingleTap(threshold, duration, axes_en);

    /* 潜伏时间（第一次敲击结束到第二次敲击开始的最小间隔）*/
    ADXL345_I2C_WriteByte(ADXL345_REG_LATENT, latent);

    /* 窗口时间（从第一次敲击开始到第二次敲击结束的最大间隔）*/
    ADXL345_I2C_WriteByte(ADXL345_REG_WINDOW, window);

    /* 使能双击检测（设置TAP_SUPPRESS=0）*/
    uint8_t tap_axes = ADXL345_I2C_ReadByte(ADXL345_REG_TAP_AXES);
    tap_axes &= ~0x80U;  /* 清除抑制位 */
    ADXL345_I2C_WriteByte(ADXL345_REG_TAP_AXES, tap_axes);
}

/**
 * @brief  使能中断
 * @param  int_type 中断类型（可或多个中断类型）
 * @param  map_int1 true: 映射到INT1, false: 映射到INT2
 */
void ADXL345_EnableInterrupt(ADXL345_IntType_t int_type, bool map_int1)
{
    /* 使能中断 */
    uint8_t int_enable = ADXL345_I2C_ReadByte(ADXL345_REG_INT_ENABLE);
    int_enable |= (uint8_t)int_type;
    ADXL345_I2C_WriteByte(ADXL345_REG_INT_ENABLE, int_enable);

    /* 映射中断到INT1或INT2
     * 每个中断位：0=映射到INT1，1=映射到INT2
     */
    uint8_t int_map = ADXL345_I2C_ReadByte(ADXL345_REG_INT_MAP);
    if (map_int1) {
        int_map &= ~(uint8_t)int_type;  /* 清除位，映射到INT1 */
    } else {
        int_map |= (uint8_t)int_type;   /* 置位，映射到INT2 */
    }
    ADXL345_I2C_WriteByte(ADXL345_REG_INT_MAP, int_map);
}

/**
 * @brief  禁用中断
 */
void ADXL345_DisableInterrupt(ADXL345_IntType_t int_type)
{
    uint8_t int_enable = ADXL345_I2C_ReadByte(ADXL345_REG_INT_ENABLE);
    int_enable &= ~(uint8_t)int_type;
    ADXL345_I2C_WriteByte(ADXL345_REG_INT_ENABLE, int_enable);
}

/**
 * @brief  读取中断源
 */
uint8_t ADXL345_GetInterruptSource(void)
{
    return ADXL345_I2C_ReadByte(ADXL345_REG_INT_SOURCE);
}

/**
 * @brief  清除中断（读取INT_SOURCE寄存器自动清除）
 */
void ADXL345_ClearInterrupt(void)
{
    ADXL345_I2C_ReadByte(ADXL345_REG_INT_SOURCE);
}

/**
 * @brief  进入测量模式
 */
void ADXL345_StartMeasurement(void)
{
    uint8_t power_ctl = ADXL345_I2C_ReadByte(ADXL345_REG_POWER_CTL);
    power_ctl |= 0x08U;  /* 设置MEASURE位 */
    power_ctl &= ~0x04U; /* 清除SLEEP位 */
    ADXL345_I2C_WriteByte(ADXL345_REG_POWER_CTL, power_ctl);
}

/**
 * @brief  进入待机模式
 */
void ADXL345_StopMeasurement(void)
{
    uint8_t power_ctl = ADXL345_I2C_ReadByte(ADXL345_REG_POWER_CTL);
    power_ctl &= ~0x08U;  /* 清除MEASURE位 */
    ADXL345_I2C_WriteByte(ADXL345_REG_POWER_CTL, power_ctl);
}

/**
 * @brief  进入休眠模式
 */
void ADXL345_EnterSleep(void)
{
    uint8_t power_ctl = ADXL345_I2C_ReadByte(ADXL345_REG_POWER_CTL);
    power_ctl &= ~0x08U;  /* 清除MEASURE位 */
    power_ctl |= 0x04U;   /* 设置SLEEP位 */
    ADXL345_I2C_WriteByte(ADXL345_REG_POWER_CTL, power_ctl);
}

/**
 * @brief  软件复位
 * @note   向POWER_CTL寄存器写入0xD7，芯片内部触发复位
 *         复位后寄存器恢复默认值，需等待至少1ms后重新配置
 */
void ADXL345_SoftReset(void)
{
    ADXL345_I2C_WriteByte(ADXL345_REG_POWER_CTL, 0xD7U);  /* 写入0xD7触发软件复位 */
    HAL_Delay(2);  /* 等待复位完成（手册要求>1ms）*/
}
