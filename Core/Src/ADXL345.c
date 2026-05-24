/**
 ******************************************************************************
 * @file    ADXL345.c
 * @brief   ADXL345 三轴数字加速度计驱动源文件
 *          使用软件I2C接口与STM32F405RGT6通信
 *
 * @实现说明
 *   - 使用GPIO模拟I2C时序进行通信（SCL=PC0, SDA=PC1）
 *   - 支持7位从地址：0x53（SDO=GND）或 0x1D（SDO=VCC）
 *   - 支持多字节读写，地址自动递增
 *   - 不依赖硬件I2C外设，PC0/PC1按开漏输出配置并配合上拉电阻使用
 ******************************************************************************
 */

#include "ADXL345.h"

/* ============================================================
 *  模块级变量
 * ============================================================ */
/* 当前配置缓存，用于计算g值转换 */
static ADXL345_Range_t s_current_range = ADXL345_RANGE_2G;
static bool s_full_resolution = false;

/* 当前I2C 7位地址 */
static uint8_t s_i2c_addr = ADXL345_I2C_ADDR_LOW;

/* ============================================================
 *  STM32的每个GPIO口都可以直接写BSRR寄存器来设置高低电平
 *  0~15位对应置位，引脚输出高电平；16~31位对应复位，引脚输出低电平
 * ============================================================ */
#define ADXL345_SCL_LOW()      (ADXL345_SCL_GPIO_Port->BSRR = (uint32_t)ADXL345_SCL_Pin << 16U)
#define ADXL345_SCL_RELEASE()  (ADXL345_SCL_GPIO_Port->BSRR = (uint32_t)ADXL345_SCL_Pin)
#define ADXL345_SDA_LOW()      (ADXL345_SDA_GPIO_Port->BSRR = (uint32_t)ADXL345_SDA_Pin << 16U)
#define ADXL345_SDA_RELEASE()  (ADXL345_SDA_GPIO_Port->BSRR = (uint32_t)ADXL345_SDA_Pin)

#define ADXL345_SDA_READ()     ((ADXL345_SDA_GPIO_Port->IDR & ADXL345_SDA_Pin) != 0U)
#define ADXL345_SCL_READ()     ((ADXL345_SCL_GPIO_Port->IDR & ADXL345_SCL_Pin) != 0U)

static void ADXL345_I2C_Delay(void)
{
    volatile uint32_t i = 700U;
    while (i-- > 0U) {
        __NOP();
    }
}

static HAL_StatusTypeDef ADXL345_I2C_WaitSclHigh(void)
{
    uint32_t timeout = ADXL345_TIMEOUT_MS * 1000U;

    while (!ADXL345_SCL_READ()) {
        if (timeout-- == 0U) {
            return HAL_TIMEOUT;
        }
        __NOP();
    }

    return HAL_OK;
}

static HAL_StatusTypeDef ADXL345_I2C_Start(void)
{
    ADXL345_SDA_RELEASE();
    ADXL345_SCL_RELEASE();
    if (ADXL345_I2C_WaitSclHigh() != HAL_OK) {
        return HAL_TIMEOUT;
    }
    ADXL345_I2C_Delay();
    ADXL345_SDA_LOW();
    ADXL345_I2C_Delay();
    ADXL345_SCL_LOW();
    ADXL345_I2C_Delay();

    return HAL_OK;
}

static void ADXL345_I2C_Stop(void)
{
    ADXL345_SDA_LOW();
    ADXL345_I2C_Delay();
    ADXL345_SCL_RELEASE();
    (void)ADXL345_I2C_WaitSclHigh();
    ADXL345_I2C_Delay();
    ADXL345_SDA_RELEASE();
    ADXL345_I2C_Delay();
}

static HAL_StatusTypeDef ADXL345_I2C_WriteRawByte(uint8_t data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        /* MSB，主机准备当前数据位*/
        if ((data & 0x80U) != 0U) {
            ADXL345_SDA_RELEASE();
        } else {
            ADXL345_SDA_LOW();
        }
        data <<= 1U;

        ADXL345_I2C_Delay(); /* 对应数据建立时间，保证SDA在SCL有效采样前稳定*/
        /* 产生SCL高电平，并确认SCL确实升高*/
        ADXL345_SCL_RELEASE();
        if (ADXL345_I2C_WaitSclHigh() != HAL_OK) {
            return HAL_TIMEOUT;
        }
        ADXL345_I2C_Delay(); /* 保持SCL高电平，让ADXL345采样SDA*/
        ADXL345_SCL_LOW();   /* 结束当前bit传输*/
        ADXL345_I2C_Delay(); /* 保持SCL低电平，为下一位准备*/
    }
    /* 读取从机发送的ACK*/
    ADXL345_SDA_RELEASE(); /* 主机释放SDA，恢复默认高电平*/
    ADXL345_I2C_Delay(); /* ACK位建立时间*/
    ADXL345_SCL_RELEASE(); /* 主机释放SCL，产生第9个时钟*/
    if (ADXL345_I2C_WaitSclHigh() != HAL_OK) {
        return HAL_TIMEOUT;
    }   
    ADXL345_I2C_Delay(); /* 在SCL高电平期间读取SDA */
    /* 如果SDA为高电平，说明从机没有ACK */
    if (ADXL345_SDA_READ()) {
        ADXL345_SCL_LOW();
        ADXL345_I2C_Delay();
        return HAL_ERROR;
    }

    ADXL345_SCL_LOW(); /*主机拉低SCL，结束ACK周期*/
    ADXL345_I2C_Delay();

    return HAL_OK;
}

static HAL_StatusTypeDef ADXL345_I2C_ReadRawByte(uint8_t *data, bool ack)
{
    uint8_t i;
    uint8_t value = 0U;

    ADXL345_SDA_RELEASE(); /* 从机ADXL345输出数据*/

    for (i = 0U; i < 8U; i++) {
        value <<= 1U;
        ADXL345_SCL_RELEASE(); /* 主机释放SCL，产生时钟*/
        if (ADXL345_I2C_WaitSclHigh() != HAL_OK) {
            return HAL_TIMEOUT;
        }
        ADXL345_I2C_Delay();
        if (ADXL345_SDA_READ()) { /* 从机ADXL345在SCL高电平期间拉低释放数据线*/
            value |= 0x01U;
        }
        ADXL345_SCL_LOW(); /* 主机拉低SCL，结束这一位*/
        ADXL345_I2C_Delay(); /*保持SCL低电平 */
    }

    if (ack) { /* 主机准备发送ACK/NACK*/
        ADXL345_SDA_LOW(); /* 发送ACK*/
    } else {
        ADXL345_SDA_RELEASE(); /* 发送NACK*/
    }

    ADXL345_I2C_Delay();
    ADXL345_SCL_RELEASE(); /*读取主机的ACK/NACK*/
    if (ADXL345_I2C_WaitSclHigh() != HAL_OK) {
        return HAL_TIMEOUT;
    }
    ADXL345_I2C_Delay();
    ADXL345_SCL_LOW();
    ADXL345_SDA_RELEASE();
    ADXL345_I2C_Delay();

    *data = value;

    return HAL_OK;
}

/**
 * @brief  I2C写一个字节
 * @param  addr 寄存器地址
 * @param  data 要写入的数据
 * @retval HAL状态
 */
static HAL_StatusTypeDef ADXL345_I2C_WriteByte(uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;

    status = ADXL345_I2C_Start();
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte((uint8_t)(s_i2c_addr << 1U));
    }
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte(addr);
    }
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte(data);
    }

    ADXL345_I2C_Stop();
    return status;
}

/**
 * @brief  I2C读一个字节
 * @param  addr 寄存器地址
 * @retval 读取的数据
 */
static uint8_t ADXL345_I2C_ReadByte(uint8_t addr)
{
    uint8_t data = 0U;

    if (ADXL345_I2C_Start() == HAL_OK &&
        ADXL345_I2C_WriteRawByte((uint8_t)(s_i2c_addr << 1U)) == HAL_OK &&  /** 写入设备地址+写标志 */
        ADXL345_I2C_WriteRawByte(addr) == HAL_OK &&
        ADXL345_I2C_Start() == HAL_OK &&
        ADXL345_I2C_WriteRawByte((uint8_t)((s_i2c_addr << 1U) | 0x01U)) == HAL_OK) { /* 写入设备地址+读标志 */
        (void)ADXL345_I2C_ReadRawByte(&data, false);
    }

    ADXL345_I2C_Stop();
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
    HAL_StatusTypeDef status;
    uint8_t i;

    if ((buf == NULL) || (len == 0U)) {
        return HAL_ERROR;
    }

    status = ADXL345_I2C_Start();
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte((uint8_t)(s_i2c_addr << 1U));
    }
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte(addr);
    }
    if (status == HAL_OK) {
        status = ADXL345_I2C_Start();
    }
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte((uint8_t)((s_i2c_addr << 1U) | 0x01U));
    }

    for (i = 0U; (status == HAL_OK) && (i < len); i++) {
        status = ADXL345_I2C_ReadRawByte(&buf[i], (i + 1U) < len);
    }

    ADXL345_I2C_Stop();
    return status;
}

/**
 * @brief  I2C多字节写
 * @param  addr 起始寄存器地址
 * @param  buf 数据缓冲区
 * @param  len 要写入的字节数
 */
static void ADXL345_I2C_WriteMulti(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t i;

    if ((buf == NULL) || (len == 0U)) {
        return;
    }

    status = ADXL345_I2C_Start();
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte((uint8_t)(s_i2c_addr << 1U));
    }
    if (status == HAL_OK) {
        status = ADXL345_I2C_WriteRawByte(addr);
    }
    for (i = 0U; (status == HAL_OK) && (i < len); i++) {
        status = ADXL345_I2C_WriteRawByte(buf[i]);
    }

    ADXL345_I2C_Stop();
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
    uint8_t offset_data[3];

    if (cfg == NULL) {
        return false;
    }

    /* 根据配置设置I2C 7位地址 */
    s_i2c_addr = cfg->addr_high ? ADXL345_I2C_ADDR_HIGH : ADXL345_I2C_ADDR_LOW;

    /* 读取设备ID，验证通信是否正常 */
    id = ADXL345_I2C_ReadByte(ADXL345_REG_DEVID);
    if (id != ADXL345_DEVICE_ID) {
        return false;  /* 通信失败或芯片不在线 */
    }

    data_format = (uint8_t)(cfg->range & 0x03U);
    if (cfg->full_res) {
        data_format |= 0x08U;  /* FULL_RES = 1 */
    }
    ADXL345_I2C_WriteByte(ADXL345_REG_DATA_FORMAT, data_format);

    /* 缓存配置用于后续g值转换 */
    s_current_range = cfg->range;
    s_full_resolution = cfg->full_res;

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

    /* At power-up, the device is in standby mode, awaiting a command to enter measurement mode
     * This command can be initiated by setting the measure bit(Bit D3) in the POWER_CTL register(0x2D)
     * It is recommended to configure the device in standby mode and then to enable measurement mode.*/
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
 *         10位模式：根据量程不同，精度不同
 *           ±2g  → 4mg/LSB    (256 LSB/g)
 *           ±4g  → 7.8mg/LSB  (128 LSB/g)
 *           ±8g  → 15.6mg/LSB (64 LSB/g)
 *           ±16g → 31.2mg/LSB (32 LSB/g)
 */
float ADXL345_RawToG(int16_t raw, ADXL345_Range_t range, bool full_res)
{
    float scale;

    if (full_res) {
        scale = 0.004f;  /* 4mg = 0.004g */
    } else {
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
