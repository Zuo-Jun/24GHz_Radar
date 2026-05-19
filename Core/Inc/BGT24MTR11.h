#ifndef __BGT24MTR11_H
#define __BGT24MTR11_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ============================================================
 *  【寄存器说明】（Page17的Table 11，16位SPI字，MSB先发）
 *
 *   Bit15  GS        LNA增益降低：1=降低约5dB，0=正常增益 
 *   Bit14  -         未使用，写0                         
 *   Bit13  -         未使用，写0                         
 *   Bit12  DIS_PA    TX发射禁用：0=允许发射，1=禁止TX    
 *   Bit11  AMUX2     模拟MUX选择位2（见Table 13）        
 *   Bit10  Test      必须写0，否则芯片误动作             
 *   Bit9   Test      必须写0，否则芯片误动作             
 *   Bit8   AMUX1     模拟MUX选择位1                     
 *   Bit7   AMUX0     模拟MUX选择位0                     
 *   Bit6   DIS_DIV64k  1=禁用÷65536分频（Q2输出）,0=使能         
 *   Bit5   DIS_DIV16   1=禁用÷16分频（Q1输出到ADF RF输入）,此位必须为0，否则ADF收不到反馈信号          
 *   Bit4   PC2_BUF   LO Buffer功率：1=高功率，0=低约4dB 
 *   Bit3   PC1_BUF   TX Buffer功率：1=高功率            
 *   Bit2   PC2_PA    TX PA功率调节位2（见下表）          
 *   Bit1   PC1_PA    TX PA功率调节位1                   
 *   Bit0   PC0_PA    TX PA功率调节位0                    
 *
 *  TX PA功率调节（PC2/PC1/PC0_PA，手册SPI-Bit调节范围约9dB）：
 *    111 → 最大功率（约+11dBm，默认上电值）
 *    110 → 降低约1.5dB
 *    101 → 降低约3dB
 *    ...（依次递减）
 *    000 → 最小功率
 *
 *  AMUX真值表（Table 13，ANA引脚输出选择）：
 *    AMUX2/1/0 = 000 → ANA = VOUT_TX（TX功率传感器）
 *    AMUX2/1/0 = 001 → ANA = VREF_TX（TX参考电压）
 *    AMUX2/1/0 = 010 → ANA = VOUT_LO（LO功率传感器）
 *    AMUX2/1/0 = 011 → ANA = VREF_LO（LO参考电压）
 *    AMUX2/1/0 = 100 → ANA = VTEMP（片内温度传感器）
 *    AMUX2/1/0 = 101 → ANA = Test_Signal1（测试用）
 *
 *  【上电默认值】（Power On State，Table 11）：
 *    DIS_PA=1（TX禁用），AMUX=100（温度输出），
 *    PC2/1/0_PA=111（PA最大功率），DIS_DIV16=0（分频使能）
 *    → 上电后TX默认关闭，必须通过SPI写入才能开启发射
 *
 *  【SPI时序】（Page18的Table 12/Figure 3）：
 *    数据在CLK下降沿锁存（与ADF4153A的上升沿相反）
 *    CS低电平期间传输，CS上升沿把数据载入内部寄存器
 *    16位数据，MSB先发
 * ============================================================ */

#define SoftSPI_DATA_Pin GPIO_PIN_8
#define SoftSPI_DATA_GPIO_Port GPIOB
#define SoftSPI_CLK_Pin GPIO_PIN_9
#define SoftSPI_CLK_GPIO_Port GPIOB
#define BGT_CS_Pin GPIO_PIN_12
#define BGT_CS_GPIO_Port GPIOB
#define BGT_TXOFF_Pin GPIO_PIN_12
#define BGT_TXOFF_GPIO_Port GPIOA
#define BGT_ANA_Pin GPIO_PIN_4
#define BGT_ANA_GPIO_Port GPIOA

#define BGT_CLK_HIGH()     HAL_GPIO_WritePin(SoftSPI_CLK_GPIO_Port, SoftSPI_CLK_Pin, GPIO_PIN_SET)
#define BGT_CLK_LOW()      HAL_GPIO_WritePin(SoftSPI_CLK_GPIO_Port, SoftSPI_CLK_Pin, GPIO_PIN_RESET)
#define BGT_DATA_HIGH()    HAL_GPIO_WritePin(SoftSPI_DATA_GPIO_Port, SoftSPI_DATA_Pin, GPIO_PIN_SET)
#define BGT_DATA_LOW()     HAL_GPIO_WritePin(SoftSPI_DATA_GPIO_Port, SoftSPI_DATA_Pin, GPIO_PIN_RESET)
#define BGT_CS_HIGH()      HAL_GPIO_WritePin(BGT_CS_GPIO_Port, BGT_CS_Pin, GPIO_PIN_SET)
#define BGT_CS_LOW()       HAL_GPIO_WritePin(BGT_CS_GPIO_Port, BGT_CS_Pin, GPIO_PIN_RESET)

#define BGT_BIT_GS          (1U << 15)  /* LNA增益降低 */
#define BGT_BIT_DIS_PA      (1U << 12)  /* TX禁用 */
#define BGT_BIT_AMUX2       (1U << 11)  /* AMUX高位 */
#define BGT_BIT_AMUX1       (1U <<  8)  /* AMUX中位 */
#define BGT_BIT_AMUX0       (1U <<  7)  /* AMUX低位 */
#define BGT_BIT_DIS_DIV64K  (1U <<  6)  /* 禁用Q2分频 */
#define BGT_BIT_DIS_DIV16   (1U <<  5)  /* 禁用Q1分频（不能置1！）*/
#define BGT_BIT_PC2_BUF     (1U <<  4)  /* LO Buffer功率 */
#define BGT_BIT_PC1_BUF     (1U <<  3)  /* TX Buffer功率 */

typedef enum {
    BGT_AMUX_VOUT_TX  = 0x00,  /* TX功率传感器输出 */
    BGT_AMUX_VREF_TX  = 0x01,  /* TX参考电压 */
    BGT_AMUX_VOUT_LO  = 0x02,  /* LO功率传感器 */
    BGT_AMUX_VREF_LO  = 0x03,  /* LO参考电压 */
    BGT_AMUX_VTEMP    = 0x04,  /* 片内温度传感器 */
    BGT_AMUX_TEST1    = 0x05,  /* 测试信号1 */
} BGT_AmuxSel_t;

typedef struct {
    /* LNA增益 */
    uint8_t  gs;  				/* 0=正常增益，1=降低约5dB（目标很近时防饱和）*/
    /* TX控制 */
    uint8_t  tx_disable;        /* 0=允许发射，1=禁止TX */
    uint8_t  tx_pa_level;       /* PA功率级别 0~7，7=最大+11dBm，每级约1.5dB */
    uint8_t  tx_buf_high_pwr;   /* TX Buffer高功率：1=高，0=低约4dB */
    /* LO */
    uint8_t  lo_buf_high_pwr;   /* LO Buffer高功率：1=高，0=低约4dB */
    /* 分频器*/
    uint8_t  dis_div64k;        /* 0=使能Q1(÷16)输出，1=禁用（禁用后ADF无输入！）*/
    uint8_t  dis_div16;         /* 0=使能Q2(÷65536)输出，1=禁用 */
    /* 模拟MUX（ANA引脚输出选择）*/
    BGT_AmuxSel_t amux;      
} BGT24MTR11_Config_t;

/* 影子寄存器（断电丢失，重新上电需重写）*/
extern uint16_t BGT24MTR11_ShadowReg;

/* 初始化：写入配置寄存器，开启发射 */
void BGT24MTR11_Init(const BGT24MTR11_Config_t *cfg);

/* 切换AMUX输出（只修改AMUX位，其他位保持不变）*/
void BGT24MTR11_SetAmux(BGT_AmuxSel_t sel);

#endif
