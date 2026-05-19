#include "BGT24MTR11.h"

uint16_t BGT24MTR11_ShadowReg = 0U;

static void BGT_SPI_Write16(uint16_t word)
{
    /* CLK空闲为低，CS拉低开始传输,t_CS_lead ≥ 20ns，GPIO操作本身已满足*/
    BGT_CLK_LOW();
    BGT_CS_LOW();

    /* 逐位发送，MSB先发（Bit15 → Bit0）*/
    for (int8_t bit = 15; bit >= 0; bit--) {
        /* 在CLK下降沿前准备好数据（t_SI_s`u ≥ 10ns）*/
        if (word >> bit & 0x1U) BGT_DATA_HIGH();   
        else BGT_DATA_LOW();

        /* Step2：CLK上升沿（数据已稳定）*/
        BGT_CLK_HIGH();
        /* Step3：CLK下降沿，BGT在此刻采样MOSI */
        BGT_CLK_LOW();
    }

    /* 16位发完，CS拉高：BGT把移位寄存器内容写入配置寄存器
     * t_CS_lag ≥ 20ns，GPIO操作已满足
     * CLK已回到低电平空闲状态，与ADF4153A的空闲状态一致 */
    BGT_CS_HIGH();
}

static uint16_t BGT_BuildReg(const BGT24MTR11_Config_t *cfg)
{
    uint16_t word = 0U;

    /* Bit15：GS，LNA增益降低 */
    if (cfg->gs) word |= BGT_BIT_GS;
    /* Bit14,13：未使用，写0（已初始化为0）*/
    /* Bit12：DIS_PA，TX发射功能由TXOFF控制，但此为需置零*/
    if (cfg->tx_disable) word |= BGT_BIT_DIS_PA;
    /* AMUX2→Bit11，AMUX1→Bit8，AMUX0→Bit7 */
    uint8_t amux = (uint8_t)cfg->amux & 0x07U;
    if (amux & 0x04U) word |= BGT_BIT_AMUX2;
    if (amux & 0x02U) word |= BGT_BIT_AMUX1;
    if (amux & 0x01U) word |= BGT_BIT_AMUX0;
    /* Bit10,9：Test位，必须写0（已初始化为0，绝对不能置1）*/
    /* Bit6：DIS_DIV64K，禁用Q2分频 */
    if (cfg->dis_div64k) word |= BGT_BIT_DIS_DIV64K;
    /* Bit5：DIS_DIV16，禁用Q1分频 */
    if (cfg->dis_div16) word |= BGT_BIT_DIS_DIV16;
    /* Bit4：PC2_BUF，LO Buffer功率 */
    if (cfg->lo_buf_high_pwr) word |= BGT_BIT_PC2_BUF;
    /* Bit3：PC1_BUF，TX Buffer功率 */
    if (cfg->tx_buf_high_pwr) word |= BGT_BIT_PC1_BUF;
    /* PC2_PA→Bit2，PC1_PA→Bit1，PC0_PA→Bit0 */
    word |= (uint16_t)(cfg->tx_pa_level & 0x07U);

    return word;
}

void BGT24MTR11_Init(const BGT24MTR11_Config_t *cfg)
{
    uint16_t reg = BGT_BuildReg(cfg);
    BGT24MTR11_ShadowReg = reg;
    BGT_SPI_Write16(reg);
}

/* 动态切换ANA引脚输出内容（只修改AMUX位）,用于调试：切换读TX功率、LO功率或温度 */
void BGT24MTR11_SetAmux(BGT_AmuxSel_t sel)
{
    /* 清除旧AMUX位 */
    BGT24MTR11_ShadowReg &= ~(BGT_BIT_AMUX2 | BGT_BIT_AMUX1 | BGT_BIT_AMUX0);
    uint8_t amux = (uint8_t)sel & 0x07U;
    if (amux & 0x04U) BGT24MTR11_ShadowReg |= BGT_BIT_AMUX2;
    if (amux & 0x02U) BGT24MTR11_ShadowReg |= BGT_BIT_AMUX1;
    if (amux & 0x01U) BGT24MTR11_ShadowReg |= BGT_BIT_AMUX0;
    BGT_SPI_Write16(BGT24MTR11_ShadowReg);
}

