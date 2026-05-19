#ifndef __ADF4153A_H
#define __ADF4153A_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* ============================================================
 *  ADF4153A 驱动头文件
 *  适用芯片：ADF4153A (Analog Devices 分数N锁相环)
 *  配套硬件：BGT24MTR11 24GHz雷达MMIC
 *  参考输入：25MHz TCXO/晶振
 *  RF输入：来自BGT24MTR11 Q1/Q1N ÷16预分频器≈1.5GHz
 *
 *  寄存器总览（每个24位，DB1:DB0为寄存器选择位）
 *   R0 (DB1:DB0=00) — N分频寄存器                       
 *    DB23      FASTLOCK     快速锁定使能                 
 *    DB22:14   INT[8:0]     整数分频 31~511              
 *    DB13:2    FRAC[11:0]   小数分子 0~MOD-1             
 *    DB1:0     C2C1=00      寄存器选择                   
 *   R1 (DB1:DB0=01) — R分频寄存器                       
 *    DB23      LOAD_CONTROL 相位重同步载入控制           
 *    DB22:20   MUXOUT[2:0]  MUXOUT引脚功能               
 *    DB19      RESERVED     保留，写0                    
 *    DB18      PRESCALER    0=4/5(≤2GHz) 1=8/9(≤4GHz)  
 *    DB17:14   R[3:0]       参考分频 1~15                
 *    DB13:2    MOD[11:0]    小数分母 2~4095              
 *    DB1:0     C2C1=01      寄存器选择                   
 *   R2 (DB1:DB0=10) — 控制寄存器                        
 *    DB15:12   RESYNC[3:0]  相位重同步计数 0=禁用        
 *    DB11      REFIN_DBL    参考倍频 0=关 1=开(≤30MHz)  
 *    DB10:7    CP[3:0]      电荷泵电流档 0~15            
 *    DB6       PD_POL       PD极性 1=正(VCO正调谐斜率)   
 *    DB5       LDP          锁定检测精度 0=24T 1=40T     
 *    DB4       POWER_DOWN   软件掉电 1=掉电              
 *    DB3       CP_3STATE    CP三态 1=高阻                
 *    DB2       CNT_RESET    计数器复位 1=复位            
 *    DB1:0     C2C1=10      寄存器选择                   
 *   R3 (DB1:DB0=11) — 噪声毛刺寄存器                      
 *    DB10      RESERVED     保留，写0                    
 *    DB9:6     T8..T5       噪声模式高4位                
 *    DB5:3     RESERVED     保留，写0                   
 *    DB2       T1           噪声模式低1位                
 *    DB1:0     C2C1=11      寄存器选择                    
 * ============================================================ */

#define ADF_LE_Pin GPIO_PIN_13
#define ADF_LE_GPIO_Port GPIOB
#define ADF_MUXOUT_Pin GPIO_PIN_0
#define ADF_MUXOUT_GPIO_Port GPIOB
#define SoftSPI_DATA_Pin GPIO_PIN_8
#define SoftSPI_DATA_GPIO_Port GPIOB
#define SoftSPI_CLK_Pin GPIO_PIN_9
#define SoftSPI_CLK_GPIO_Port GPIOB

#define ADF_LE_LOW()    HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_RESET)
#define ADF_LE_HIGH()   HAL_GPIO_WritePin(ADF_LE_GPIO_Port, ADF_LE_Pin, GPIO_PIN_SET)
#define ADF_DATA_LOW()		HAL_GPIO_WritePin(SoftSPI_DATA_GPIO_Port, SoftSPI_DATA_Pin, GPIO_PIN_RESET)
#define ADF_DATA_HIGH()		HAL_GPIO_WritePin(SoftSPI_DATA_GPIO_Port, SoftSPI_DATA_Pin, GPIO_PIN_SET)
#define ADF_CLK_LOW()		HAL_GPIO_WritePin(SoftSPI_CLK_GPIO_Port, SoftSPI_CLK_Pin, GPIO_PIN_RESET)
#define ADF_CLK_HIGH()		HAL_GPIO_WritePin(SoftSPI_CLK_GPIO_Port, SoftSPI_CLK_Pin, GPIO_PIN_SET)

/* ---------- 系统频率常数（单位：Hz） ---------- */
#define ADF_REFIN_HZ        25000000UL   /* 外部参考：25MHz晶振 */
#define BGT_PRESCALER_DIV   16           /* BGT24 Q1输出 = VCO/16 */
 
/* ---------- R1 MUXOUT 配置枚举 ---------- */
typedef enum {
    MUXOUT_THREE_STATE    = 0,  /* 000:高阻 */
	MUXOUT_DLOCK_DIGITAL  = 1,  /* 001:数字锁定检测（最常用）*/
	MUXOUT_N_DIV_OUT      = 2,  /* 010:N分频器输出 */
	MUXOUT_LOGIC_HIGH     = 3,  /* 011:恒高 */
	MUXOUT_R_DIV_OUT      = 4,  /* 100:R分频器输出 */
	MUXOUT_ALOCK_ANALOG   = 5,  /* 101:模拟锁定检测 */
	MUXOUT_FASTLOCK_SW    = 6,  /* 110:只在外部滤波器需要动态切换阻尼电阻时才用*/
	MUXOUT_LOGIC_LOW      = 7   /* 111:恒低 */
} ADF_MuxoutMode_t;
 
/* ---------- R3 噪声/毛刺模式枚举 ---------- */
typedef enum {
    NOISE_SPUR_LOW_SPUR       = 0x00,  /* 低毛刺（dither开，底噪高~10dB，无杂散峰） */
    NOISE_SPUR_LOW_NOISE_SPUR = 0x1C,  /* 平衡：dither关，噪声与毛刺折中 */
    NOISE_SPUR_LOWEST_NOISE   = 0x1F   /* 最低噪声（dither关+CP优化，有杂散峰但被窄带滤波器衰减） */
} ADF_NoiseSpur_t;
 

typedef struct {
    /* ---------- R0：N分频 ---------- */
    uint8_t  fastlock_en;  /* 使能为1时，CP电流为最大值；为0时为寄存器设定值*/
    uint32_t INT;        /* 整数分频值 91..511（prescaler=4/5时最小31;prescaler=8/9时最小91）*/
    uint32_t FRAC;       /* 小数分子 0..MOD-1 */
	
    /* ---------- R1：R分频 ---------- */
	uint8_t resync_en;   /* 高电平使能，装载相位重同步延迟值;低电平正常分频 */
	ADF_MuxoutMode_t  muxout;      /* MUXOUT引脚功能 */
	uint8_t  prescaler;  /* 0=4/5（≤2GHz），1=8/9（≤4GHz）*/
	uint8_t  R;          /* 参考分频 1..15，决定PFD频率 */
    uint32_t MOD;        /* 小数分母 2..4095，low-spur模式下≥50 */

/* ============================================================
*   Phase Resync解决的问题是：同一个频率，每次PLL锁定后VCO的相位起点是随机的。
*	雷达测距和测速依赖的是发射信号和接收回波之间的相对频率差或相位差，
*	不是VCO相对于参考晶振的绝对相位。
*	每次测量都是一个独立的完整过程：发射、接收、做FFT、得到峰值频率。
*	这个峰值频率只和目标的距离或速度有关，和VCO相位从哪里起步无关。
*	需要Phase Resync的典型场景是雷达阵列里多个收发通道必须相位对齐，
*	或者要把两次不同时间的测量结果做相干叠加（合成孔径雷达之类）。
* ============================================================ */
 
	/* ---------- R2：控制 ---------- */
	/* Phase Resync（相位重同步通常不需要，设0禁用）*/
    uint8_t  resync;   /* RESYNC值 0=禁用，1..15=启用 */
    /* resync_delay 写R1时LOAD_CONTROL=1时MOD位置放延迟值，通常不用 */
    uint8_t  refin_doubler;  /* 0=不倍频，1=倍频（输入参考信号超过30MHz不能倍频）*/
    uint8_t  cp_current;     /* 电荷泵电流档 0..15，见手册Table对应RSET */
    uint8_t  pd_polarity;    /* 1=正极性（VCO正调谐斜率时用此值） */
    uint8_t  ldp;            /* 0=24PFD周期，1=40PFD周期锁定判定 */
    /* power_down/cp_three_state/counter_reset 由驱动内部管理，不放入配置结构体 */
	
    /* ---------- R3：噪声毛刺 ---------- */
    ADF_NoiseSpur_t   noise_spur;  /* 噪声/毛刺模式 */
	
} ADF4153A_Config_t;
 
/* -------- 影子寄存器：保留R2用于PowerDown位操作 -------- */
/* R0/R1每次写都完整重算，不需要读回；R2的PowerDown需要改单位 */
extern uint32_t ADF4153A_R2_Shadow;
 
/* ---------- API 函数声明 ---------- */
/* 初始化：按手册六步序列写入所有寄存器，等待锁定 */
void ADF4153A_Init(const ADF4153A_Config_t *cfg); 

/* 动态修改频率（只写R1+R0，保持其他寄存器不变）
 * MOD双缓冲：内部先写R1再写R0，两者必须同时传入 */
void ADF4153A_SetFrequency(uint32_t INT, uint32_t FRAC, uint32_t MOD);

/* 等待数字锁定检测（MUXOUT需配置为MUXOUT_DLOCK_DIGITAL）
 * 返回true=锁定，false=超时 */
bool ADF4153A_WaitLock(uint32_t timeout_ms);

/* 软件掉电（修改R2 DB4，保持其他位不变，依赖R2_Shadow） */
void ADF4153A_PowerDown(bool enable);
 
/* 频率计算辅助函数 */
double ADF4153A_CalcFpfd(double refin_hz, uint8_t doubler, uint8_t R); /* Fpfd不能超过32MHz*/
double ADF4153A_CalcFres(double refin_hz, uint8_t doubler, uint8_t R, uint32_t MOD);
double ADF4153A_CalcRFout(double refin_hz, uint8_t doubler, uint8_t R,
                             uint32_t MOD, uint32_t INT, uint32_t FRAC);
 
#endif

