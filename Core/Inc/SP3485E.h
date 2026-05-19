#ifndef __SP3485E_H
#define __SP3485E_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 *  SP3485E RS-485收发器驱动
 *  芯片：UMW SP3485E（3.3V半双工RS-485，最高12Mbps）
 *  MCU接口：UART4 + 1个GPIO控制方向
 *
 *  /RE与DE短接为一个引脚（DIR）：
 *    DIR = HIGH→发送模式（DE=1使能驱动，/RE=1关闭接收）
 *    DIR = LOW →接收模式（DE=0关闭驱动，/RE=0使能接收）
 *
 *  关键时序（来自手册Table 8、Table 9）：
 *    发送使能建立时间 tPZH/tPZL max = 90ns → GPIO拉高后可立即发UART
 *    发送完成切换时间 tPHZ max = 80ns       → TC中断后需等总线空闲
 *
 *  必须用UART的TC（Transmission Complete）中断判断发送完成，不能用TXE（TX Empty），
 *  TXE触发时移位寄存器内还有最后一位数据未发出，此时切换DIR会截断最后一个字节。
 * ============================================================ */

#define SP3485E_EN_Pin GPIO_PIN_4
#define SP3485E_EN_GPIO_Port GPIOC
#define RS485_RX_BUF_SIZE   256         /* 接收环形缓冲区大小，必须为2的幂 */
#define RS485_TX_TIMEOUT_MS 100         /* 发送超时（ms） */

extern UART_HandleTypeDef huart4;

#define RS485_SwitchToTX()  HAL_GPIO_WritePin(SP3485E_EN_GPIO_Port, SP3485E_EN_Pin, GPIO_PIN_SET)
#define RS485_SwitchToRX()  HAL_GPIO_WritePin(SP3485E_EN_GPIO_Port, SP3485E_EN_Pin, GPIO_PIN_RESET)

/* 接收状态 */
typedef enum{
    RS485_OK        = 0,
    RS485_TIMEOUT   = 1,
    RS485_OVERFLOW  = 2,
    RS485_ERROR     = 3,
}RS485_Status_t;

typedef struct{
    /* 接收环形缓冲区相关变量 */
    uint8_t  rx_buf[RS485_RX_BUF_SIZE];
    uint16_t rx_head;       /* 写指针（由UART中断更新） */
    uint16_t rx_tail;       /* 读指针（由用户读取更新） */

    /* 发送状态标志位 */
    volatile bool tx_busy;  /* 标志着此时正在发送给 */
    volatile bool tx_done;  /* TC中断完成置位，标志着发送完成 */
}RS485_Handle_t;

extern RS485_Handle_t g_rs485;

/* 初始化：配置DIR引脚为接收模式，启动UART接收中断 */
void RS485_Init(void);

/* 发送数据（阻塞，等待TC完成后切回接收模式）
 * data: 发送缓冲区指针
 * len:  发送字节数
 * 返回：RS485_OK / RS485_TIMEOUT */
RS485_Status_t RS485_Send(const uint8_t *data, uint16_t len);

/* 发送字符串（NULL结尾）*/
RS485_Status_t RS485_SendStr(const char *str);

/* 查询接收缓冲区中可读字节数 */
uint16_t RS485_Available(void);

/* 从接收缓冲区读取数据，返回实际读取字节数
 * buf:     目标缓冲区
 * max_len: 最多读取字节数 */
uint16_t RS485_Read(uint8_t *buf, uint16_t max_len);

int16_t RS485_ReadByte(void);

/* 清空接收缓冲区 */
void RS485_FlushRx(void);

#endif 
