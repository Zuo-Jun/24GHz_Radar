#include "SP3485E.h"

/*  RS-485与普通串口的本质区别：
 *    普通UART：单端TTL，TX引脚输出0~3.3V的电压，GND作为参考。
 *	  信号抗干扰能力弱，传输距离短(通常不超过几米)，也不支持多设备挂在同一根线上。
 *	  直接调用HAL_UART_Transmit即可。
 *
 *    RS-485是差分信号，用A和B两根线，接收方判断的是A-B的电压差而不是绝对电压。
 *    手册里写的判决门限是±200mV，也就是说即使两根线上都叠加了几伏的共模噪声，
 *    只要A-B差值还能保持在200mV以上，接收就不会出错。这是它抗干扰强的根本原因。
 *    传输距离可以达到1200米，最多可以挂256个设备在同一条总线上（SP3485E的1/8单位负载特性）。*/

RS485_Handle_t g_rs485 ={
	.rx_buf={0},
	.rx_head=0,
	.rx_tail=0,
	.tx_busy=false,
	.tx_done=false,
};

/* 接收中断数据缓存区(每次接收到1字节先存储在这，在搬运到环形缓冲) */
static uint8_t s_rx_byte=0;

/* 用位运算代替取模运算，&(SIZE-1)等价于%SIZE，但运算速度更快 */
static inline uint16_t rbuf_next(uint16_t idx) 
{
	return (idx+1)&(RS485_RX_BUF_SIZE-1);
}

static inline bool rbuf_empty(void)
{
	return g_rs485.rx_head==g_rs485.rx_tail;
}

/* 牺牲一个空间来判断是空还是满 */
static inline bool rbuf_full(void)
{
	return rbuf_next(g_rs485.rx_head)==g_rs485.rx_tail;
}

/* 清空接收缓冲区 */
void RS485_FlushRx(void)
{
    g_rs485.rx_head = g_rs485.rx_tail = 0;
}

static inline void rbuf_push(uint8_t byte)
{
	if(!rbuf_full())
	{
		g_rs485.rx_buf[g_rs485.rx_head]=byte;
		g_rs485.rx_head=rbuf_next(g_rs485.rx_head);
	}
	 /* 满时丢弃最新字节（不覆盖旧数据） */
}

/* 返回接收缓冲区中可读字节数 */
uint16_t RS485_Available(void)
{
    return (uint16_t)((g_rs485.rx_head-g_rs485.rx_tail)&(RS485_RX_BUF_SIZE-1));
}

/* 缓冲区若不为空，则根据rx_tail读取缓冲区内对应的一个字节返回，并将rx_tail移动一位 */
static inline int16_t rbuf_pop(void)
{
    if (rbuf_empty()) return -1;
    uint8_t byte = g_rs485.rx_buf[g_rs485.rx_tail];
    g_rs485.rx_tail = rbuf_next(g_rs485.rx_tail);
    return (int16_t)byte;
}

/* 从接收缓冲区读取数据，返回实际读取字节数 */
uint16_t RS485_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len) {
        int16_t byte = rbuf_pop();
        if (byte < 0) break;
        buf[count++] = (uint8_t)byte;
    }
    return count;
}

int16_t RS485_ReadByte(void)
{
	return rbuf_pop();
}

void RS485_Init(void)
{
	/* 清空驱动句柄 */
	memset(&g_rs485, 0, sizeof(RS485_Handle_t));
	RS485_SwitchToRX();
	/* 开启接收中断 */
	HAL_UART_Receive_IT(&huart4,&s_rx_byte,1);
}

/* 每收到1字节触发,把字节推入环形缓冲区，然后重新开启下一字节接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        /* 发送期间UART RX会收到自己的回波，丢弃 */
        if (!g_rs485.tx_busy) {
            rbuf_push(s_rx_byte);
        }
        /* 重启接收中断 */
        HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
    }
}

/* =======================上方为接收，下方为发送========================== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        g_rs485.tx_done = true; /* 置位tx_done，Send()里的等待循环会退出 */
    }
}

/*	流程：
 *    1. 等待上一次发送完成（如果有）
 *    2. 切换到发送模式
 *    3. 启动UART中断发送
 *    4. 等待TC中断（g_rs485.tx_done）
 *    5. 切换回接收模式
 *    6. 重新启动UART接收中断 */
RS485_Status_t RS485_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return RS485_ERROR;

    /* 等待上一次发送完成（超时保护） */
    uint32_t t0 = HAL_GetTick();
    while (g_rs485.tx_busy) {
        if ((HAL_GetTick() - t0) > RS485_TX_TIMEOUT_MS) {
            /* 超时：强制复位发送状态 */
            g_rs485.tx_busy = false;
            g_rs485.tx_done = false;
            RS485_SwitchToRX();
            HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
            return RS485_TIMEOUT;
        }
    }

    /* 切换到发送模式
     * tPZH max = 90ns，UART起始位时间（115200bps时约8.68us）远大于此，不需要额外延时 */
    g_rs485.tx_busy = true;
    g_rs485.tx_done = false;
    /* 停止接收中断，防止发送期间自收（半双工总线上发送数据会回到RX） */
    HAL_UART_AbortReceive(&huart4);
    RS485_SwitchToTX();

    /* 启动中断发送 */
    if (HAL_UART_Transmit_IT(&huart4, (uint8_t *)data, len)!= HAL_OK) {
        RS485_SwitchToRX();
        g_rs485.tx_busy = false;
        HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
        return RS485_ERROR;
    }

    /* 等待TC中断（发送完成回调置位tx_done）
     * TC表示移位寄存器已完全空，最后一位已发出，此时切换DIR安全 */
    t0 = HAL_GetTick();
    while (!g_rs485.tx_done) {
        if ((HAL_GetTick() - t0) > RS485_TX_TIMEOUT_MS) {
            HAL_UART_AbortTransmit(&huart4);
            RS485_SwitchToRX();
            g_rs485.tx_busy = false;
            g_rs485.tx_done = false;
            HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
            return RS485_TIMEOUT;
        }
    }

    /* TC完成后切回接收模式
     * tPHZ max = 80ns，HAL函数调用本身已远超此时间，无需额外延时 */
    RS485_SwitchToRX();
    g_rs485.tx_busy = false;
    g_rs485.tx_done = false;
    /* 重新启动UART接收中断 */
    HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
    return RS485_OK;
}

/* 发送NULL结尾字符串 */
RS485_Status_t RS485_SendStr(const char *str)
{
    if (str == NULL) return RS485_ERROR;
    uint16_t len = (uint16_t)strlen(str);
    return RS485_Send((const uint8_t *)str, len);
}


