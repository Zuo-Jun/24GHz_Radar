#ifndef RADAR_PROTOCOL_H
#define RADAR_PROTOCOL_H

#include "radar_frame.h"
#include <stdint.h>

/* ============================================================
 *  Radar_protocol 通信协议层
 *
 *    1. RadarProtocol_Init()          设置设备ID，SEQ清零
 *    2. RadarProtocol_GetSeq()        查询当前SEQ，便于调试
 *    3. RadarProtocol_SendData()      发送二进制测量帧，不等ACK
 *    4. RadarProtocol_SendDataWaitAck() 发送二进制测量帧，并等待ACK
 *    5. RadarProtocol_SendDebug()     发送ASCII调试帧
 *
 *  CRC、小端写入、payload打包、ACK解析等细节全部放在.c文件内部static函数中。
 *  这样主循环只需要关心“发送什么”，不用关心“协议内部怎么拼字节”。
 * ============================================================ */

/* ---------- status位定义 ----------
 *
 * 上位机应优先看status判断数据是否可信，而不是只看数值字段。
 *
 * bit0 / bit1:
 *   由RadarFrame_t中的water_level_valid、water_velocity_valid自动生成。
 *
 * bit2:
 *   ADXL345初始化成功后由协议层自动置位，表示姿态/倾角模块可用。
 *
 * bit3 / bit4:
 *   PLL、ADC等模块状态，当前通过extra_status传入。
 *   后续如果要自动维护这些位，需要修改其他模块代码，按约定单独审核。
 */
#define RADAR_STATUS_WATER_LEVEL_VALID    (1U << 0)
#define RADAR_STATUS_WATER_VELOCITY_VALID (1U << 1)
#define RADAR_STATUS_ATTITUDE_VALID       (1U << 2)
#define RADAR_STATUS_PLL_LOCK_OK          (1U << 3)
#define RADAR_STATUS_ADC_OK               (1U << 4)
#define RADAR_STATUS_SIGNAL_WEAK          (1U << 5)
#define RADAR_STATUS_RANGE_OVER_LIMIT     (1U << 6)
#define RADAR_STATUS_VELOCITY_ABNORMAL    (1U << 7)

/* ACK等待参数：
 *   第一阶段只发不等ACK时不用管；
 *   上位机能回ACK后，RadarProtocol_SendDataWaitAck()内部使用这两个参数。
 */
#define RADAR_PROTO_ACK_TIMEOUT_MS 500U
#define RADAR_PROTO_MAX_RETRY      3U

typedef enum {
    RADAR_PROTO_OK = 0,
    RADAR_PROTO_ERROR,
    RADAR_PROTO_TIMEOUT,
    RADAR_PROTO_RS485_ERROR,
} RadarProto_Status_t;

/* 初始化协议层：
 *   dev_id : 本机设备编号，用于多设备挂同一RS-485总线时区分节点
 *   SEQ    : 上电后从0开始
 */
void RadarProtocol_Init(uint16_t dev_id);

/* 查询当前待发送SEQ，主要用于串口调试或断点观察。 */
uint16_t RadarProtocol_GetSeq(void);

/* 发送二进制测量数据帧，不等待ACK。
 *
 * extra_status用于补充其他模块状态位，例如：
 *   RADAR_STATUS_PLL_LOCK_OK | RADAR_STATUS_ADC_OK | RADAR_STATUS_SELF_TEST_OK
 *
 * 函数内部会自动：
 *   RadarFrame_t -> status合成 -> 定点化payload -> 组二进制帧 -> CRC16 -> RS485发送
 */
RadarProto_Status_t RadarProtocol_SendData(const RadarFrame_t *frame, uint16_t extra_status);

/* 发送二进制测量数据帧，并等待上位机ACK。
 *
 * 收到匹配ACK后SEQ递增；
 * 超时则使用同一个SEQ重发，最多重发RADAR_PROTO_MAX_RETRY次。
 */
RadarProto_Status_t RadarProtocol_SendDataWaitAck(const RadarFrame_t *frame, uint16_t extra_status);

/* 发送ASCII调试帧：
 *   $RADAR,WL:1.234,WV:0.123*XX\r\n
 *
 * 只用于串口助手观察，不参与SEQ和ACK机制。
 */
RadarProto_Status_t RadarProtocol_SendDebug(const RadarFrame_t *frame);

#endif /* RADAR_PROTOCOL_H */
