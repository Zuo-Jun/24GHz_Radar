#include "Radar_protocol.h"
#include "SP3485E.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  协议固定参数
 *
 *  二进制应用帧：
 *    SOF | TYPE | DEV_ID | SEQ | LEN | PAYLOAD | CRC16
 *
 *  当前实现的测量数据PAYLOAD固定为10字节：
 *    status         uint16_t  2字节
 *    water_level    int32_t   4字节，单位mm
 *    water_velocity int32_t   4字节，单位mm/s
 *
 *  所有多字节字段均采用小端格式：低字节在前。
 * ============================================================ */
#define RADAR_PROTO_SOF               0x55AAU
#define RADAR_PROTO_TYPE_RADAR_DATA   0x01U
#define RADAR_PROTO_TYPE_ACK          0x81U
#define RADAR_PROTO_HEADER_SIZE       8U /* SOF(2) + TYPE(1) + DEV_ID(2) + SEQ(2) + LEN(1) */
#define RADAR_PROTO_CRC_SIZE          2U
#define RADAR_PROTO_MIN_FRAME_SIZE    (RADAR_PROTO_HEADER_SIZE + RADAR_PROTO_CRC_SIZE)
#define RADAR_PROTO_DATA_PAYLOAD_SIZE 10U /* status(2) + water_level(4) + water_velocity(4) */
#define RADAR_PROTO_DATA_FRAME_SIZE   (RADAR_PROTO_MIN_FRAME_SIZE + RADAR_PROTO_DATA_PAYLOAD_SIZE)

static uint16_t s_dev_id = 1U; /* 设备ID，单设备时固定为1 */
static uint16_t s_seq    = 0U; /* 待发送数据帧的SEQ，成功发送后递增 */

/* ============================================================
 *  小端读写辅助函数
 *
 *  不把结构体直接强转成uint8_t数组发送，原因是：
 *    1. 结构体可能有编译器自动填充字节；
 *    2. 结构体对齐方式不应该成为通信协议的一部分；
 *    3. 逐字段写入可以保证上位机按文档稳定解析。
 * ============================================================ */
static void put_u16_le(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void put_i32_le(uint8_t *buf, int32_t value)
{
    uint32_t v = (uint32_t)value;
    buf[0] = (uint8_t)(v & 0xFFU);
    buf[1] = (uint8_t)((v >> 8) & 0xFFU);
    buf[2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint16_t get_u16_le(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

/* ============================================================
 *  物理量定点化
 *
 *  协议不直接传float，而是传定点整数：
 *    water_level_m      -> mm
 *    water_velocity_mps -> mm/s
 *
 *  这样做的原因：
 *    1. 上位机、数据库、脚本语言解析更简单；
 *    2. 不需要处理float大小端和IEEE754表示细节；
 *    3. 当前测量结果用1mm、1mm/s分辨率已经足够。
 * ============================================================ */
static int32_t float_to_i32_1000(float value)
{
    float scaled = value * 1000.0f;
    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }
    return (int32_t)(scaled - 0.5f);
}

/* ============================================================
 *  status合成
 *
 *  本函数只处理协议层能直接知道的信息：
 *    water_level_valid    -> WATER_LEVEL_VALID
 *    water_velocity_valid -> WATER_VELOCITY_VALID
 *
 *  extra_status由调用者传入，用于补充其他模块状态：
 *    PLL_LOCK_OK、ADC_OK、SELF_TEST_OK等。
 *
 *  如果水位无效，当前先置SIGNAL_WEAK作为基础原因。
 *  后续如果算法层能区分弱信号、超量程、异常速度，再细分对应bit。
 * ============================================================ */
static uint16_t make_status(const RadarFrame_t *frame, uint16_t extra_status)
{
    uint16_t status = extra_status;

    if (frame == NULL) {
        return status;
    }

    if (frame->water_level_valid) {
        status |= RADAR_STATUS_WATER_LEVEL_VALID;
    } else {
        status |= RADAR_STATUS_SIGNAL_WEAK;
    }

    if (frame->water_velocity_valid) {
        status |= RADAR_STATUS_WATER_VELOCITY_VALID;
    }

    return status;
}

/* ============================================================
 *  CRC16(Modbus)
 *
 *  参数：
 *    初值    : 0xFFFF
 *    多项式  : 0xA001
 *    发送顺序: 低字节在前
 *
 *  本协议CRC计算范围：
 *    TYPE | DEV_ID | SEQ | LEN | PAYLOAD
 *
 *  不包含：
 *    SOF 和 CRC16本身。
 * ============================================================ */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (crc >> 1) ^ 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* ============================================================
 *  通用二进制帧封装
 *
 *  只负责把传入字段拼成字节流：
 *    SOF -> TYPE -> DEV_ID -> SEQ -> LEN -> PAYLOAD -> CRC16
 *
 *  注意：
 *    这个函数不发送，也不递增SEQ。
 *    可靠发送需要在超时时重发同一帧，因此SEQ必须由外层发送逻辑控制。
 * ============================================================ */
static uint16_t build_frame(uint8_t type, uint16_t dev_id, uint16_t seq,
                            const uint8_t *payload, uint8_t payload_len,
                            uint8_t *out_buf, uint16_t out_size)
{
    uint16_t frame_len = (uint16_t)RADAR_PROTO_MIN_FRAME_SIZE + payload_len;
    uint16_t crc;

    if (out_buf == NULL || out_size < frame_len || payload == NULL) {
        return 0U;
    }

    put_u16_le(&out_buf[0], RADAR_PROTO_SOF);
    out_buf[2] = type;
    put_u16_le(&out_buf[3], dev_id);
    put_u16_le(&out_buf[5], seq);
    out_buf[7] = payload_len;
    memcpy(&out_buf[8], payload, payload_len);

    crc = crc16_modbus(&out_buf[2], (uint16_t)(1U + 2U + 2U + 1U + payload_len));
    put_u16_le(&out_buf[8U + payload_len], crc);

    return frame_len;
}

/* ============================================================
 *  测量数据帧封装
 *
 *  从RadarFrame_t生成TYPE=0x01的正式二进制应用帧。
 *
 *  PAYLOAD固定布局：
 *    byte0~1 : status
 *    byte2~5 : water_level_mm
 *    byte6~9 : water_velocity_mms
 *
 *  即使某个物理量无效，数值字段仍然会填入；
 *  上位机必须优先根据status中的VALID位判断该数值是否可信。
 * ============================================================ */
static uint16_t build_data_frame(const RadarFrame_t *frame, uint16_t extra_status,
                                 uint16_t seq, uint8_t *out_buf, uint16_t out_size)
{
    uint8_t payload[RADAR_PROTO_DATA_PAYLOAD_SIZE];
    uint16_t status;
    int32_t water_level_mm = 0;
    int32_t water_velocity_mms = 0;

    if (frame == NULL) {
        return 0U;
    }

    status = make_status(frame, extra_status);
    water_level_mm = float_to_i32_1000(frame->water_level_m);
    water_velocity_mms = float_to_i32_1000(frame->water_velocity_mps);

    put_u16_le(&payload[0], status);
    put_i32_le(&payload[2], water_level_mm);
    put_i32_le(&payload[6], water_velocity_mms);

    return build_frame(RADAR_PROTO_TYPE_RADAR_DATA, s_dev_id, seq,
                       payload, RADAR_PROTO_DATA_PAYLOAD_SIZE,
                       out_buf, out_size);
}

/* ASCII调试帧XOR校验：
 *   校验范围为$和*之间的正文，不包含$和*。
 */
static uint8_t calc_xor(const char *str, uint16_t len)
{
    uint8_t xor_val = 0U;

    for (uint16_t i = 0; i < len; i++) {
        xor_val ^= (uint8_t)str[i];
    }

    return xor_val;
}

/* ============================================================
 *  ASCII调试帧封装
 *
 *  格式：
 *    $RADAR,WL:1.234,WV:0.123*XX\r\n
 *
 *  用途：
 *    开发阶段用普通串口助手直接观察水位、水速。
 *
 *  注意：
 *    ASCII帧不作为最终数据库入库协议；
 *    正式应用使用二进制帧，因为字段固定、CRC可靠、解析成本低。
 * ============================================================ */
static uint16_t build_debug_frame(const RadarFrame_t *frame, char *out_buf, uint16_t out_size)
{
    char body[80];
    int body_len;
    int frame_len;
    uint8_t xor_val;

    if (frame == NULL || out_buf == NULL || out_size == 0U) {
        return 0U;
    }

    if (frame->water_level_valid) {
        body_len = snprintf(body, sizeof(body), "RADAR,WL:%.3f", frame->water_level_m);
    } else {
        body_len = snprintf(body, sizeof(body), "RADAR,WL:----");
    }
    if (body_len < 0 || body_len >= (int)sizeof(body)) {
        return 0U;
    }

    if (frame->water_velocity_valid) {
        body_len += snprintf(body + body_len, sizeof(body) - (uint16_t)body_len,
                             ",WV:%.3f", frame->water_velocity_mps);
    } else {
        body_len += snprintf(body + body_len, sizeof(body) - (uint16_t)body_len,
                             ",WV:----");
    }
    if (body_len < 0 || body_len >= (int)sizeof(body)) {
        return 0U;
    }

    xor_val = calc_xor(body, (uint16_t)body_len);
    frame_len = snprintf(out_buf, out_size, "$%s*%02X\r\n", body, xor_val);
    if (frame_len < 0 || frame_len >= (int)out_size) {
        return 0U;
    }

    return (uint16_t)frame_len;
}

/* RS485驱动层状态转换为协议层状态。
 * 对外API不暴露RS485_Status_t，避免上层同时依赖两个模块的状态枚举。
 */
static RadarProto_Status_t rs485_to_proto_status(RS485_Status_t status)
{
    if (status == RS485_OK) {
        return RADAR_PROTO_OK;
    }
    if (status == RS485_TIMEOUT) {
        return RADAR_PROTO_TIMEOUT;
    }
    return RADAR_PROTO_RS485_ERROR;
}

/* ============================================================
 *  ACK帧判断
 *
 *  ACK格式固定：
 *    SOF | TYPE | DEV_ID | SEQ | LEN | CRC16
 *
 *  其中：
 *    TYPE = 0x81
 *    LEN  = 0
 *
 *  判断ACK是否有效需要同时满足：
 *    1. SOF正确；
 *    2. CRC正确；
 *    3. TYPE为ACK；
 *    4. DEV_ID等于本机设备号；
 *    5. SEQ等于当前等待确认的数据帧SEQ。
 * ============================================================ */
static bool is_ack_frame(const uint8_t *buf, uint16_t len, uint16_t dev_id, uint16_t seq)
{
    uint16_t crc_rx;
    uint16_t crc_calc;

    if (buf == NULL || len < RADAR_PROTO_MIN_FRAME_SIZE) {
        return false;
    }
    if (get_u16_le(&buf[0]) != RADAR_PROTO_SOF) {
        return false;
    }
    if (buf[2] != RADAR_PROTO_TYPE_ACK) {
        return false;
    }
    if (get_u16_le(&buf[3]) != dev_id) {
        return false;
    }
    if (get_u16_le(&buf[5]) != seq) {
        return false;
    }
    if (buf[7] != 0U) {
        return false;
    }

    crc_rx = get_u16_le(&buf[8]);
    crc_calc = crc16_modbus(&buf[2], 6U);

    return crc_rx == crc_calc;
}

/* ============================================================
 *  等待ACK
 *
 *  从SP3485E.c维护的RS-485接收环形缓冲中逐字节读取，
 *  寻找长度固定为10字节的ACK帧。
 *
 *  当前只处理ACK，因为下位机发送测量帧后只关心上位机确认。
 *  如果后续支持SET_PARAM/READ_PARAM这类带payload的下行帧，
 *  可以把这里扩展成通用“从字节流中找SOF并按LEN收完整帧”的解析器。
 * ============================================================ */
static bool wait_ack(uint16_t dev_id, uint16_t seq, uint32_t timeout_ms)
{
    uint8_t frame[RADAR_PROTO_MIN_FRAME_SIZE];
    uint8_t idx = 0U;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms) {
        int16_t rx = RS485_ReadByte();
        if (rx < 0) {
            continue;
        }

        uint8_t byte = (uint8_t)rx;

        if (idx == 0U) {
            if (byte != (uint8_t)(RADAR_PROTO_SOF & 0xFFU)) {
                continue;
            }
            frame[idx++] = byte;
        } else if (idx == 1U) {
            if (byte != (uint8_t)(RADAR_PROTO_SOF >> 8)) {
                idx = 0U;
                continue;
            }
            frame[idx++] = byte;
        } else {
            frame[idx++] = byte;
            if (idx >= RADAR_PROTO_MIN_FRAME_SIZE) {
                if (is_ack_frame(frame, RADAR_PROTO_MIN_FRAME_SIZE, dev_id, seq)) {
                    return true;
                }
                idx = 0U;
            }
        }
    }

    return false;
}

void RadarProtocol_Init(uint16_t dev_id)
{
    s_dev_id = dev_id;
    s_seq = 0U;
}

uint16_t RadarProtocol_GetSeq(void)
{
    return s_seq;
}

/* ============================================================
 *  RadarProtocol_SendData
 *
 *  单向发送测量数据帧，不等待ACK。
 *
 *  推荐用于第一阶段联调：
 *    下位机周期性发送固定数据或真实测量数据；
 *    上位机只接收、打印十六进制、校验CRC、解析字段。
 *
 *  SEQ规则：
 *    RS485_Send成功后SEQ递增；
 *    如果底层发送失败，SEQ不变，避免上位机看到无意义跳号。
 * ============================================================ */
RadarProto_Status_t RadarProtocol_SendData(const RadarFrame_t *frame, uint16_t extra_status)
{
    uint8_t tx_buf[RADAR_PROTO_DATA_FRAME_SIZE]; // 8字节头 + 10字节payload + 2字节CRC
    uint16_t len = build_data_frame(frame, extra_status, s_seq, tx_buf, sizeof(tx_buf));
    RadarProto_Status_t ret;

    if (len == 0U) {
        return RADAR_PROTO_ERROR;
    }

    ret = rs485_to_proto_status(RS485_Send(tx_buf, len));
    if (ret == RADAR_PROTO_OK) {
        s_seq++;
    }

    return ret;
}

/* ============================================================
 *  RadarProtocol_SendDataWaitAck
 *
 *  可靠发送流程：
 *    1. 使用当前SEQ构造测量数据帧；
 *    2. 发送；
 *    3. 等待上位机返回TYPE=0x81且DEV_ID/SEQ匹配的ACK；
 *    4. 超时则重发同一帧；
 *    5. 收到ACK后SEQ递增。
 *
 *  关键点：
 *    重发时必须保持同一个SEQ。
 *    因此这里先缓存seq=s_seq，再进入重发循环。
 * ============================================================ */
RadarProto_Status_t RadarProtocol_SendDataWaitAck(const RadarFrame_t *frame, uint16_t extra_status)
{
    uint8_t tx_buf[RADAR_PROTO_DATA_FRAME_SIZE];
    uint16_t seq = s_seq;
    uint16_t len = build_data_frame(frame, extra_status, seq, tx_buf, sizeof(tx_buf));

    if (len == 0U) {
        return RADAR_PROTO_ERROR;
    }

    for (uint8_t retry = 0U; retry <= RADAR_PROTO_MAX_RETRY; retry++) {
        // RS485_FlushRx();
        RadarProto_Status_t ret = rs485_to_proto_status(RS485_Send(tx_buf, len));
        if (ret != RADAR_PROTO_OK) {
            return ret;
        }

        if (wait_ack(s_dev_id, seq, RADAR_PROTO_ACK_TIMEOUT_MS)) {
            s_seq++;
            return RADAR_PROTO_OK;
        }
    }

    return RADAR_PROTO_TIMEOUT;
}

/* ============================================================
 *  RadarProtocol_SendDebug
 *
 *  发送ASCII调试帧，不影响SEQ，也不等待ACK。
 *
 *  如果同时调试二进制帧和ASCII帧，建议分阶段发送，
 *  否则上位机解析二进制字节流时会混入ASCII内容。
 * ============================================================ */
RadarProto_Status_t RadarProtocol_SendDebug(const RadarFrame_t *frame)
{
    char tx_buf[96];
    uint16_t len = build_debug_frame(frame, tx_buf, sizeof(tx_buf));

    if (len == 0U) {
        return RADAR_PROTO_ERROR;
    }

    return rs485_to_proto_status(RS485_Send((const uint8_t *)tx_buf, len));
}
