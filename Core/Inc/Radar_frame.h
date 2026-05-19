#ifndef RADAR_FRAME_H
#define RADAR_FRAME_H

#include "Radar_config.h"
#include <stdbool.h>

/* ============================================================
 *  一帧完整的测量结果
 *
 *  一帧流程：
 *    1. 水位：1次上扫chirp → Range-FFT → 距离
 *    2. 水速：N_CHIRPS_WATER_V对三角波（上扫提取相位）→ 相位差分 → 速度
 *    3. 车速：N_CHIRPS_CAR对三角波 → 联立方程 → 距离+速度
 *  总时间约：1ms + 32ms + 8ms = 41ms（每帧约25Hz刷新率）
 * ============================================================ */
typedef struct {
    /* 水位 */
    float water_level_m;        /* 雷达到水面的距离（米）
                                 * 实际水位 = 雷达安装高度 - water_level_m */
    bool  water_level_valid;    /* 水位测量是否有效 */
    float water_level_snr;      /* 水位测量信噪比（dB） */

    /* 水速 */
    float water_velocity_mps;   /* 水面流速（m/s），正值=水流靠近雷达方向 */
    bool  water_velocity_valid; /* 水速测量是否有效 */

    /* 车辆（测试用，实测河道时可不使用） */
    float car_range_m;          /* 车辆到雷达的距离（米） */
    float car_velocity_mps;     /* 车辆速度（m/s），正值=车辆靠近雷达 */
    bool  car_valid;            /* 车辆测量是否有效 */
} RadarFrame_t;

/* 执行一帧完整测量，填充结果结构体 */
void Radar_MeasureFrame(RadarFrame_t *frame);

#endif /* RADAR_FRAME_H */