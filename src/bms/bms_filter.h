/*
 * bms_filter.h —— 采样滤波（滑动平均 + 一阶低通）
 *
 * 对应 docs/design.md 第 2 节。两级串联：
 *
 *   x -> [滑动平均 8 点] -> [一阶低通 α=0.2] -> y
 *
 * 为什么两级都要：
 *   滑动平均压随机噪声（白噪声），但对阶跃响应慢；
 *   一阶低通压高频毛刺，对缓慢漂移无能为力。
 *   只用一种，要么残留毛刺，要么响应太迟钝。
 *
 * 注意保护判断用的是「原始值 + 滤波值双路」，滤波后的值只用于上报和显示，
 * 单靠滤波值判断会漏掉真正快速上升的故障（见 bms_protect.c）。
 */
#ifndef BMS_FILTER_H
#define BMS_FILTER_H

#include <stdint.h>

#define BMS_MA_WINDOW   8
#define BMS_LP_ALPHA    200   /* α = 0.2，按千分比表示 */

typedef struct
{
    int32_t buf[BMS_MA_WINDOW];
    int32_t sum;
    uint8_t idx;
    uint8_t count;
} bms_movavg_t;

void bms_movavg_init(bms_movavg_t *f);

/* 推入一个新值，返回窗口内均值。窗口未满时按已有样本数平均 */
int32_t bms_movavg_push(bms_movavg_t *f, int32_t x);

typedef struct
{
    int32_t y;
    int32_t alpha_milli;
    uint8_t inited;
} bms_lowpass_t;

void bms_lowpass_init(bms_lowpass_t *f, int32_t alpha_milli);

/* y = α·x + (1−α)·y，返回滤波后结果 */
int32_t bms_lowpass_update(bms_lowpass_t *f, int32_t x);

/* 一路通道 = 两级串联 */
typedef struct
{
    bms_movavg_t ma;
    bms_lowpass_t lp;
} bms_chan_t;

void bms_chan_init(bms_chan_t *c);

/* 推入原始值，返回滤波值 */
int32_t bms_chan_push(bms_chan_t *c, int32_t x);

#endif
