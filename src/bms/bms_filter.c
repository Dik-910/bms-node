/*
 * bms_filter.c —— 滤波实现
 */
#include "bms_filter.h"

#include <string.h>

void bms_movavg_init(bms_movavg_t *f)
{
    if (f == 0)
        return;

    memset(f, 0, sizeof(*f));
}

int32_t bms_movavg_push(bms_movavg_t *f, int32_t x)
{
    if (f == 0)
        return x;

    if (f->count < BMS_MA_WINDOW)
    {
        /* 窗口未满：先填满再谈"平均"，否则前几个值会被 0 稀释 */
        f->buf[f->idx] = x;
        f->sum += x;
        f->count++;
        f->idx = (uint8_t)((f->idx + 1) % BMS_MA_WINDOW);
        return f->sum / f->count;
    }

    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = x;
    f->sum += x;
    f->idx = (uint8_t)((f->idx + 1) % BMS_MA_WINDOW);

    return f->sum / BMS_MA_WINDOW;
}

void bms_lowpass_init(bms_lowpass_t *f, int32_t alpha_milli)
{
    if (f == 0)
        return;

    memset(f, 0, sizeof(*f));

    if (alpha_milli <= 0 || alpha_milli > 1000)
        alpha_milli = BMS_LP_ALPHA;

    f->alpha_milli = alpha_milli;
}

int32_t bms_lowpass_update(bms_lowpass_t *f, int32_t x)
{
    if (f == 0)
        return x;

    if (!f->inited)
    {
        /*
         * 首值直接采用，不从 0 开始收敛。
         * 否则开机后一段时间的滤波值是「从零爬上来」的假数据，
         * 这段时间的保护判断全都不准。
         */
        f->y = x;
        f->inited = 1;
        return f->y;
    }

    /*
     * 定点运算：y = (α·x + (1000−α)·y) / 1000
     *
     * 必须加 500 做四舍五入。直接截断的话会有**稳态误差**：
     * α=200、x=20000 时，y 会卡在 19996 再也上不去 ——
     * 因为 (0.2·20000 + 0.8·19996) 截断后还是 19996，形成不动点。
     * 这个误差在保护判断里会被当成真实偏差，所以不能留着。
     */
    {
        int32_t num = f->alpha_milli * x + (1000 - f->alpha_milli) * f->y;

        f->y = (num + 500) / 1000;
    }
    return f->y;
}

void bms_chan_init(bms_chan_t *c)
{
    if (c == 0)
        return;

    bms_movavg_init(&c->ma);
    bms_lowpass_init(&c->lp, BMS_LP_ALPHA);
}

int32_t bms_chan_push(bms_chan_t *c, int32_t x)
{
    if (c == 0)
        return x;

    return bms_lowpass_update(&c->lp, bms_movavg_push(&c->ma, x));
}
