/*
 * bms_soc.c —— SOC 估算实现
 */
#include "bms_soc.h"

#include <string.h>

/*
 * LiFePO4 单节 OCV-SOC 表（25℃）。
 *
 * 选磷酸铁锂而不是三元：储能场景主流是 LFP。
 *
 * 注意这张表的**平台期问题**：3.25V~3.32V 覆盖了 30%~70% 的容量，
 * 电压变化只有 70mV —— 采样精度 ±5mV 就意味着 ±4% 的 SOC 误差。
 * 所以 OCV 修正只在静置足够久时才可信，而且低温下这张表更平，
 * 修正价值更低（见 docs/design.md 的「局限与边界」）。
 */
typedef struct
{
    uint16_t mv;
    uint16_t permille;
} ocv_point_t;

static const ocv_point_t g_ocv[] =
{
    {3000,    0},
    {3200,  100},
    {3250,  300},
    {3300,  500},
    {3320,  700},
    {3350,  900},
    {3400, 1000},
};

#define OCV_POINTS (sizeof(g_ocv) / sizeof(g_ocv[0]))

int32_t bms_ocv_lookup(uint32_t cell_mv)
{
    /*
     * 超出表范围不能钳位，必须返回 -1（不可信）。
     *
     * 这是演示中发现的真问题：过压保护后回落到 3423mV，
     * 高于表的顶端 3400mV。如果钳位成 100%，OCV 修正会把 SOC 直接拉到满值 ——
     * 而物理上此时电池只是刚脱离充电，端电压还带着表面电荷，根本不是静置 OCV。
     *
     * 留一点容差（下限 200mV、上限 20mV，约 4 个采样 LSB）给误差，
     * 但明显超出就是「这个电压不能用来估 SOC」。
     */
    if (cell_mv <= g_ocv[0].mv)
        return (cell_mv + 200u < g_ocv[0].mv) ? -1 : g_ocv[0].permille;

    if (cell_mv >= g_ocv[OCV_POINTS - 1].mv)
        return (cell_mv > g_ocv[OCV_POINTS - 1].mv + 20u) ? -1 : g_ocv[OCV_POINTS - 1].permille;

    for (size_t i = 1; i < OCV_POINTS; i++)
    {
        if (cell_mv > g_ocv[i].mv)
            continue;

        {
            uint32_t lo_mv = g_ocv[i - 1].mv;
            uint32_t hi_mv = g_ocv[i].mv;
            int32_t  lo_soc = g_ocv[i - 1].permille;
            int32_t  hi_soc = g_ocv[i].permille;

            /* 线性插值：soc = lo + (mv − lo_mv)·(hi_soc − lo_soc)/(hi_mv − lo_mv) */
            return lo_soc + (int32_t)((cell_mv - lo_mv) * (uint32_t)(hi_soc - lo_soc) /
                                      (hi_mv - lo_mv));
        }
    }

    return g_ocv[OCV_POINTS - 1].permille;
}

void bms_soc_default_cfg(bms_soc_cfg_t *cfg)
{
    if (cfg == 0)
        return;

    cfg->rest_current_ma = 100;          /* 0.1 A */
    cfg->rest_time_ms    = 300000u;      /* 5 分钟 */
    cfg->min_interval_ms = 600000u;      /* 10 分钟 */
    cfg->temp_min_dc     = 0;            /* 0℃ */
    cfg->temp_max_dc     = 450;          /* 45℃ */
}

void bms_soc_init(bms_soc_t *s, const bms_soc_cfg_t *cfg,
                  uint32_t q_mah, uint8_t cells, int32_t soc0_permille)
{
    if (s == 0)
        return;

    memset(s, 0, sizeof(*s));

    if (cfg != 0)
        s->cfg = *cfg;
    else
        bms_soc_default_cfg(&s->cfg);

    s->q_mah = (q_mah != 0) ? q_mah : 100000u;
    s->cells = (cells != 0) ? cells : 1;

    if (soc0_permille < 0)
        soc0_permille = 0;
    if (soc0_permille > BMS_SOC_MAX_PERMILLE)
        soc0_permille = BMS_SOC_MAX_PERMILLE;

    s->soc0_permille = soc0_permille;

    /*
     * 开机时把「距上次修正」设为已满足最小间隔，
     * 这样只要静置够久就能立刻修正一次 —— 上电时电池通常正处在静置状态，
     * 这是拿到可信基准的最好时机。错过它，要再等到下次停机。
     */
    s->since_fix_ms = s->cfg.min_interval_ms;
}

static int32_t soc_compute(const bms_soc_t *s)
{
    /*
     * soc = soc0 + charge_ma_ms · 1000 / (3600000 · q_mah)
     *
     * 3600000 = 每小时毫秒数，用来把 mA·ms 换成 mAh；
     * 乘 1000 是换成千分比。分子用 int64：满载 100Ah 时约 3.6e12，
     * 32 位放不下。
     */
    int64_t denom = 3600000LL * (int64_t)s->q_mah;
    int64_t delta = (s->charge_ma_ms * 1000LL) / denom;
    int64_t soc = (int64_t)s->soc0_permille + delta;

    if (soc < 0)
        soc = 0;
    if (soc > BMS_SOC_MAX_PERMILLE)
        soc = BMS_SOC_MAX_PERMILLE;

    return (int32_t)soc;
}

void bms_soc_step(bms_soc_t *s, int32_t current_ma, uint32_t pack_mv,
                  int16_t temp_dc, uint32_t dt_ms)
{
    int32_t abs_i;

    if (s == 0 || dt_ms == 0)
        return;

    /* 1. 安时积分：充电为正，SOC 增加 */
    s->charge_ma_ms += (int64_t)current_ma * (int64_t)dt_ms;

    /* 2. 静置计时 */
    abs_i = (current_ma < 0) ? -current_ma : current_ma;
    if (abs_i <= s->cfg.rest_current_ma)
    {
        if (s->rest_ms < 0xFFFFFFFFu - dt_ms)
            s->rest_ms += dt_ms;
    }
    else
    {
        s->rest_ms = 0;
    }

    if (s->since_fix_ms < 0xFFFFFFFFu - dt_ms)
        s->since_fix_ms += dt_ms;

    /* 3. OCV 修正：三个条件缺一不可 */
    if (s->rest_ms >= s->cfg.rest_time_ms &&
        s->since_fix_ms >= s->cfg.min_interval_ms &&
        temp_dc >= s->cfg.temp_min_dc &&
        temp_dc <= s->cfg.temp_max_dc)
    {
        uint32_t cell_mv = pack_mv / s->cells;
        int32_t  ocv_soc = bms_ocv_lookup(cell_mv);

        if (ocv_soc >= 0)
        {
            s->soc0_permille = ocv_soc;
            s->charge_ma_ms = 0;          /* 把累积误差清零 */
            s->since_fix_ms = 0;
            s->rest_ms = 0;
            s->corrections++;
        }
    }
}

int32_t bms_soc_get(const bms_soc_t *s)
{
    if (s == 0)
        return 0;

    return soc_compute(s);
}
