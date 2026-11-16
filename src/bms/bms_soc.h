/*
 * bms_soc.h —— SOC 估算（安时积分 + OCV 静置修正）
 *
 * 对应 docs/design.md 第 3 节，但**符号约定与文档原文不一致，以代码为准**：
 *
 *   design.md 写的是 SOC = SOC₀ − (1/Q)∫I dt，同时又说 I 充电为正。
 *   这两条互相矛盾：充电时 I > 0，减号会让 SOC 下降。
 *   本实现统一为「充电为正，SOC 用加号」，与 can-node 的 DBC 一致。
 *
 * 实现要点：
 *
 * 1. 电荷累积不除以 1000，单位保持 mA·ms。
 *    先除会截断：50mA × 10ms = 0.5 → 0，长期下来 SOC 会莫名其妙地不动。
 *
 * 2. 从「基准点」算，不逐次累加：
 *      SOC = SOC₀ + charge_since_anchor · 1000 / (3600000 · Q_mAh)
 *    逐次累加每次都有一次整数除法截断，误差会单向累积；
 *    从基准点算只有最后一次截断。OCV 修正时把基准点挪过去、电荷清零。
 *
 * 3. OCV 修正的三个条件（docs/design.md）：
 *    静置够久 + 距上次修正够久 + 温度在表有效范围内。
 *    三条缺一不可 —— 少任何一条，都会在某些工况下用不可信的 OCV 覆盖积分值。
 */
#ifndef BMS_SOC_H
#define BMS_SOC_H

#include <stdint.h>

#define BMS_SOC_MAX_PERMILLE  1000   /* 100.0% */

typedef struct
{
    int32_t  rest_current_ma;   /* 判定静置的电流阈值（绝对值） */
    uint32_t rest_time_ms;      /* 需要持续静置多久才允许修正 */
    uint32_t min_interval_ms;   /* 两次修正的最小间隔 */
    int16_t  temp_min_dc;       /* OCV 表有效温度下限，0.1℃ */
    int16_t  temp_max_dc;       /* 上限 */
} bms_soc_cfg_t;

typedef struct
{
    bms_soc_cfg_t cfg;

    uint32_t q_mah;             /* 额定容量 */
    uint8_t  cells;             /* 串联节数，用于把包压换算成单节电压 */

    int32_t  soc0_permille;     /* 基准点 SOC */
    int64_t  charge_ma_ms;      /* 自基准点以来的电荷，mA·ms，充电为正 */

    uint32_t rest_ms;           /* 已连续静置时间 */
    uint32_t since_fix_ms;      /* 距上次修正的时间 */
    uint32_t corrections;       /* 修正次数，用来判断"这台设备有没有机会修正" */
} bms_soc_t;

/* 默认配置：0.1A 静置阈值、5 分钟静置、10 分钟最小间隔、0~45℃ */
void bms_soc_default_cfg(bms_soc_cfg_t *cfg);

void bms_soc_init(bms_soc_t *s, const bms_soc_cfg_t *cfg,
                  uint32_t q_mah, uint8_t cells, int32_t soc0_permille);

/*
 * 推进一步。dt_ms 由调用方给出（采样周期累加），不在这里读时钟 ——
 * 这样测试可以精确推进时间，不用真等 5 分钟。
 */
void bms_soc_step(bms_soc_t *s, int32_t current_ma, uint32_t pack_mv,
                  int16_t temp_dc, uint32_t dt_ms);

int32_t bms_soc_get(const bms_soc_t *s);

/*
 * 单节 OCV 查表，返回 SOC permille。
 * 返回 -1 表示电压超出表的范围（此时不能用它修正）。
 */
int32_t bms_ocv_lookup(uint32_t cell_mv);

#endif
