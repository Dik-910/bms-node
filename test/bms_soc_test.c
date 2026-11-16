/*
 * bms_soc_test.c —— SOC 估算
 *
 * 三个层次：
 *   1. 安时积分的数值正确性（用能心算出结果的条件核对）
 *   2. OCV 修正的三个触发条件必须**同时**满足
 *   3. 边界：钳位、查表范围、无静置机会时误差如何表现
 */
#include <stdio.h>
#include <string.h>
#include "bms_soc.h"

static int g_fail;
static int g_total;

static void check(int cond, const char *what, long got, long want)
{
    g_total++;
    if (!cond)
    {
        g_fail++;
        printf("  [FAIL] %s (got %ld, want %ld)\n", what, got, want);
    }
}

#define PACK_MV_3300  (13 * 3300)   /* 单节 3.30V，查表对应 SOC 500 */

static void test_ocv_table(void)
{
    printf("\nOCV table\n");

    check(bms_ocv_lookup(3000) == 0, "3.00V -> 0", bms_ocv_lookup(3000), 0);
    check(bms_ocv_lookup(3200) == 100, "3.20V -> 100", bms_ocv_lookup(3200), 100);
    check(bms_ocv_lookup(3300) == 500, "3.30V -> 500", bms_ocv_lookup(3300), 500);
    check(bms_ocv_lookup(3400) == 1000, "3.40V -> 1000", bms_ocv_lookup(3400), 1000);

    /* 插值：3225 在 3200(100) 与 3250(300) 之间，正好一半 -> 200 */
    check(bms_ocv_lookup(3225) == 200, "interpolate 3.225V -> 200",
          bms_ocv_lookup(3225), 200);

    /*
     * 超出表范围一律返回 -1（不可信），不钳位。
     * 钳位会把「刚充完电、端电压虚高」误判成满电，
     * 这个错误在 OCV 修正里会直接把 SOC 拉到 100%。
     */
    check(bms_ocv_lookup(1400) == -1, "far below table rejected",
          bms_ocv_lookup(1400), -1);
    check(bms_ocv_lookup(3500) == -1, "above table rejected (not clamped)",
          bms_ocv_lookup(3500), -1);
    check(bms_ocv_lookup(3420) == 1000, "within tolerance still works",
          bms_ocv_lookup(3420), 1000);
}

static void test_coulomb_counting(void)
{
    bms_soc_t s;
    bms_soc_cfg_t cfg;

    printf("\nCoulomb counting\n");
    bms_soc_default_cfg(&cfg);
    bms_soc_init(&s, &cfg, 100000, 13, 500);   /* 100Ah, 起始 50% */

    check(bms_soc_get(&s) == 500, "initial soc", bms_soc_get(&s), 500);

    /*
     * 放电 10A 持续 1 小时：100Ah 的 10% -> SOC 应降到 400。
     * 用 10ms 步长推进 360000 次太慢，用 1000ms 步长 3600 次。
     * 电流为负表示放电。
     */
    for (int i = 0; i < 3600; i++)
        bms_soc_step(&s, -10000, PACK_MV_3300, 250, 1000);

    check(bms_soc_get(&s) == 400, "10A discharge for 1h -> 400",
          bms_soc_get(&s), 400);

    /* 反向充电同样电量，应回到 500 */
    for (int i = 0; i < 3600; i++)
        bms_soc_step(&s, 10000, PACK_MV_3300, 250, 1000);

    check(bms_soc_get(&s) == 500, "10A charge for 1h -> back to 500",
          bms_soc_get(&s), 500);

    /*
     * 小电流 + 短步长：测的是「累积后再除」而不是「每步先除」。
     *
     * 每步电荷 = -50mA × 10ms = -0.5 mA·ms。若实现写成
     *     charge += I * dt / 1000;
     * 每步都截断成 0，跑多久 SOC 都不动。
     * 正确做法是保持 mA·ms 单位累积，最后才换算（本实现）。
     *
     * 4 小时 × 50mA = 200mAh = 100Ah 的 0.2% = 2 permille -> 498。
     *
     * 另外必须把静置阈值设为 0：默认 100mA 会让 -50mA 被判成静置，
     * 5 分钟后触发 OCV 修正，把 SOC 直接重置成查表值 —— 那样测的就不是积分了。
     */
    cfg.rest_current_ma = 0;
    bms_soc_init(&s, &cfg, 100000, 13, 500);
    for (int i = 0; i < 4 * 3600 * 100; i++)
        bms_soc_step(&s, -50, PACK_MV_3300, 250, 10);

    check(bms_soc_get(&s) == 498, "small current: accumulate then divide",
          bms_soc_get(&s), 498);
    check(s.corrections == 0, "no correction during this test", s.corrections, 0);
}

static void test_clamp(void)
{
    bms_soc_t s;
    bms_soc_cfg_t cfg;

    printf("\nClamping\n");
    bms_soc_default_cfg(&cfg);
    bms_soc_init(&s, &cfg, 100000, 13, 500);

    /* 放电远超容量：钳到 0，不能变负数 */
    for (int i = 0; i < 7200; i++)
        bms_soc_step(&s, -200000, PACK_MV_3300, 250, 1000);

    check(bms_soc_get(&s) == 0, "over-discharge clamps to 0", bms_soc_get(&s), 0);

    /* 反向充满：钳到 1000 */
    bms_soc_init(&s, &cfg, 100000, 13, 500);
    for (int i = 0; i < 7200; i++)
        bms_soc_step(&s, 200000, PACK_MV_3300, 250, 1000);

    check(bms_soc_get(&s) == 1000, "over-charge clamps to 1000",
          bms_soc_get(&s), 1000);
}

static void test_ocv_correction_happy(void)
{
    bms_soc_t s;
    bms_soc_cfg_t cfg;

    printf("\nOCV correction (all conditions met)\n");
    bms_soc_default_cfg(&cfg);
    bms_soc_init(&s, &cfg, 100000, 13, 800);   /* 故意给个错的基准 80% */

    /* 静置 5 分钟（阈值），包压对应单节 3.30V -> 查表 500 */
    for (int i = 0; i < 30; i++)
        bms_soc_step(&s, 0, PACK_MV_3300, 250, 10000);

    check(bms_soc_get(&s) == 500, "corrected to 500 by OCV",
          bms_soc_get(&s), 500);
    check(s.corrections == 1, "one correction happened", s.corrections, 1);
    check(s.charge_ma_ms == 0, "charge accumulator reset", (long)s.charge_ma_ms, 0);
}

static void test_ocv_correction_gated(void)
{
    bms_soc_t s;
    bms_soc_cfg_t cfg;

    printf("\nOCV correction gates\n");

    /* --- 条件1：静置时间不够 --- */
    bms_soc_default_cfg(&cfg);
    bms_soc_init(&s, &cfg, 100000, 13, 800);

    for (int i = 0; i < 29; i++)   /* 4分50秒，差一点 */
        bms_soc_step(&s, 0, PACK_MV_3300, 250, 10000);

    check(s.corrections == 0, "no correction before rest time met",
          s.corrections, 0);
    check(bms_soc_get(&s) == 800, "soc unchanged without correction",
          bms_soc_get(&s), 800);

    /* --- 条件2：电流不为零（不算静置） --- */
    bms_soc_init(&s, &cfg, 100000, 13, 800);
    for (int i = 0; i < 30; i++)
        bms_soc_step(&s, -2000, PACK_MV_3300, 250, 10000);  /* 2A 放电 */

    check(s.corrections == 0, "no correction while current flows",
          s.corrections, 0);

    /* --- 条件3：温度超出查表有效范围 --- */
    bms_soc_init(&s, &cfg, 100000, 13, 800);
    for (int i = 0; i < 30; i++)
        bms_soc_step(&s, 0, PACK_MV_3300, 500, 10000);   /* 50℃ > 45℃ */

    check(s.corrections == 0, "no correction outside temp window",
          s.corrections, 0);

    /* --- 条件4：静置够但间隔不够（刚修正过就再静置） --- */
    bms_soc_init(&s, &cfg, 100000, 13, 800);
    for (int i = 0; i < 30; i++)                 /* 第一次修正 */
        bms_soc_step(&s, 0, PACK_MV_3300, 250, 10000);
    check(s.corrections == 1, "first correction ok", s.corrections, 1);

    /* 再静置 5 分钟：满足了 rest，但距上次修正只有 5 分钟 < 10 分钟 */
    for (int i = 0; i < 30; i++)
        bms_soc_step(&s, 0, PACK_MV_3300, 250, 10000);
    check(s.corrections == 1, "no second correction within min interval",
          s.corrections, 1);

    /* 继续等到超过最小间隔（再等 5 分钟多） */
    for (int i = 0; i < 31; i++)
        bms_soc_step(&s, 0, PACK_MV_3300, 250, 10000);
    check(s.corrections == 2, "second correction after interval",
          s.corrections, 2);
}

static void test_drift_without_correction(void)
{
    bms_soc_t s;
    bms_soc_cfg_t cfg;

    printf("\nDrift when correction never happens\n");

    /*
     * 模拟真实工况：设备连续运行，电流始终不为零，永远得不到静置机会。
     * 给采样加 +50mA 的偏置（相当于电流传感器零点漂移 50mA）。
     *
     * 50mA × 24h = 1.2Ah = 1.2% 的累积误差，且方向单一、不可自愈。
     * 这就是 docs/design.md 说的「连续运行时误差会持续累积」。
     */
    bms_soc_default_cfg(&cfg);
    bms_soc_init(&s, &cfg, 100000, 13, 500);

    for (int i = 0; i < 24 * 3600; i++)
        bms_soc_step(&s, -1000 + 50, PACK_MV_3300, 250, 1000);

    check(s.corrections == 0, "no correction during continuous run",
          s.corrections, 0);

    {
        int32_t soc = bms_soc_get(&s);
        /* -950mA 放 24 小时 = 22.8 Ah = 22.8% -> 约 228 permille */
        int32_t expect = 500 - 228;

        check(soc >= expect - 2 && soc <= expect + 2,
              "drift matches sensor bias", soc, expect);
    }
}

int main(void)
{
    printf("=== bms_soc_test ===\n");

    test_ocv_table();
    test_coulomb_counting();
    test_clamp();
    test_ocv_correction_happy();
    test_ocv_correction_gated();
    test_drift_without_correction();

    printf("\nResult: %d checks, %d failed\n", g_total, g_fail);
    return g_fail == 0 ? 0 : 1;
}
