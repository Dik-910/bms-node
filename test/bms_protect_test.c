/*
 * bms_protect_test.c —— 保护状态机
 *
 * 按 docs/design.md 的迁移表逐条验证，另外专门测三类容易写错的地方：
 *   - 迟滞：值卡在阈值附近不能反复翻转
 *   - 双路：只有原始值越限（单次毛刺）不许动作
 *   - 恢复：延时未到不能合闸；延时期间再越限要重新断开并清零计时
 */
#include <stdio.h>
#include <string.h>
#include "bms_protect.h"

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

static bms_sample_t mk(uint32_t mv, int32_t ma, int16_t dc)
{
    bms_sample_t s;

    s.pack_mv = mv;
    s.current_ma = ma;
    s.temp_dc = dc;
    return s;
}

/* 正常工况：13 串 × 3.23V，零电流，25℃ */
static bms_sample_t normal(void)
{
    return mk(42000, 0, 250);
}

static void step(bms_prot_t *p, bms_sample_t s, uint32_t n, uint32_t dt_ms)
{
    for (uint32_t i = 0; i < n; i++)
        bms_prot_step(p, &s, &s, dt_ms);
}

static void test_warn_path(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nNormal -> Warn -> Normal\n");

    bms_prot_init(&p, 0);
    step(&p, normal(), 5, 100);
    check(p.state == BMS_PROT_NORMAL, "starts normal", p.state, BMS_PROT_NORMAL);
    check(bms_prot_relay(&p) == BMS_RELAY_CLOSED, "relay closed", 0, 0);

    /* 越过告警阈值（46150）但不到保护（47450） */
    s = mk(46500, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_WARN, "warn on over-voltage", p.state, BMS_PROT_WARN);
    check((p.faults & BMS_F_OV_WARN) != 0, "OV_WARN bit set", p.faults & BMS_F_OV_WARN, BMS_F_OV_WARN);
    check(bms_prot_relay(&p) == BMS_RELAY_CLOSED, "relay stays closed on warn", 0, 0);

    /* 回落到「阈值 − 迟滞」以下才清（46150 − 500 = 45650） */
    s = mk(45700, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_WARN, "hysteresis holds warn", p.state, BMS_PROT_WARN);

    s = mk(45600, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_NORMAL, "clears below threshold-hyst",
          p.state, BMS_PROT_NORMAL);
    check(p.faults == 0, "all faults cleared", p.faults, 0);
}

static void test_protect_and_recover(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nNormal -> Protect -> Recover -> Normal\n");

    bms_prot_init(&p, 0);
    step(&p, normal(), 3, 100);

    /* 越过保护阈值 */
    s = mk(48000, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "protect on over-voltage",
          p.state, BMS_PROT_PROTECT);
    check(bms_prot_relay(&p) == BMS_RELAY_OPEN, "relay opens", 0, 0);
    check(p.trips == 1, "trip counted", p.trips, 1);

    /*
     * 回落必须低于「告警阈值 − 迟滞」= 45650，否则告警位不会清，
     * 恢复期就永远等不到 !has_warn（下面 test_warn_blocks_recover 专门测这个）。
     * 用 45000，明确在迟滞带之外。
     */
    s = mk(45000, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_RECOVER, "enters recover", p.state, BMS_PROT_RECOVER);
    check(bms_prot_relay(&p) == BMS_RELAY_OPEN, "relay still open during recover", 0, 0);

    /* 延时未到（默认 10s），保持恢复 */
    step(&p, s, 50, 100);   /* 5s */
    check(p.state == BMS_PROT_RECOVER, "stays recover before delay",
          p.state, BMS_PROT_RECOVER);

    /* 延时到，且无告警 -> 回正常 */
    step(&p, s, 60, 100);   /* 再 6s，累计 11s */
    check(p.state == BMS_PROT_NORMAL, "returns to normal after delay",
          p.state, BMS_PROT_NORMAL);
    check(bms_prot_relay(&p) == BMS_RELAY_CLOSED, "relay closes", 0, 0);
}

static void test_recover_retrip(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nRe-trip during recover resets the timer\n");

    bms_prot_init(&p, 0);
    step(&p, normal(), 2, 100);

    s = mk(48000, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "in protect", p.state, BMS_PROT_PROTECT);

    s = mk(45000, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_RECOVER, "in recover", p.state, BMS_PROT_RECOVER);

    /* 等了 8 秒，差 2 秒就到 10 秒 */
    step(&p, s, 80, 100);

    /* 又越过保护阈值：立即回保护，计时清零 */
    s = mk(48000, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "re-trip -> protect", p.state, BMS_PROT_PROTECT);
    check(p.trips == 2, "second trip counted", p.trips, 2);

    /* 再次回落，必须重新等满 10 秒 —— 之前的 8 秒不能算 */
    s = mk(45000, 0, 250);
    step(&p, s, 90, 100);   /* 9s */
    check(p.state == BMS_PROT_RECOVER, "still recovering at 9s",
          p.state, BMS_PROT_RECOVER);

    step(&p, s, 20, 100);   /* 累计 11s */
    check(p.state == BMS_PROT_NORMAL, "normal after full delay",
          p.state, BMS_PROT_NORMAL);
}

static void test_dual_path(void)
{
    bms_prot_t p;
    bms_sample_t raw, filt;

    printf("\nDual-path: raw spike alone must not trip\n");

    bms_prot_init(&p, 0);

    /* 正常建立起来 */
    raw = normal();
    filt = normal();
    for (int i = 0; i < 3; i++)
        bms_prot_step(&p, &raw, &filt, 100);

    /*
     * 单次采样尖峰：原始值冲到 48000，滤波值还在正常范围。
     * 设计文档要求「两者同时越限才动作」—— 这就是抗单点毛刺。
     */
    raw = mk(48000, 0, 250);
    filt = mk(42000, 0, 250);
    bms_prot_step(&p, &raw, &filt, 100);

    check(p.state == BMS_PROT_NORMAL, "raw spike alone does not trip",
          p.state, BMS_PROT_NORMAL);
    check((p.faults & BMS_F_OV_PROT) == 0, "no OV_PROT bit",
          p.faults & BMS_F_OV_PROT, 0);

    /* 两路都越限时才动作 */
    raw = mk(48000, 0, 250);
    filt = mk(48000, 0, 250);
    bms_prot_step(&p, &raw, &filt, 100);
    check(p.state == BMS_PROT_PROTECT, "both paths tripped",
          p.state, BMS_PROT_PROTECT);
}

static void test_current_direction(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nOver-current is directional\n");

    /* 充电过流：+60A 超过 oc_warn(50A) 但不到 oc_prot(80A) */
    bms_prot_init(&p, 0);
    s = mk(42000, 60000, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_WARN, "charge over-current warns",
          p.state, BMS_PROT_WARN);
    check((p.faults & BMS_F_OC_WARN) != 0, "OC_WARN set",
          p.faults & BMS_F_OC_WARN, BMS_F_OC_WARN);
    check((p.faults & BMS_F_OD_WARN) == 0, "OD_WARN not set",
          p.faults & BMS_F_OD_WARN, 0);

    /* 放电过流：−110A 超过 od_warn(100A) */
    bms_prot_init(&p, 0);
    s = mk(42000, -110000, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_WARN, "discharge over-current warns",
          p.state, BMS_PROT_WARN);
    check((p.faults & BMS_F_OD_WARN) != 0, "OD_WARN set",
          p.faults & BMS_F_OD_WARN, BMS_F_OD_WARN);
    check((p.faults & BMS_F_OC_WARN) == 0, "OC_WARN not set",
          p.faults & BMS_F_OC_WARN, 0);

    /* 充电 90A 超过 oc_prot -> 直接保护 */
    bms_prot_init(&p, 0);
    s = mk(42000, 90000, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "charge over-current trips",
          p.state, BMS_PROT_PROTECT);

    /* 放电 −160A 超过 od_prot -> 直接保护 */
    bms_prot_init(&p, 0);
    s = mk(42000, -160000, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "discharge over-current trips",
          p.state, BMS_PROT_PROTECT);
}

static void test_temperature(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nTemperature limits\n");

    /* 46℃ 超过 ot_warn(45℃) */
    bms_prot_init(&p, 0);
    s = mk(42000, 0, 460);
    step(&p, s, 1, 100);
    check((p.faults & BMS_F_OT_WARN) != 0, "OT_WARN set",
          p.faults & BMS_F_OT_WARN, BMS_F_OT_WARN);

    /* 56℃ 超过 ot_prot(55℃) */
    bms_prot_init(&p, 0);
    s = mk(42000, 0, 560);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "over-temp trips",
          p.state, BMS_PROT_PROTECT);

    /* −1℃ 低于 ut_warn(0℃) */
    bms_prot_init(&p, 0);
    s = mk(42000, 0, -10);
    step(&p, s, 1, 100);
    check((p.faults & BMS_F_UT_WARN) != 0, "UT_WARN set",
          p.faults & BMS_F_UT_WARN, BMS_F_UT_WARN);

    /* −11℃ 低于 ut_prot(−10℃) */
    bms_prot_init(&p, 0);
    s = mk(42000, 0, -110);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "under-temp trips",
          p.state, BMS_PROT_PROTECT);
}

static void test_undervoltage(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nUnder-voltage\n");

    bms_prot_init(&p, 0);
    s = mk(38500, 0, 250);   /* 低于 uv_warn(39000) */
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_WARN, "under-voltage warns", p.state, BMS_PROT_WARN);

    s = mk(36000, 0, 250);   /* 低于 uv_prot(36400) */
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "under-voltage trips",
          p.state, BMS_PROT_PROTECT);
}

static void test_warn_blocks_recover(void)
{
    bms_prot_t p;
    bms_sample_t s;

    printf("\nRecover requires no warning as well\n");

    /*
     * 场景：过压保护后回落到「保护阈值以下、但仍在告警阈值以上」。
     * 此时保护条件解除，但告警还在 —— 不能合闸。
     * 只看延时、不看告警的实现会在这里错误地恢复。
     */
    bms_prot_init(&p, 0);
    s = mk(48000, 0, 250);
    step(&p, s, 1, 100);
    check(p.state == BMS_PROT_PROTECT, "tripped", p.state, BMS_PROT_PROTECT);

    s = mk(47000, 0, 250);   /* < 47450−500，但 > 46150 告警阈值 */
    step(&p, s, 200, 100);   /* 等 20 秒，远超 10 秒延时 */

    check((p.faults & BMS_F_OV_WARN) != 0, "still warning",
          p.faults & BMS_F_OV_WARN, BMS_F_OV_WARN);
    check(p.state != BMS_PROT_NORMAL, "does not return to normal with warning",
          p.state, BMS_PROT_NORMAL);

    /* 降到告警阈值以下，再等一个延时周期才恢复 */
    s = mk(45000, 0, 250);
    step(&p, s, 150, 100);
    check(p.state == BMS_PROT_NORMAL, "normal after warning clears",
          p.state, BMS_PROT_NORMAL);
}

int main(void)
{
    printf("=== bms_protect_test ===\n");

    test_warn_path();
    test_protect_and_recover();
    test_recover_retrip();
    test_dual_path();
    test_current_direction();
    test_temperature();
    test_undervoltage();
    test_warn_blocks_recover();

    printf("\nResult: %d checks, %d failed\n", g_total, g_fail);
    return g_fail == 0 ? 0 : 1;
}
