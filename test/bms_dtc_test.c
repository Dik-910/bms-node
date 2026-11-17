/*
 * bms_dtc_test.c —— 保护故障位到 DTC 的映射
 *
 * 要验证的不是 dtc.c 本身（那个在 can-node 里测过完整生命周期），
 * 而是**这张表**：
 *   - 每个保护位都能落到一个码上，漏一个就是「保护动作了但读不到码」
 *   - 码表里每条都有对应的故障位，多一条就是永远报不出来的死码
 *   - 告警位不进存储器（否则历史码会被轻微波动写满）
 *   - 通过上报能推进老化，故障最终会自己消失
 *
 * 一个容易误解的语义：store->count 是「测过的码数」，不是「有故障的码数」。
 * dtc_report 即使报通过也会建条目 —— 否则 0x19 0A（报支持的 DTC）在无故障时
 * 返回空，tester 会以为这个节点不支持故障码。所以断言要看清的是
 * dtc_count_by_mask(CONFIRMED)，不是 count。
 */
#include <stdio.h>
#include <string.h>

#include "bms_dtc.h"
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

static void setup(dtc_store_t *st, dtc_config_t *cfg)
{
    memset(st, 0, sizeof(*st));
    bms_dtc_default_config(cfg);
    dtc_init(st, cfg);
}

/* 跑一个完整的操作循环：先报失败，再换循环报失败 -> confirmed */
static void confirm_in_two_cycles(dtc_store_t *st, uint16_t faults)
{
    dtc_operation_cycle(st);
    bms_dtc_sync(st, faults, 1000);

    dtc_operation_cycle(st);
    bms_dtc_sync(st, faults, 2000);
}

/* ---- 每个保护位都必须有码 ---- */

static void test_every_prot_bit_maps(void)
{
    static const uint16_t bits[] =
    {
        BMS_F_OV_PROT, BMS_F_UV_PROT, BMS_F_OC_PROT,
        BMS_F_OD_PROT, BMS_F_OT_PROT, BMS_F_UT_PROT,
    };
    const size_t n = sizeof(bits) / sizeof(bits[0]);

    printf("每个保护位都能报出码\n");

    for (size_t i = 0; i < n; i++)
    {
        dtc_store_t st;
        dtc_config_t cfg;

        setup(&st, &cfg);
        confirm_in_two_cycles(&st, bits[i]);

        check(dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED) == 1,
              "单bit应确认一条", dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED), 1);
        check(st.count == 6, "六个码都应被测到", st.count, 6);
    }
}

/* ---- 码表与故障位一一对应，没有死码 ---- */

static void test_no_orphan_codes(void)
{
    static const uint32_t expect[] =
    {
        0x00A001u, 0x00A002u, 0x00A003u, 0x00A004u, 0x00A005u, 0x00A006u,
    };
    const size_t n = sizeof(expect) / sizeof(expect[0]);
    const dtc_def_t *defs = 0;
    uint16_t count = bms_dtc_defs(&defs);

    printf("码表内容与数量\n");

    check(count == n, "码表条数应为 6", count, (long)n);

    for (size_t i = 0; i < n && i < count; i++)
        check(defs[i].code == expect[i], "码值顺序", (long)defs[i].code,
              (long)expect[i]);

    /* 每个已定义的码都能被某个故障位触发 */
    for (uint16_t i = 0; i < count; i++)
    {
        dtc_store_t st;
        dtc_config_t cfg;
        int hit = 0;

        setup(&st, &cfg);

        /* 六个位逐个单独触发，看这个码是否出现过 */
        for (uint16_t b = 0; b < 6 && !hit; b++)
        {
            uint16_t bit = (uint16_t)(1u << (b + 8)); /* BMS_F_*_PROT 在 bit8..13 */

            memset(&st, 0, sizeof(st));
            dtc_init(&st, &cfg);
            confirm_in_two_cycles(&st, bit);

            hit = (dtc_find(&st, defs[i].code) != 0);
        }

        check(hit, "码必须有位能触发", (long)i, 1);
    }
}

/* ---- 告警位不该进存储器 ---- */

static void test_warn_bits_ignored(void)
{
    dtc_store_t st;
    dtc_config_t cfg;

    printf("告警位不进存储器\n");

    setup(&st, &cfg);
    confirm_in_two_cycles(&st, BMS_F_WARN_MASK);

    check(dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED) == 0,
          "确认条数应为 0", dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED), 0);
    check(dtc_count_by_mask(&st, DTC_STATUS_PENDING) == 0,
          "也不该有 pending", dtc_count_by_mask(&st, DTC_STATUS_PENDING), 0);
}

/* ---- 多故障并存 ---- */

static void test_multiple_faults(void)
{
    dtc_store_t st;
    dtc_config_t cfg;
    uint16_t both = BMS_F_OV_PROT | BMS_F_OT_PROT;

    printf("两个故障同时存在\n");

    setup(&st, &cfg);
    confirm_in_two_cycles(&st, both);

    check(dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED) == 2,
          "两条都应确认", dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED), 2);
    check(dtc_find(&st, 0x00A001u) != 0, "过压码应在表内", 1, 1);
    check(dtc_find(&st, 0x00A005u) != 0, "过温码应在表内", 1, 1);
}

/* ---- 单次失败只是 pending ---- */

static void test_pending_only(void)
{
    dtc_store_t st;
    dtc_config_t cfg;

    printf("单循环失败只是 pending\n");

    setup(&st, &cfg);
    dtc_operation_cycle(&st);
    bms_dtc_sync(&st, BMS_F_OV_PROT, 1000);

    check(dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED) == 0,
          "还不该确认", dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED), 0);
    check(dtc_count_by_mask(&st, DTC_STATUS_PENDING) == 1,
          "应为 pending", dtc_count_by_mask(&st, DTC_STATUS_PENDING), 1);
}

/* ---- 故障消失后按循环老化 ---- */

static void test_aging_clears(void)
{
    dtc_store_t st;
    dtc_config_t cfg;

    printf("无故障循环推进老化\n");

    setup(&st, &cfg);
    confirm_in_two_cycles(&st, BMS_F_UV_PROT);

    check(dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED) == 1,
          "先确认一条", dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED), 1);

    /*
     * 连续 40 个「测过且通过」的循环。
     *
     * 多跑几轮是因为 aging 的结算在 cycle 切换时进行：
     * 确认故障的那一轮本身是失败的，要等下一次 cycle 切换才被结算，
     * 所以第 1 轮只会把计数归零，真正开始累加是第 2 轮起。
     */
    for (int i = 0; i < 45; i++)
    {
        dtc_operation_cycle(&st);
        bms_dtc_sync(&st, 0, (uint32_t)(3000 + i * 100));
    }

    check(dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED) == 0,
          "老化后应自动清除", dtc_count_by_mask(&st, DTC_STATUS_CONFIRMED), 0);
    check(dtc_find(&st, 0x00A002u) != 0,
          "条目本身应保留（0x19 0A 还要报支持）", 1, 1);
}

/* ---- 同步条数必须等于码表条数（防止表被截断） ---- */

static void test_sync_count(void)
{
    dtc_store_t st;
    dtc_config_t cfg;
    const dtc_def_t *defs = 0;
    int n;

    printf("同步覆盖全部码\n");

    setup(&st, &cfg);
    n = bms_dtc_sync(&st, 0, 0);

    check(n == bms_dtc_defs(&defs), "每轮上报条数应等于码表条数",
          n, (long)bms_dtc_defs(&defs));
}

int main(void)
{
    printf("=== bms_dtc_test ===\n");

    test_every_prot_bit_maps();
    test_no_orphan_codes();
    test_warn_bits_ignored();
    test_multiple_faults();
    test_pending_only();
    test_aging_clears();
    test_sync_count();

    printf("\nResult: %d checks, %d failed\n", g_total, g_fail);
    return g_fail == 0 ? 0 : 1;
}
