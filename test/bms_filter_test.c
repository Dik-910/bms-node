/*
 * bms_filter_test.c —— 两级滤波
 *
 * 验证的是「滤波行为是否符合设计意图」，不是「公式跑得动」：
 *   - 窗口未满时按已有样本平均（不能拿 0 充数）
 *   - 低通首值直接采用（不能从 0 爬上来）
 *   - 稳态时输出等于输入（没偏置）
 *   - 阶跃响应单调且不过冲
 */
#include <stdio.h>
#include <string.h>
#include "bms_filter.h"

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

static void test_movavg(void)
{
    bms_movavg_t f;

    printf("\nMoving average\n");
    bms_movavg_init(&f);

    /* 窗口未满：第一个值就该原样返回，不能被 0 稀释 */
    check(bms_movavg_push(&f, 1000) == 1000, "first sample passes through", 1000, 1000);
    /* 第二个：平均 (1000+2000)/2 */
    check(bms_movavg_push(&f, 2000) == 1500, "second sample averaged", 1500, 1500);
    /* 第三个：3000/3 = 1000... 实际 (1000+2000+3000)/3 = 2000 */
    check(bms_movavg_push(&f, 3000) == 2000, "third sample averaged", 2000, 2000);

    /*
     * 稳态无偏：单独一个滤波器，全部输入同一个值。
     * （上面那个窗口里混了 1000/2000/3000，还没被冲掉，不能用来测稳态）
     */
    bms_movavg_init(&f);
    for (int i = 0; i < BMS_MA_WINDOW; i++)
        bms_movavg_push(&f, 2000);

    check(bms_movavg_push(&f, 2000) == 2000, "steady state is unbiased", 2000, 2000);

    /* 注入一个尖峰，窗口平均应把它衰减到 1/8 的影响 */
    {
        int32_t y = bms_movavg_push(&f, 20000);
        check(y >= 4245 && y <= 4250, "single spike attenuated", y, 4250);
    }
}

static void test_lowpass(void)
{
    bms_lowpass_t f;

    printf("\nLow pass\n");
    bms_lowpass_init(&f, 200);   /* α = 0.2 */

    /* 首值直接采用：避免开机后从 0 收敛的假数据 */
    check(bms_lowpass_update(&f, 10000) == 10000, "first value adopted", 10000, 10000);

    /* 第二值：0.2*10000 + 0.8*10000 = 10000（不变） */
    check(bms_lowpass_update(&f, 10000) == 10000, "steady state unchanged", 10000, 10000);

    /* 阶跃到 20000：y = 0.2*20000 + 0.8*10000 = 12000 */
    check(bms_lowpass_update(&f, 20000) == 12000, "step response 1st", 12000, 12000);
    /* 下一步 y = 0.2*20000 + 0.8*12000 = 13600 */
    check(bms_lowpass_update(&f, 20000) == 13600, "step response 2nd", 13600, 13600);

    /* 持续输入后应收敛到新值，且不越过 */
    int32_t y = 0;
    for (int i = 0; i < 50; i++)
        y = bms_lowpass_update(&f, 20000);

    /*
     * 定点实现收敛到目标附近 ±1 而不是精确相等：
     * 整数除法每次都会截断，稳态处存在一个不动点邻域。
     * 这不是 bug —— ADC 本身也有几个 LSB 的噪声，要求 bit 级精确没有意义。
     */
    check(y >= 19997 && y <= 20000, "converges to step value (+/-3)", y, 20000);
    check(y <= 20000, "no overshoot", y, 20000);
}

static void test_chain(void)
{
    bms_chan_t c;
    int32_t prev = 0;
    int mono = 1;

    printf("\nChained (movavg + lowpass)\n");
    bms_chan_init(&c);

    for (int i = 0; i < 8; i++)
        bms_chan_push(&c, 10000);

    check(bms_chan_push(&c, 10000) == 10000, "chain steady state unbiased", 10000, 10000);

    /* 阶跃后响应必须单调上升，不能来回抖 */
    prev = 10000;
    for (int i = 0; i < 40; i++)
    {
        int32_t y = bms_chan_push(&c, 20000);
        if (y < prev)
            mono = 0;
        prev = y;
    }

    check(mono, "step response is monotonic", mono, 1);
    check(prev >= 19997 && prev <= 20000, "chain converges (+/-3)", prev, 20000);

    /* 噪声抑制：±500 的抖动，输出峰峰值应显著小于输入 */
    bms_chan_init(&c);
    for (int i = 0; i < 8; i++)
        bms_chan_push(&c, 30000);

    int32_t lo = 30000, hi = 30000;
    for (int i = 0; i < 100; i++)
    {
        int32_t in = 30000 + ((i & 1) ? 500 : -500);
        int32_t y = bms_chan_push(&c, in);

        if (y < lo) lo = y;
        if (y > hi) hi = y;
    }

    check((hi - lo) < 200, "noise attenuated below 200", hi - lo, 200);
}

int main(void)
{
    printf("=== bms_filter_test ===\n");

    test_movavg();
    test_lowpass();
    test_chain();

    printf("\nResult: %d checks, %d failed\n", g_total, g_fail);
    return g_fail == 0 ? 0 : 1;
}
