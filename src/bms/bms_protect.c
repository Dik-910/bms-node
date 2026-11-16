/*
 * bms_protect.c —— 保护状态机实现
 */
#include "bms_protect.h"

#include <string.h>

/*
 * 每个量的「置位」与「清除」判断。
 *
 * 置位：raw 与 filt 同时越限（抗单次毛刺 + 不漏真实快变）
 * 清除：filt 回到 阈值 ∓ 迟滞 以内（防止在阈值附近反复翻转）
 *
 * 这两个条件不对称是故意的。对称写会得到一个「进得慢、出得快」的怪状态机。
 */

static void eval_high(uint32_t raw, uint32_t filt, uint32_t thresh, uint32_t hyst,
                      uint16_t bit, uint16_t *faults)
{
    if ((*faults & bit) != 0)
    {
        /* 已置位：要降到 阈值−迟滞 以下才清 */
        if (filt + hyst < thresh)
            *faults = (uint16_t)(*faults & ~bit);
    }
    else if (raw > thresh && filt > thresh)
    {
        *faults = (uint16_t)(*faults | bit);
    }
}

static void eval_low(uint32_t raw, uint32_t filt, uint32_t thresh, uint32_t hyst,
                     uint16_t bit, uint16_t *faults)
{
    if ((*faults & bit) != 0)
    {
        /* 已置位：要升到 阈值+迟滞 以上才清 */
        if (filt > thresh + hyst)
            *faults = (uint16_t)(*faults & ~bit);
    }
    else if (raw < thresh && filt < thresh)
    {
        *faults = (uint16_t)(*faults | bit);
    }
}

/* 电流按方向比较：charging 为正，discharging 用 −current */
static void eval_cur(int32_t raw, int32_t filt, int32_t thresh, int32_t hyst,
                     uint16_t bit, uint16_t *faults)
{
    if ((*faults & bit) != 0)
    {
        if (filt < thresh - hyst)
            *faults = (uint16_t)(*faults & ~bit);
    }
    else if (raw > thresh && filt > thresh)
    {
        *faults = (uint16_t)(*faults | bit);
    }
}

void bms_prot_default_cfg(bms_prot_cfg_t *cfg)
{
    if (cfg == 0)
        return;

    memset(cfg, 0, sizeof(*cfg));

    /* 13 串 LFP：单节 3.65V 满、2.8V 空 -> 包级 47450 / 36400 mV */
    cfg->ov_warn_mv = 46150;   /* 单节约 3.55V */
    cfg->ov_prot_mv = 47450;   /* 单节 3.65V */
    cfg->uv_warn_mv = 39000;   /* 单节 3.00V */
    cfg->uv_prot_mv = 36400;   /* 单节 2.80V */
    cfg->hyst_mv    = 500;     /* 单节约 38mV，约为噪声峰峰值的 2 倍 */

    cfg->oc_warn_ma = 50000;   /* 50 A */
    cfg->oc_prot_ma = 80000;   /* 80 A */
    cfg->od_warn_ma = 100000;  /* 100 A */
    cfg->od_prot_ma = 150000;  /* 150 A */
    cfg->hyst_ma    = 2000;    /* 2 A */

    cfg->ot_warn_dc = 450;     /* 45℃ */
    cfg->ot_prot_dc = 550;     /* 55℃ */
    cfg->ut_warn_dc = 0;       /* 0℃ */
    cfg->ut_prot_dc = -100;    /* −10℃ */
    cfg->hyst_dc    = 20;      /* 2℃ */

    cfg->recover_delay_ms = 10000u;  /* 10 秒 */
}

void bms_prot_init(bms_prot_t *p, const bms_prot_cfg_t *cfg)
{
    if (p == 0)
        return;

    memset(p, 0, sizeof(*p));

    if (cfg != 0)
        p->cfg = *cfg;
    else
        bms_prot_default_cfg(&p->cfg);

    p->state = BMS_PROT_NORMAL;
}

bms_prot_state_t bms_prot_step(bms_prot_t *p, const bms_sample_t *raw,
                               const bms_sample_t *filt, uint32_t dt_ms)
{
    uint16_t f;
    int32_t  charge_raw, charge_filt, dis_raw, dis_filt;
    int has_warn, has_prot;

    if (p == 0 || raw == 0 || filt == 0)
        return BMS_PROT_NORMAL;

    f = p->faults;

    /* ---- 电压 ---- */
    eval_high(raw->pack_mv, filt->pack_mv, p->cfg.ov_warn_mv, p->cfg.hyst_mv,
              BMS_F_OV_WARN, &f);
    eval_low(raw->pack_mv, filt->pack_mv, p->cfg.uv_warn_mv, p->cfg.hyst_mv,
             BMS_F_UV_WARN, &f);
    eval_high(raw->pack_mv, filt->pack_mv, p->cfg.ov_prot_mv, p->cfg.hyst_mv,
              BMS_F_OV_PROT, &f);
    eval_low(raw->pack_mv, filt->pack_mv, p->cfg.uv_prot_mv, p->cfg.hyst_mv,
             BMS_F_UV_PROT, &f);

    /* ---- 电流：拆成充电/放电两个方向，各自独立判断 ---- */
    charge_raw  = (raw->current_ma  > 0) ? raw->current_ma  : 0;
    charge_filt = (filt->current_ma > 0) ? filt->current_ma : 0;
    dis_raw     = (raw->current_ma  < 0) ? -raw->current_ma  : 0;
    dis_filt    = (filt->current_ma < 0) ? -filt->current_ma : 0;

    eval_cur(charge_raw, charge_filt, p->cfg.oc_warn_ma, p->cfg.hyst_ma,
             BMS_F_OC_WARN, &f);
    eval_cur(charge_raw, charge_filt, p->cfg.oc_prot_ma, p->cfg.hyst_ma,
             BMS_F_OC_PROT, &f);
    eval_cur(dis_raw, dis_filt, p->cfg.od_warn_ma, p->cfg.hyst_ma,
             BMS_F_OD_WARN, &f);
    eval_cur(dis_raw, dis_filt, p->cfg.od_prot_ma, p->cfg.hyst_ma,
             BMS_F_OD_PROT, &f);

    /* ---- 温度 ---- */
    if (f & BMS_F_OT_WARN)
    {
        if ((int32_t)filt->temp_dc + p->cfg.hyst_dc < p->cfg.ot_warn_dc)
            f = (uint16_t)(f & ~BMS_F_OT_WARN);
    }
    else if (raw->temp_dc > p->cfg.ot_warn_dc && filt->temp_dc > p->cfg.ot_warn_dc)
    {
        f = (uint16_t)(f | BMS_F_OT_WARN);
    }

    if (f & BMS_F_OT_PROT)
    {
        if ((int32_t)filt->temp_dc + p->cfg.hyst_dc < p->cfg.ot_prot_dc)
            f = (uint16_t)(f & ~BMS_F_OT_PROT);
    }
    else if (raw->temp_dc > p->cfg.ot_prot_dc && filt->temp_dc > p->cfg.ot_prot_dc)
    {
        f = (uint16_t)(f | BMS_F_OT_PROT);
    }

    if (f & BMS_F_UT_WARN)
    {
        if ((int32_t)filt->temp_dc - p->cfg.hyst_dc > p->cfg.ut_warn_dc)
            f = (uint16_t)(f & ~BMS_F_UT_WARN);
    }
    else if (raw->temp_dc < p->cfg.ut_warn_dc && filt->temp_dc < p->cfg.ut_warn_dc)
    {
        f = (uint16_t)(f | BMS_F_UT_WARN);
    }

    if (f & BMS_F_UT_PROT)
    {
        if ((int32_t)filt->temp_dc - p->cfg.hyst_dc > p->cfg.ut_prot_dc)
            f = (uint16_t)(f & ~BMS_F_UT_PROT);
    }
    else if (raw->temp_dc < p->cfg.ut_prot_dc && filt->temp_dc < p->cfg.ut_prot_dc)
    {
        f = (uint16_t)(f | BMS_F_UT_PROT);
    }

    p->faults = f;

    has_warn = (f & BMS_F_WARN_MASK) != 0;
    has_prot = (f & BMS_F_PROT_MASK) != 0;

    /* ---- 状态迁移 ---- */
    switch (p->state)
    {
    case BMS_PROT_NORMAL:
        if (has_prot)
        {
            p->state = BMS_PROT_PROTECT;
            p->state_ms = 0;
            p->trips++;
        }
        else if (has_warn)
        {
            p->state = BMS_PROT_WARN;
            p->state_ms = 0;
        }
        else
        {
            if (p->state_ms < 0xFFFFFFFFu - dt_ms)
                p->state_ms += dt_ms;
        }
        break;

    case BMS_PROT_WARN:
        if (has_prot)
        {
            p->state = BMS_PROT_PROTECT;
            p->state_ms = 0;
            p->trips++;
        }
        else if (!has_warn)
        {
            p->state = BMS_PROT_NORMAL;
            p->state_ms = 0;
        }
        else
        {
            if (p->state_ms < 0xFFFFFFFFu - dt_ms)
                p->state_ms += dt_ms;
        }
        break;

    case BMS_PROT_PROTECT:
        if (!has_prot)
        {
            /* 保护条件解除，进入恢复确认期。此时仍然断开主回路 */
            p->state = BMS_PROT_RECOVER;
            p->state_ms = 0;
        }
        else
        {
            p->state_ms = 0;
        }
        break;

    case BMS_PROT_RECOVER:
        if (has_prot)
        {
            /* 期间再次越限：重新断开，计时清零 */
            p->state = BMS_PROT_PROTECT;
            p->state_ms = 0;
            p->trips++;
        }
        else
        {
            p->state_ms += dt_ms;

            /*
             * 必须同时满足「延时到」和「无告警」才回正常。
             * 只看延时的话，带着告警合闸会立刻又跳，等于白等。
             */
            if (p->state_ms >= p->cfg.recover_delay_ms && !has_warn)
            {
                p->state = BMS_PROT_NORMAL;
                p->state_ms = 0;
            }
        }
        break;

    default:
        p->state = BMS_PROT_NORMAL;
        p->state_ms = 0;
        break;
    }

    return p->state;
}

bms_prot_state_t bms_prot_state(const bms_prot_t *p)
{
    return (p != 0) ? p->state : BMS_PROT_NORMAL;
}

uint16_t bms_prot_faults(const bms_prot_t *p)
{
    return (p != 0) ? p->faults : 0;
}

bms_relay_t bms_prot_relay(const bms_prot_t *p)
{
    if (p == 0)
        return BMS_RELAY_CLOSED;

    return (p->state == BMS_PROT_PROTECT || p->state == BMS_PROT_RECOVER)
               ? BMS_RELAY_OPEN
               : BMS_RELAY_CLOSED;
}

const char *bms_prot_fault_name(uint16_t bit)
{
    switch (bit)
    {
    case BMS_F_OV_WARN: return "OV_WARN";
    case BMS_F_UV_WARN: return "UV_WARN";
    case BMS_F_OC_WARN: return "OC_WARN";
    case BMS_F_OD_WARN: return "OD_WARN";
    case BMS_F_OT_WARN: return "OT_WARN";
    case BMS_F_UT_WARN: return "UT_WARN";
    case BMS_F_OV_PROT: return "OV_PROT";
    case BMS_F_UV_PROT: return "UV_PROT";
    case BMS_F_OC_PROT: return "OC_PROT";
    case BMS_F_OD_PROT: return "OD_PROT";
    case BMS_F_OT_PROT: return "OT_PROT";
    case BMS_F_UT_PROT: return "UT_PROT";
    default: return "?";
    }
}
