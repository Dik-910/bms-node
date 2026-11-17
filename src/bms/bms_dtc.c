#include "bms_dtc.h"

#include <stddef.h>

#include "bms_protect.h"

/*
 * P0A0x 段（SAE J2012）是高压 / 动力电池相关的一组码。
 *
 * 用 3 字节格式（0x00A001 = P0A001），与 uds.c 里 DTC 格式标识
 * UDS_DTC_FORMAT_ISO14229 一致。can-node 那边的通信类故障用 0xC0xxxx，
 * 两个节点挂同一条总线上也不会撞码。
 */
static const dtc_def_t g_defs[] =
{
    {0x00A001u, "Battery pack over-voltage"},
    {0x00A002u, "Battery pack under-voltage"},
    {0x00A003u, "Charge over-current"},
    {0x00A004u, "Discharge over-current"},
    {0x00A005u, "Battery over-temperature"},
    {0x00A006u, "Battery under-temperature"},
};

typedef struct
{
    uint16_t bit;  /* bms_protect.h 里的故障位 */
    uint32_t code; /* 对应的诊断故障码 */
} dtc_map_t;

/*
 * 顺序与 g_defs 一一对应。加保护项时两张表一起改 ——
 * 单元测试会按位逐个验证，漏了会直接失败。
 */
static const dtc_map_t g_map[] =
{
    {BMS_F_OV_PROT, 0x00A001u},
    {BMS_F_UV_PROT, 0x00A002u},
    {BMS_F_OC_PROT, 0x00A003u},
    {BMS_F_OD_PROT, 0x00A004u},
    {BMS_F_OT_PROT, 0x00A005u},
    {BMS_F_UT_PROT, 0x00A006u},
};

#define MAP_COUNT (sizeof(g_map) / sizeof(g_map[0]))

/* 两张表必须一样长，否则下面按下标取码会越界。编译期就能查出来 */
typedef char map_matches_defs[(MAP_COUNT == (sizeof(g_defs) / sizeof(g_defs[0]))) ? 1 : -1];

uint16_t bms_dtc_defs(const dtc_def_t **out)
{
    if (out != 0)
        *out = g_defs;

    return (uint16_t)(sizeof(g_defs) / sizeof(g_defs[0]));
}

void bms_dtc_default_config(dtc_config_t *cfg)
{
    if (cfg == 0)
        return;

    cfg->def_count = (uint16_t)(sizeof(g_defs) / sizeof(g_defs[0]));
    cfg->defs = g_defs;
    cfg->confirm_cycles = 2;
    cfg->aging_cycles = DTC_DEFAULT_AGING_CYCLES;
}

int bms_dtc_sync(dtc_store_t *st, uint16_t faults, uint32_t now_ms)
{
    int reported = 0;

    if (st == 0)
        return -1;

    for (size_t i = 0; i < MAP_COUNT; i++)
    {
        if (dtc_report(st, g_map[i].code, (faults & g_map[i].bit) != 0, now_ms) == 0)
            reported++;
    }

    return reported;
}
