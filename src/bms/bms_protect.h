/*
 * bms_protect.h —— 四级保护状态机
 *
 * 状态与迁移严格对应 docs/design.md 第 1 节的表：
 *
 *   正常 --告警--> 告警 --全部回到阈值−迟滞--> 正常
 *   正常/告警 --保护--> 保护 --全部回到阈值−迟滞--> 恢复 --稳定 T 秒--> 正常
 *   恢复 --再次越限--> 保护（计时清零）
 *
 * 三个设计点，每个都有具体的失效模式作为理由：
 *
 * 1. 迟滞（hysteresis）
 *    阈值附近有采样噪声。没有迟滞带，值在阈值上下抖动会造成状态高频翻转：
 *    继电器反复吸合、故障码疯狂刷写。迟滞宽度取噪声峰峰值的 2 倍。
 *
 * 2. 恢复延时
 *    保护动作后立刻恢复，若故障仍存在就变成「断开—恢复—再断开」，
 *    对接触器寿命伤害很大。延时确认等于一次软件滤波。
 *
 * 3. 双路判断（原始值 + 滤波值都要越限）
 *    只看滤波值：真实故障快速上升时会被平滑掉，动作滞后。
 *    只看原始值：单次采样毛刺就跳闸，误动作。
 *    两者同时越限才动作，兼顾速度与抗扰 —— 这是 docs/design.md 第 2 节的要求。
 *
 * 注意：双路只用于**进入**判断。退出判断用滤波值 + 迟滞，
 * 否则毛刺会让保护状态反复退出又进入。
 */
#ifndef BMS_PROTECT_H
#define BMS_PROTECT_H

#include <stdint.h>

#include "bms_types.h"

typedef enum
{
    BMS_PROT_NORMAL  = 0,
    BMS_PROT_WARN    = 1,
    BMS_PROT_PROTECT = 2,
    BMS_PROT_RECOVER = 3,
} bms_prot_state_t;

/* 故障位。低位是告警，高位是保护，便于用掩码区分 */
#define BMS_F_OV_WARN   (1u << 0)
#define BMS_F_UV_WARN   (1u << 1)
#define BMS_F_OC_WARN   (1u << 2)   /* 充电过流 */
#define BMS_F_OD_WARN   (1u << 3)   /* 放电过流 */
#define BMS_F_OT_WARN   (1u << 4)
#define BMS_F_UT_WARN   (1u << 5)

#define BMS_F_OV_PROT   (1u << 8)
#define BMS_F_UV_PROT   (1u << 9)
#define BMS_F_OC_PROT   (1u << 10)
#define BMS_F_OD_PROT   (1u << 11)
#define BMS_F_OT_PROT   (1u << 12)
#define BMS_F_UT_PROT   (1u << 13)

#define BMS_F_WARN_MASK 0x003Fu
#define BMS_F_PROT_MASK 0x3F00u

typedef struct
{
    /* 电压（包级，mV） */
    uint32_t ov_warn_mv, ov_prot_mv;
    uint32_t uv_warn_mv, uv_prot_mv;
    uint32_t hyst_mv;

    /* 电流（mA，均为正值，内部按方向比较） */
    int32_t  oc_warn_ma, oc_prot_ma;   /* 充电方向：current > 阈值 */
    int32_t  od_warn_ma, od_prot_ma;   /* 放电方向：−current > 阈值 */
    int32_t  hyst_ma;

    /* 温度（0.1℃） */
    int16_t  ot_warn_dc, ot_prot_dc;
    int16_t  ut_warn_dc, ut_prot_dc;
    int16_t  hyst_dc;

    /* 恢复确认时间 */
    uint32_t recover_delay_ms;
} bms_prot_cfg_t;

typedef struct
{
    bms_prot_cfg_t cfg;

    bms_prot_state_t state;
    uint32_t state_ms;      /* 处于当前状态的累计时间 */
    uint16_t faults;        /* 当前活跃故障位 */
    uint32_t trips;         /* 累计进入保护次数，用于观测 */
} bms_prot_t;

/* 默认参数：13 串 LFP、100Ah */
void bms_prot_default_cfg(bms_prot_cfg_t *cfg);

void bms_prot_init(bms_prot_t *p, const bms_prot_cfg_t *cfg);

/*
 * 推进一步，返回新状态。
 * raw 为未滤波采样，filt 为滤波后采样（都要提供，见头文件开头第 3 点）。
 */
bms_prot_state_t bms_prot_step(bms_prot_t *p, const bms_sample_t *raw,
                               const bms_sample_t *filt, uint32_t dt_ms);

bms_prot_state_t bms_prot_state(const bms_prot_t *p);
uint16_t bms_prot_faults(const bms_prot_t *p);

/* 只有 PROTECT 与 RECOVER 期间断开主回路 */
bms_relay_t bms_prot_relay(const bms_prot_t *p);

/* 故障位对应的名字，用于日志与 UDS 上报 */
const char *bms_prot_fault_name(uint16_t bit);

#endif
