/*
 * bms_types.h —— BMS 层共用的数据类型与默认参数
 *
 * 全部用整数定点，不用浮点：
 * 目标芯片可能是没有 FPU 的 Cortex-M0/M3，浮点运算靠软件模拟，
 * 一次除法几百个周期 —— 在 10ms 的采样周期里这是浪费。
 * 电压用 mV、电流用 mA、温度用 0.1℃、SOC 用 0.1%。
 */
#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>

/* 默认电池配置：13 串、100Ah（48V 级储能包） */
#define BMS_DEFAULT_CELLS        13
#define BMS_DEFAULT_Q_MAH        100000u   /* 100 Ah */

/* 采样周期：10 ms（100 Hz）。与 CAN 上报解耦，上报是 50~100ms 一次 */
#define BMS_SAMPLE_PERIOD_MS     10u

/*
 * 一次采样。
 *
 * 报文的电流约定沿用 can-node 的 DBC：充电为正、放电为负。
 * 这个约定要写死在结构体注释里 —— 保护逻辑和 SOC 积分都依赖它，
 * 一旦某处反了，表现为「充电时 SOC 下降」，排查起来非常费时。
 */
typedef struct
{
    uint32_t pack_mv;      /* 包端电压，mV */
    int32_t  current_ma;   /* 充电为正、放电为负，mA */
    int16_t  temp_dc;      /* 最高单体温度，0.1℃ */
} bms_sample_t;

/* 保护动作：上层据此控制主回路 */
typedef enum
{
    BMS_RELAY_CLOSED = 0,  /* 闭合，允许充放电 */
    BMS_RELAY_OPEN   = 1,  /* 断开 */
} bms_relay_t;

#endif
