/*
 * bms_dtc.h —— 保护故障位到诊断故障码的映射
 *
 * 为什么单独一层，而不是写在 app 的 main 里：
 *   保护的六个位（见 bms_protect.h）和 DTC 的六个码是两套编号，靠一张表
 *   对上。这张表要能被单元测试直接验证 —— 漏一条映射的后果是「保护动作了、
 *   继电器断了，但售后读不到码」，而且演示时完全看不出来（灯亮着、车停了，
 *   一切「正常」）。
 *
 * 只映射保护级（BMS_F_*_PROT），告警级不进存储器：
 *   告警会自己回落，记成 DTC 只会让真正的故障淹没在一堆历史码里。
 *   这和 ISO 14229 对 pending/confirmed 的设计意图一致 —— 存储器是给
 *   「需要事后追溯的故障」用的，不是状态指示灯。
 */
#ifndef BMS_DTC_H
#define BMS_DTC_H

#include <stdint.h>

#include "dtc.h"

/*
 * 取定义表与条数。out 可为 NULL，此时只返回条数。
 * 返回的指针指向静态常量，生命周期是整个进程。
 */
uint16_t bms_dtc_defs(const dtc_def_t **out);

/*
 * 填默认配置：确认需 2 个循环、老化 40 个循环。
 * 这两个值是主机厂常用值，改了要单独说明理由。
 */
void bms_dtc_default_config(dtc_config_t *cfg);

/*
 * 把当前保护故障位同步进存储器。
 *
 * **每一轮采样都要调**，包括没有故障的时候 —— dtc_report 靠「通过」的
 * 上报来推进老化计数，只在故障时调的结果是故障永远清不掉。
 *
 * 返回本轮上报的码数量（等于定义表的条数），负值表示存储器未初始化。
 */
int bms_dtc_sync(dtc_store_t *st, uint16_t faults, uint32_t now_ms);

#endif
