/*
 * bms_node.c —— BMS 节点主程序
 *
 * 采样源说明（这一版是演示用的，不是最终形态）：
 *   用一段场景脚本按时间给出「包压 / 电流 / 温度」，走完整生命周期：
 *   静置 -> 充电 -> 过压告警 -> 过压保护 -> 回落恢复 -> 正常。
 *   真实硬件上这里换成 ADC + 定时器触发采样，其余代码一行不改 ——
 *   滤波、SOC、保护、UDS、CAN 上报都不依赖采样从哪来。
 *
 * 主循环分工：
 *   10ms   采样 -> 滤波 -> SOC 积分 -> 保护判断
 *   100ms  CAN 上报（0x200 PackData）
 *   随时   UDS 请求处理（ISO-TP 收上来，响应发回去）
 */
/* c99 下 clock_gettime / usleep 需要显式打开扩展声明（与 can-node 一致） */
#define _DEFAULT_SOURCE

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "bms_filter.h"
#include "bms_protect.h"
#include "bms_soc.h"
#include "bms_types.h"

#include "can_driver.h"
#include "can_driver_socketcan.h"
#include "can_matrix.h"
#include "isotp.h"
#include "uds.h"

#define ID_PACK_DATA   0x200
#define ID_REQUEST     0x7E0
#define ID_RESPONSE    0x7E8

#define TX_PERIOD_MS   100

/* ---------- 全局运行状态（UDS 回调需要读取） ---------- */

static volatile int g_stop;

static bms_prot_t   g_prot;
static bms_soc_t    g_soc;
static bms_sample_t g_raw;      /* 未滤波 */
static bms_sample_t g_filt;     /* 滤波后，用于上报与展示 */

static bms_chan_t g_ch_mv, g_ch_ma, g_ch_tc;

static const uint8_t g_vin[17] = "LSVBM1234567890A";

/* ---------- 采样场景 ---------- */

typedef struct
{
    uint32_t hold_ms;
    uint32_t mv;
    int32_t  ma;
    int16_t  dc;
} scene_step_t;

/*
 * 13 串 LFP。阈值见 bms_prot_default_cfg：
 *   OV_WARN 46150 / OV_PROT 47450
 *   UV_WARN 39000 / UV_PROT 36400
 * 温度 45/55℃ 告警/保护
 */
static const scene_step_t g_scene[] =
{
    { 4000, 42900,      0, 250},   /* 静置：OCV 修正把 SOC 拉回真值 */
    { 4000, 43200,  15000, 260},   /* 充电 15A */
    { 3000, 46600,  20000, 310},   /* 过压告警 */
    { 2000, 48000,  20000, 330},   /* 过压保护 */
    /*
     * 回落阶段的电压要选在表的有效范围中部。
     * 之前用 44500mV（单节 3.42V）—— 那对 LFP 是「刚充满」的电压，
     * 静置后 OCV 修正会把 SOC 正确但突兀地拉到接近满值。
     * 不是算法错，是场景不符合物理：过压保护后电压不会停在满电附近。
     */
    { 5000, 43200,      0, 280},   /* 回落，恢复确认中 */
    { 9000, 42900,      0, 260},   /* 回正常 */
};

#define SCENE_STEPS (sizeof(g_scene) / sizeof(g_scene[0]))

static uint64_t now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000);
}

static bms_sample_t scene_sample(uint64_t elapsed_ms)
{
    uint64_t acc = 0;

    for (size_t i = 0; i < SCENE_STEPS; i++)
    {
        acc += g_scene[i].hold_ms;
        if (elapsed_ms < acc)
        {
            bms_sample_t s;

            s.pack_mv = g_scene[i].mv;
            s.current_ma = g_scene[i].ma;
            s.temp_dc = g_scene[i].dc;
            return s;
        }
    }

    /* 脚本跑完后保持最后一帧 */
    {
        bms_sample_t s;
        const scene_step_t *last = &g_scene[SCENE_STEPS - 1];

        s.pack_mv = last->mv;
        s.current_ma = last->ma;
        s.temp_dc = last->dc;
        return s;
    }
}

/* ---------- UDS DID 回调 ---------- */

static int read_cell_mv(uint8_t *out, uint8_t *len)
{
    out[0] = (uint8_t)(g_filt.pack_mv >> 8);
    out[1] = (uint8_t)(g_filt.pack_mv & 0xFF);
    *len = 2;
    return 0;
}

static int read_current(uint8_t *out, uint8_t *len)
{
    /* int32，大端：ISO 14229 规定 UDS 报文用大端 */
    int32_t v = g_filt.current_ma;

    out[0] = (uint8_t)((v >> 24) & 0xFF);
    out[1] = (uint8_t)((v >> 16) & 0xFF);
    out[2] = (uint8_t)((v >> 8) & 0xFF);
    out[3] = (uint8_t)(v & 0xFF);
    *len = 4;
    return 0;
}

static int read_temp(uint8_t *out, uint8_t *len)
{
    int16_t v = g_filt.temp_dc;

    out[0] = (uint8_t)((v >> 8) & 0xFF);
    out[1] = (uint8_t)(v & 0xFF);
    *len = 2;
    return 0;
}

static int read_soc(uint8_t *out, uint8_t *len)
{
    int32_t v = bms_soc_get(&g_soc);

    out[0] = (uint8_t)((v >> 8) & 0xFF);
    out[1] = (uint8_t)(v & 0xFF);
    *len = 2;
    return 0;
}

static int read_prot_state(uint8_t *out, uint8_t *len)
{
    out[0] = (uint8_t)bms_prot_state(&g_prot);
    *len = 1;
    return 0;
}

static int read_faults(uint8_t *out, uint8_t *len)
{
    uint16_t f = bms_prot_faults(&g_prot);

    out[0] = (uint8_t)(f >> 8);
    out[1] = (uint8_t)(f & 0xFF);
    *len = 2;
    return 0;
}

/*
 * 保护阈值可写。演示「0x2E 写参数需要先解锁」这条链路，
 * 真实项目里这个值要落 Flash，本版本只改内存（掉电丢失）。
 */
static uint32_t g_ov_prot_mv = 47450;

static int read_ov_threshold(uint8_t *out, uint8_t *len)
{
    out[0] = (uint8_t)(g_ov_prot_mv >> 24);
    out[1] = (uint8_t)((g_ov_prot_mv >> 16) & 0xFF);
    out[2] = (uint8_t)((g_ov_prot_mv >> 8) & 0xFF);
    out[3] = (uint8_t)(g_ov_prot_mv & 0xFF);
    *len = 4;
    return 0;
}

static int write_ov_threshold(const uint8_t *in, uint8_t len)
{
    uint32_t v;

    if (len != 4)
        return -1;

    v = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
        ((uint32_t)in[2] << 8) | (uint32_t)in[3];

    /* 拒绝明显不合理的值：写坏了阈值比不写更危险 */
    if (v < 40000 || v > 50000)
        return -1;

    g_ov_prot_mv = v;
    g_prot.cfg.ov_prot_mv = v;

    printf("[bms] OV protection threshold updated to %u mV\n", (unsigned)v);
    fflush(stdout);
    return 0;
}

static const uds_did_entry_t g_dids[] =
{
    {0xF190, 17, g_vin, 0, 0, UDS_SESSION_DEFAULT, 0},
    {0x0101, 0, 0, read_cell_mv,     0, UDS_SESSION_DEFAULT, 0},
    {0x0102, 0, 0, read_current,     0, UDS_SESSION_DEFAULT, 0},
    {0x0103, 0, 0, read_temp,        0, UDS_SESSION_DEFAULT, 0},
    {0x0104, 0, 0, read_soc,         0, UDS_SESSION_DEFAULT, 0},
    {0x0105, 0, 0, read_prot_state,  0, UDS_SESSION_DEFAULT, 0},
    {0x0106, 0, 0, read_faults,      0, UDS_SESSION_DEFAULT, 0},
    {0x0201, 0, 0, read_ov_threshold, write_ov_threshold,
     UDS_SESSION_EXTENDED, 1},
};

/* ---------- 上报 ---------- */

static void send_frame(can_driver_t *can, const uint8_t *data, uint8_t dlc)
{
    can_frame_t f;

    memset(&f, 0, sizeof(f));
    f.id = ID_PACK_DATA;
    f.dlc = dlc;
    memcpy(f.data, data, dlc);

    can_driver_send(can, &f);
}

/* ---------- 主循环 ---------- */

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

int main(int argc, char **argv)
{
    can_socketcan_t sk;
    can_driver_t can;
    isotp_t tp;
    isotp_config_t icfg = {ID_RESPONSE, ID_REQUEST, 8, 10, 0};
    uds_server_t srv;
    uds_config_t ucfg;
    bms_soc_cfg_t scfg;
    uint8_t req[64], resp[64];
    const char *ifname = "vcan0";
    int seconds = 30;
    uint64_t t0, last_sample = 0, last_tx = 0, last_log = 0;
    int last_state = -1;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc)
            seconds = atoi(argv[++i]);
        else if (strcmp(argv[i], "--if") == 0 && i + 1 < argc)
            ifname = argv[++i];
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* ---- 通道与协议栈 ---- */
    memset(&sk, 0, sizeof(sk));
    snprintf(sk.ifname, sizeof(sk.ifname), "%s", ifname);
    sk.rx_filter_id = ID_REQUEST;
    sk.fd = -1;

    if (can_driver_init(&can, can_socketcan_ops(), &sk, 500000) != CAN_OK)
    {
        printf("cannot open %s (need: ip link set up %s)\n", ifname, ifname);
        return 1;
    }

    isotp_init(&tp, &icfg);

    memset(&ucfg, 0, sizeof(ucfg));
    ucfg.dids = g_dids;
    ucfg.did_count = (uint16_t)(sizeof(g_dids) / sizeof(g_dids[0]));
    uds_init(&srv, &ucfg);

    /* ---- BMS 模块 ---- */
    bms_chan_init(&g_ch_mv);
    bms_chan_init(&g_ch_ma);
    bms_chan_init(&g_ch_tc);

    bms_prot_init(&g_prot, 0);
    g_ov_prot_mv = g_prot.cfg.ov_prot_mv;

    bms_soc_default_cfg(&scfg);
    /*
     * 演示用缩短了静置判定：真实 5 分钟在 30 秒的演示里等不到。
     * 这三个值是**演示参数**，不是工程参数，真实配置见 bms_soc_default_cfg。
     */
    scfg.rest_time_ms = 2000;
    scfg.min_interval_ms = 8000;

    /* 故意从 80% 开始，让 OCV 修正的效果看得见 */
    bms_soc_init(&g_soc, &scfg, BMS_DEFAULT_Q_MAH, BMS_DEFAULT_CELLS, 800);

    t0 = now_ms();
    last_sample = t0;
    last_tx = t0;
    last_log = t0;

    printf("[bms] listening on %s, req=0x%03X rsp=0x%03X, %d s\n",
           ifname, ID_REQUEST, ID_RESPONSE, seconds);
    printf("[bms] initial soc=%d permille (will be corrected by OCV)\n",
           bms_soc_get(&g_soc));
    fflush(stdout);

    while (!g_stop)
    {
        uint64_t now = now_ms();
        uint64_t elapsed = now - t0;

        if (elapsed / 1000 >= (uint64_t)seconds && seconds > 0)
            break;

        /* ---- 10ms 采样与控制 ---- */
        if (now - last_sample >= BMS_SAMPLE_PERIOD_MS)
        {
            uint32_t dt = (uint32_t)(now - last_sample);

            g_raw = scene_sample(elapsed);

            /* 三路独立滤波：电压慢变、电流快变、温度几乎不变 */
            g_filt.pack_mv = (uint32_t)bms_chan_push(&g_ch_mv, (int32_t)g_raw.pack_mv);
            g_filt.current_ma = bms_chan_push(&g_ch_ma, g_raw.current_ma);
            g_filt.temp_dc = (int16_t)bms_chan_push(&g_ch_tc, g_raw.temp_dc);

            /* SOC 用原始电流积分：滤波会引入滞后，积分不需要平滑 */
            bms_soc_step(&g_soc, g_raw.current_ma, g_raw.pack_mv, g_raw.temp_dc, dt);

            bms_prot_step(&g_prot, &g_raw, &g_filt, dt);

            last_sample = now;
        }

        /* ---- 100ms 上报 ---- */
        if (now - last_tx >= TX_PERIOD_MS)
        {
            struct can_matrix_pack_data_t msg;
            uint8_t buf[8];

            memset(&msg, 0, sizeof(msg));
            msg.pack_voltage = (uint16_t)(g_filt.pack_mv / 10);
            msg.pack_current = (int16_t)(g_filt.current_ma / 10);
            msg.soc = (uint8_t)(bms_soc_get(&g_soc) / 10);
            msg.temp_max = (int8_t)((g_filt.temp_dc / 10) + 40);
            msg.prot_state = (uint8_t)bms_prot_state(&g_prot);
            msg.fault_code = (uint8_t)(bms_prot_faults(&g_prot) & 0xFF);

            if (can_matrix_pack_data_pack(buf, &msg, sizeof(buf)) > 0)
                send_frame(&can, buf, 8);

            last_tx = now;
        }

        /* ---- 状态变化时打印（这是最有信息量的日志） ---- */
        if (last_state != (int)g_prot.state)
        {
            printf("[bms] t=%5llums  state %d -> %d  faults=0x%04X relay=%s\n",
                   (unsigned long long)elapsed, last_state, g_prot.state,
                   g_prot.faults,
                   bms_prot_relay(&g_prot) == BMS_RELAY_OPEN ? "OPEN" : "CLOSED");
            fflush(stdout);
            last_state = g_prot.state;
        }

        /* ---- 1s 打一次概要 ---- */
        if (now - last_log >= 1000)
        {
            printf("[bms] t=%5llums  %5umV %6dmA %3d.%dC  soc=%3d  prot=%d faults=0x%04X corr=%u\n",
                   (unsigned long long)elapsed,
                   g_filt.pack_mv, g_filt.current_ma,
                   g_filt.temp_dc / 10, (g_filt.temp_dc < 0 ? -g_filt.temp_dc : g_filt.temp_dc) % 10,
                   bms_soc_get(&g_soc), g_prot.state, g_prot.faults,
                   (unsigned)g_soc.corrections);
            fflush(stdout);
            last_log = now;
        }

        /* ---- UDS ---- */
        isotp_poll(&tp, &can);

        {
            uint16_t len = 0;

            if (isotp_recv(&tp, req, &len) == ISOTP_OK)
            {
                int rlen = uds_handle_request(&srv, req, len, resp, sizeof(resp));

                if (rlen > 0)
                {
                    isotp_send(&tp, &can, resp, (uint16_t)rlen);

                    /* 把响应推完再继续，否则下一次请求可能打断传输 */
                    for (int i = 0; i < 50 && isotp_tx_busy(&tp); i++)
                    {
                        isotp_poll(&tp, &can);
                        usleep(500);
                    }
                }
            }
        }

        /* S3 会话计时需要推进 */
        {
            static uint64_t last_tick = 0;
            uint64_t t = now_ms();

            if (last_tick == 0)
                last_tick = t;

            uds_tick(&srv, (uint32_t)(t - last_tick));
            last_tick = t;
        }

        usleep(2000);
    }

    printf("\n[bms] stopping\n");
    printf("[bms] final: soc=%d permille, state=%d, faults=0x%04X, trips=%u, corrections=%u\n",
           bms_soc_get(&g_soc), g_prot.state, g_prot.faults,
           (unsigned)g_prot.trips, (unsigned)g_soc.corrections);

    can_driver_deinit(&can);
    return 0;
}
