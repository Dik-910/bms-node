# bms-node

储能电池管理节点固件，基于 STM32 实现数据采样、保护控制与荷电状态估算，
复用 [can-node](https://github.com/Dik-910/can-node) 的 CAN 与 UDS 栈。

## 功能

### 采样

- 多通道 ADC 采样：单体电压、母线电流、温度
- 滑动平均 + 一阶低通滤波，抑制采样噪声
- 采样周期与 CAN 上报周期解耦，上报按降采样后的值

### 保护状态机

四级状态，带迟滞带防抖动：

```
正常 ──越过告警阈值──→ 告警 ──越过保护阈值──→ 保护
  ↑                      │                      │
  └──回落到恢复阈值──────┴──────────────────────┘
```

| 状态 | 动作 | 恢复条件 |
|---|---|---|
| 正常 | 正常充放电 | — |
| 告警 | 上报故障码，限制电流 | 低于告警阈值 − 迟滞量 |
| 保护 | 断开主回路 | 低于保护阈值 − 迟滞量 |
| 恢复 | 延时确认后回正常 | 稳定运行 T 秒 |

迟滞量按电池规格书的允许波动范围确定，避免阈值附近反复跳变。

### 荷电状态估算

- **安时积分**：`SOC = SOC₀ − (∫I dt) / Q_rated`，实时跟踪，不依赖静置
- **OCV 修正**：静置足够长后按开路电压查表校正积分累积误差
- 积分误差随工况累积，修正时机由静置时长与电流阈值共同判定

### 故障码

- 故障记录含类型、触发时刻、当时采样值
- 按优先级上报当前最高级别故障
- 支持通过 UDS 读取历史故障与清除

### 诊断接口

通过 UDS 暴露为被诊断对象：

| 服务 | 用途 |
|---|---|
| `0x22` | 读单体电压、电流、温度、SOC、故障码 |
| `0x2E` | 写保护阈值与标定参数 |
| `0x31` | 触发自检例程 |

## 架构

```
应用层      bms/     保护状态机、SOC 估算、故障管理
            uds/     诊断服务（复用 can-node）
中间层      net/     ISO-TP（复用 can-node）
驱动层      can/     CAN 驱动（复用 can-node）
            adc/     ADC 采样与滤波
板级层      bsp/     时钟、GPIO、采样引脚
```

`src/can/` 与 `src/net/` 从 can-node 同步，**保持同源**，不在本仓库单独演进。

## 目录

```
src/bsp/     板级支持
src/can/     CAN 驱动（同步自 can-node）
src/net/     ISO-TP（同步自 can-node）
src/bms/     采样、保护状态机、SOC、故障码
src/uds/     诊断服务
test/        单元测试与 PC 侧桩
docs/        设计文档
```

## 通信协议

报文定义统一维护在 [can-node/docs/can_matrix.dbc](https://github.com/Dik-910/can-node/blob/main/docs/can_matrix.dbc)，
说明见 [protocol.md](https://github.com/Dik-910/can-node/blob/main/docs/protocol.md)。

本节点发送 `0x200`（采样数据，周期 50 ms）与 `0x280`（故障事件，状态迁移时触发）。
DBC 为唯一权威来源，本地不单独维护副本。

## 构建

```bash
make            # 生成 build/bms-node.elf 与 .bin
make test       # PC 侧单元测试，不依赖硬件
make flash
```

## 版本计划

| 版本 | 内容 |
|---|---|
| v0.1 | ADC 采样与滤波，CAN 上报 `0x200` |
| v0.2 | 四级保护状态机、故障码记录 |
| v0.3 | 安时积分 + OCV 修正的 SOC 估算 |
| v0.4 | UDS 诊断对接（`0x22` / `0x2E` / `0x31`） |
| v1.0 | 保护响应时间与采样精度达标，协议冻结 |

## 相关仓库

- [can-node](https://github.com/Dik-910/can-node) —— 车载节点固件，提供 CAN / ISO-TP / UDS 栈
- [smartgateway](https://github.com/Dik-910/smartgateway) —— Linux 网关端

## 许可证

木兰宽松许可证，第 2 版。见 [LICENSE](LICENSE)。
