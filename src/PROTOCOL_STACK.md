# 协议栈来源

`src/can/`、`src/net/`、`src/uds/`、`src/app/node_health.*` 来自
[can-node](https://github.com/Dik-910/can-node) 的 **8b329bb**，
未做修改，便于与本仓库的 BMS 业务层解耦。

## 为什么不抽成公共库

两种做法都可以，这里选了复制：

- **复制**：两个仓库各自能独立构建、跑 CI、单独发给别人看；
  代价是改了协议栈要同步两处。
- **submodule / 独立包**：不重复，但克隆要 `--recursive`，
  CI 和"别人拿到仓库能不能直接编译"都多一层依赖。

这个项目更看重"单仓库可独立构建"，所以复制，并在
`tools/sync_stack.sh` 里写明同步方式。

## 同步

```bash
cd ~/work
bash bms-node/tools/sync_stack.sh
```

脚本会从 can-node 重新拷贝这几个文件，并更新本文的 commit 号。
同步后必须重跑 `make test` —— 协议栈的行为属于 BMS 节点的地基，
升级它等于换了地基。
