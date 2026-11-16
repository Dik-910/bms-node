#!/usr/bin/env bash
# 从 can-node 同步协议栈源码。改协议栈时用，不要手改 bms-node 里的副本。
set -euo pipefail

CAN=${1:-/home/dikl/work/can-node}
DST=$(cd "$(dirname "$0")/.." && pwd)

[ -d "$CAN/src" ] || { echo "can-node not found at $CAN"; exit 1; }

cp "$CAN/src/can/can_driver.h" "$CAN/src/can/can_driver.c" \
   "$CAN/src/can/can_driver_stub.c" "$CAN/src/can/can_driver_socketcan.c" "$DST/src/can/"
cp "$CAN/src/net/isotp.h" "$CAN/src/net/isotp.c" "$DST/src/net/"
cp "$CAN/src/uds/uds.h" "$CAN/src/uds/uds.c" "$DST/src/uds/"
cp "$CAN/src/app/node_health.h" "$CAN/src/app/node_health.c" "$DST/src/app/"

SHA=$(git -C "$CAN" rev-parse --short HEAD)
sed -i "s/\*\*[0-9a-f]\{7,\}\*\*/**$SHA**/" "$DST/src/PROTOCOL_STACK.md"

echo "synced protocol stack from can-node@$SHA"
echo "now run: make test"
