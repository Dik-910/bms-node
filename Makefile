CC      ?= gcc
CFLAGS  += -std=c99 -Wall -Wextra -I. -Isrc/bms -Isrc/can -Isrc/can/generated -Isrc/net -Isrc/uds -Isrc/app

.PHONY: all test clean

all: test

BMS_SRC := src/bms/bms_filter.c src/bms/bms_soc.c src/bms/bms_protect.c

# test 也依赖 node：协议栈编译不过时必须在这里就失败，
# 否则单测只测 bms 三个模块，协议栈的编译错误会一路漏到 CI 的 make node。
test: node test/bms_filter_test test/bms_soc_test test/bms_protect_test test/bms_dtc_test
	@for t in test/bms_filter_test test/bms_soc_test test/bms_protect_test test/bms_dtc_test; do echo "=== $$t ==="; ./$$t || exit 1; done

test/bms_filter_test: test/bms_filter_test.c src/bms/bms_filter.c
	$(CC) $(CFLAGS) $^ -o $@

test/bms_soc_test: test/bms_soc_test.c src/bms/bms_soc.c
	$(CC) $(CFLAGS) $^ -o $@

test/bms_protect_test: test/bms_protect_test.c src/bms/bms_protect.c
	$(CC) $(CFLAGS) $^ -o $@

# DTC 映射测试要连协议栈的 dtc.c 一起编 —— 它验证的正是「这张表和存储器对得上」
test/bms_dtc_test: test/bms_dtc_test.c src/bms/bms_dtc.c src/uds/dtc.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	$(RM) test/bms_filter_test test/bms_soc_test test/bms_protect_test test/bms_dtc_test

# ---- 节点可执行文件 ----
STACK_SRC := src/can/can_driver.c src/can/can_driver_socketcan.c \
             src/can/generated/can_matrix.c src/net/isotp.c src/uds/uds.c \
             src/uds/dtc.c

node: src/app/bms_node.c src/bms/bms_filter.c src/bms/bms_soc.c src/bms/bms_protect.c \
      src/bms/bms_dtc.c $(STACK_SRC)
	$(CC) $(CFLAGS) $^ -o $@

all: node test
