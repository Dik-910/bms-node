CC      ?= gcc
CFLAGS  += -std=c99 -Wall -Wextra -I. -Isrc/bms -Isrc/can -Isrc/can/generated -Isrc/net -Isrc/uds -Isrc/app

.PHONY: all test clean

all: test

BMS_SRC := src/bms/bms_filter.c src/bms/bms_soc.c src/bms/bms_protect.c

test: test/bms_filter_test test/bms_soc_test test/bms_protect_test
	@for t in $^; do echo "=== $$t ==="; ./$$t || exit 1; done

test/bms_filter_test: test/bms_filter_test.c src/bms/bms_filter.c
	$(CC) $(CFLAGS) $^ -o $@

test/bms_soc_test: test/bms_soc_test.c src/bms/bms_soc.c
	$(CC) $(CFLAGS) $^ -o $@

test/bms_protect_test: test/bms_protect_test.c src/bms/bms_protect.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	$(RM) test/bms_filter_test test/bms_soc_test test/bms_protect_test

# ---- 节点可执行文件 ----
STACK_SRC := src/can/can_driver.c src/can/can_driver_socketcan.c \
             src/can/generated/can_matrix.c src/net/isotp.c src/uds/uds.c

node: src/app/bms_node.c src/bms/bms_filter.c src/bms/bms_soc.c src/bms/bms_protect.c $(STACK_SRC)
	$(CC) $(CFLAGS) $^ -o $@

all: node test
