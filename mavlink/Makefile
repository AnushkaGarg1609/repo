# MAVLink v2 stack --- PC build for unit testing
#
# The stack itself has exactly one cross-module dependency: core/fixed_point.h
# for fx_t and the Q16.16 primitives. The bridge additionally uses the core's
# mission_t and flight_mode_t, which is the whole point of it.
#
#   make            build and run the unit tests
#   make test       same
#   make clean      remove build artefacts

CC      ?= gcc
CORE    := ../core
CFLAGS  := -std=c99 -Wall -Wextra -O2 -I. -I$(CORE)
LDFLAGS := -lm

MAV_SRC  := mavlink_encode.c mavlink_decode.c mavlink_bridge.c
CORE_SRC := $(CORE)/fixed_point.c $(CORE)/mission.c $(CORE)/flight_modes.c
TEST_SRC := test_mavlink.c

TEST_BIN := test_mavlink

.PHONY: all test clean

all: test

$(TEST_BIN): $(MAV_SRC) $(CORE_SRC) $(TEST_SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(TEST_BIN) $(TEST_BIN).exe *.o
