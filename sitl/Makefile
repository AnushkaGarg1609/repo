# Software-in-the-loop harness
#
#   make          build sitl_bridge
#   make run      build and run against 127.0.0.1:14550
#   make clean
#
# The firmware sources (core/, mavlink/) are compiled unmodified. Only
# vehicle_sim.c and sitl_link.c exist solely for the simulation.
#
#   make compat   also build sitl_bridge_ardupilot, which impersonates
#                 ArduCopter so Mission Planner offers its full copter UI
#                 (Takeoff button, mode dropdown, home controls). Enabled by
#                 -DMAV_ARDUPILOT_COMPAT; the default build stays honest.

CC      ?= gcc
CORE    := ../core
MAVLINK := ../mavlink

CFLAGS  := -std=c99 -Wall -Wextra -O2 -I. -I$(CORE) -I$(MAVLINK)
LDFLAGS := -lm

# Detect Windows: Winsock needs an explicit library
ifeq ($(OS),Windows_NT)
  LDFLAGS += -lws2_32
  BIN         := sitl_bridge.exe
  COMPAT_BIN  := sitl_bridge_ardupilot.exe
else
  BIN         := sitl_bridge
  COMPAT_BIN  := sitl_bridge_ardupilot
endif

FIRMWARE_SRC := \
	$(CORE)/fixed_point.c \
	$(CORE)/rotation.c \
	$(CORE)/sensor_fusion.c \
	$(CORE)/control.c \
	$(CORE)/flight_modes.c \
	$(CORE)/arming.c \
	$(CORE)/mission.c \
	$(MAVLINK)/mavlink_encode.c \
	$(MAVLINK)/mavlink_decode.c \
	$(MAVLINK)/mavlink_bridge.c

SIM_SRC := $(CORE)/vehicle_sim.c sitl_link.c sitl_bridge.c

.PHONY: all compat run clean

all: $(BIN)

$(BIN): $(FIRMWARE_SRC) $(SIM_SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# ArduCopter-impersonating build. Same sources, one preprocessor flag.
compat: $(COMPAT_BIN)

$(COMPAT_BIN): $(FIRMWARE_SRC) $(SIM_SRC)
	$(CC) $(CFLAGS) -DMAV_ARDUPILOT_COMPAT -o $@ $^ $(LDFLAGS)

run: $(BIN)
	./$(BIN)

clean:
	rm -f sitl_bridge sitl_bridge.exe \
	      sitl_bridge_ardupilot sitl_bridge_ardupilot.exe *.o
