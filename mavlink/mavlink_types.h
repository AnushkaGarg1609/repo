/**
 * mavlink_types.h
 *
 * MAVLink v2 Wire Protocol --- Types, Constants, Packed Payload Structs
 *
 * Section: MAVLink Ground Control Protocol Engine
 * Target:  VEGA THEJAS32 RV32IM (no FPU, no heap)
 *
 * This file is the ONLY dependency for the entire MAVLink stack.
 * No stdlib, no malloc, no floating-point arithmetic --- just fixed-width
 * types, compile-time structs, and integer-only IEEE-754 conversion.
 *
 * Reference: MAVLink v2 specification (mavlink.io), common.xml
 *   Frame format:
 *     [STX][LEN][INCOMPAT][COMPAT][SEQ][SYS][COMP][MSGID:3][PAYLOAD][CRC:2]
 *
 * ---------------------------------------------------------------------------
 * WIRE LAYOUT RULE (MAVLink v2 field reordering)
 * ---------------------------------------------------------------------------
 * Payload fields are NOT in XML declaration order. They are sorted by field
 * size, largest first (8, then 4, then 2, then 1 byte), stable within each
 * size class. Every struct below is written in WIRE order, and each carries
 * a compile-time assertion on its size. Get this wrong and the GCS silently
 * shows garbage --- there is no error message.
 *
 * ---------------------------------------------------------------------------
 * PAYLOAD TRUNCATION
 * ---------------------------------------------------------------------------
 * MAVLink v2 senders drop trailing zero bytes from the payload and set LEN to
 * the shortened length; the CRC covers only the truncated bytes. Receivers
 * must zero-pad back to the full message length before interpreting the
 * struct. Both directions are handled in mavlink_encode.c / mavlink_decode.c.
 */

#ifndef MAVLINK_TYPES_H
#define MAVLINK_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* fx_t and the Q16.16 primitives come from the flight core. This is the only
 * cross-module dependency in the MAVLink stack, and it is a header-only one:
 * core/fixed_point.h itself includes nothing but <stdint.h>/<stdbool.h>.
 * Build with -I../core. */
#include "fixed_point.h"

/* Compile-time struct-size check (C99-compatible; no _Static_assert needed). */
#define MAV_STATIC_ASSERT(cond, tag) \
    typedef char mav_static_assert_##tag[(cond) ? 1 : -1]

/* ============================================================================
 * SECTION 1: Wire Frame Constants
 * ========================================================================= */

#define MAVLINK_V2_STX              0xFD    /* MAVLink v2 start-of-frame magic */
#define MAVLINK_V1_STX              0xFE    /* v1 magic --- recognised, skipped */
#define MAVLINK_MAX_PAYLOAD_LEN     255     /* Maximum payload bytes */
#define MAVLINK_NUM_HEADER_BYTES    10      /* STX+LEN+INCOMPAT+COMPAT+SEQ+SYS+COMP+MSGID(3) */
#define MAVLINK_NUM_CRC_BYTES       2       /* Appended after payload */
#define MAVLINK_SIGNATURE_LEN       13      /* Present only if INCOMPAT_FLAG_SIGNED */
#define MAVLINK_MAX_FRAME_LEN       (MAVLINK_NUM_HEADER_BYTES + MAVLINK_MAX_PAYLOAD_LEN + \
                                     MAVLINK_NUM_CRC_BYTES + MAVLINK_SIGNATURE_LEN)

#define MAVLINK_IFLAG_SIGNED        0x01    /* incompat_flags bit 0 */
#define MAVLINK_VERSION_FIELD       3       /* HEARTBEAT.mavlink_version for v2 */

/**
 * Longest payload in the dialect of Section 3 (STATUSTEXT, 51 bytes).
 *
 * The parser rejects a LEN field larger than this the moment it reads it,
 * rather than dutifully consuming up to 255 bytes it can never interpret.
 * This matters on a noisy radio link: a stray 0xFD in line noise is read as a
 * start byte, and whatever follows it is read as the length. Accepting a
 * bogus length of 250 means swallowing the next 250 bytes of the stream ---
 * several real frames, including possibly a pilot command --- before
 * resynchronising. Capping the window at the largest frame we could actually
 * act on bounds that loss to one frame's worth of bytes.
 *
 * The cost is that a genuine message outside our dialect that happens to be
 * longer than this resynchronises inside its payload instead of after it.
 * That message was going to be discarded either way, so the trade favours the
 * tighter bound. Raise this if a longer message is ever added to the dialect;
 * test_mavlink.c asserts the table never exceeds it.
 */
#define MAVLINK_MAX_DIALECT_PAYLOAD 51

/* ============================================================================
 * SECTION 2: System / Component IDs
 * ========================================================================= */

#define MAV_SYS_ID                  1       /* Our autopilot system ID */
#define MAV_COMP_ID_AUTOPILOT       1       /* MAV_COMP_ID_AUTOPILOT1 */
#define MAV_COMP_ID_MISSIONPLANNER  190     /* Typical GCS component ID */
#define MAV_COMP_ID_ALL             0       /* Broadcast component */

/* ============================================================================
 * SECTION 3: Message IDs
 * ========================================================================= */

#define MAVLINK_MSG_ID_HEARTBEAT             0
#define MAVLINK_MSG_ID_SYS_STATUS            1
#define MAVLINK_MSG_ID_SET_MODE             11
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ   20
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST   21
#define MAVLINK_MSG_ID_PARAM_VALUE          22
#define MAVLINK_MSG_ID_PARAM_SET            23
#define MAVLINK_MSG_ID_GPS_RAW_INT          24
#define MAVLINK_MSG_ID_ATTITUDE             30
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_ID_MISSION_CURRENT      42
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43
#define MAVLINK_MSG_ID_MISSION_COUNT        44
#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL    45
#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED 46
#define MAVLINK_MSG_ID_MISSION_ACK          47
#define MAVLINK_MSG_ID_MISSION_REQUEST_INT  51
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM  66
#define MAVLINK_MSG_ID_MISSION_ITEM_INT     73
#define MAVLINK_MSG_ID_VFR_HUD              74
#define MAVLINK_MSG_ID_COMMAND_LONG         76
#define MAVLINK_MSG_ID_COMMAND_ACK          77
#define MAVLINK_MSG_ID_STATUSTEXT          253

/* CRC_EXTRA seeds --- per-message constants from the MAVLink dialect XML.
 * These bind a message's field layout to its CRC: a GCS built against a
 * different definition of the same message ID fails the CRC rather than
 * silently misparsing. Values must match common.xml exactly. */
#define MAVLINK_CRC_EXTRA_HEARTBEAT             50
#define MAVLINK_CRC_EXTRA_SYS_STATUS           124
#define MAVLINK_CRC_EXTRA_SET_MODE              89
#define MAVLINK_CRC_EXTRA_PARAM_REQUEST_READ   214
#define MAVLINK_CRC_EXTRA_PARAM_REQUEST_LIST   159
#define MAVLINK_CRC_EXTRA_PARAM_VALUE          220
#define MAVLINK_CRC_EXTRA_PARAM_SET            168
#define MAVLINK_CRC_EXTRA_GPS_RAW_INT           24
#define MAVLINK_CRC_EXTRA_ATTITUDE              39
#define MAVLINK_CRC_EXTRA_GLOBAL_POSITION_INT  104
#define MAVLINK_CRC_EXTRA_MISSION_CURRENT       28
#define MAVLINK_CRC_EXTRA_MISSION_REQUEST_LIST 132
#define MAVLINK_CRC_EXTRA_MISSION_COUNT        221
#define MAVLINK_CRC_EXTRA_MISSION_CLEAR_ALL    232
#define MAVLINK_CRC_EXTRA_MISSION_ITEM_REACHED  11
#define MAVLINK_CRC_EXTRA_MISSION_ACK          153
#define MAVLINK_CRC_EXTRA_MISSION_REQUEST_INT  196
#define MAVLINK_CRC_EXTRA_REQUEST_DATA_STREAM  148
#define MAVLINK_CRC_EXTRA_MISSION_ITEM_INT      38
#define MAVLINK_CRC_EXTRA_VFR_HUD               20
#define MAVLINK_CRC_EXTRA_COMMAND_LONG         152
#define MAVLINK_CRC_EXTRA_COMMAND_ACK          143
#define MAVLINK_CRC_EXTRA_STATUSTEXT            83

/* ============================================================================
 * SECTION 4: MAVLink Enums
 * ========================================================================= */

typedef enum { MAV_TYPE_QUADROTOR = 2 } mav_type_t;

/* MAV_AUTOPILOT_GENERIC tells the GCS "custom_mode uses this firmware's own
 * numbering" --- Mission Planner then shows the raw number rather than
 * mislabelling it with ArduPilot's mode names.
 *
 * MAV_AUTOPILOT_ARDUPILOTMEGA is used ONLY when the firmware is built with
 * -DMAV_ARDUPILOT_COMPAT (see mav_encode_heartbeat). It is a deliberate lie
 * told to a ground station so it will offer its ArduCopter UI --- the Takeoff
 * button, the flight-mode dropdown, the home controls. The flight code is
 * unchanged; only the two advertised fields (autopilot id and custom_mode
 * numbering) differ. The honest board build never defines the macro. */
typedef enum {
    MAV_AUTOPILOT_GENERIC       = 0,
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3
} mav_autopilot_t;

typedef enum {
    MAV_MODE_FLAG_SAFETY_ARMED          = 0x80,
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED  = 0x40,
    MAV_MODE_FLAG_HIL_ENABLED           = 0x20,
    MAV_MODE_FLAG_STABILIZE_ENABLED     = 0x10,
    MAV_MODE_FLAG_GUIDED_ENABLED        = 0x08,
    MAV_MODE_FLAG_AUTO_ENABLED          = 0x04,
    MAV_MODE_FLAG_TEST_ENABLED          = 0x02,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED   = 0x01
} mav_mode_flag_t;

typedef enum {
    MAV_STATE_UNINIT      = 0,
    MAV_STATE_BOOT        = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY     = 3,
    MAV_STATE_ACTIVE      = 4,
    MAV_STATE_CRITICAL    = 5,
    MAV_STATE_EMERGENCY   = 6,
    MAV_STATE_POWEROFF    = 7
} mav_state_t;

typedef enum {
    MAV_RESULT_ACCEPTED             = 0,
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    MAV_RESULT_DENIED               = 2,
    MAV_RESULT_UNSUPPORTED          = 3,
    MAV_RESULT_FAILED               = 4
} mav_result_t;

typedef enum {
    MAV_MISSION_ACCEPTED            = 0,
    MAV_MISSION_ERROR               = 1,
    MAV_MISSION_UNSUPPORTED_FRAME   = 2,
    MAV_MISSION_UNSUPPORTED         = 3,
    MAV_MISSION_NO_SPACE            = 4,
    MAV_MISSION_INVALID             = 5,
    MAV_MISSION_INVALID_SEQUENCE    = 12,
    MAV_MISSION_DENIED              = 13,
    MAV_MISSION_OPERATION_CANCELLED = 14
} mav_mission_result_t;

typedef enum {
    MAV_CMD_NAV_WAYPOINT          = 16,
    MAV_CMD_NAV_LOITER_UNLIM      = 17,
    MAV_CMD_NAV_RETURN_TO_LAUNCH  = 20,
    MAV_CMD_NAV_LAND              = 21,
    MAV_CMD_NAV_TAKEOFF           = 22,
    MAV_CMD_DO_SET_MODE           = 176,
    MAV_CMD_DO_SET_HOME           = 179,
    MAV_CMD_PREFLIGHT_CALIBRATION = 241,
    MAV_CMD_MISSION_START         = 300,
    MAV_CMD_COMPONENT_ARM_DISARM  = 400,
    MAV_CMD_SET_MESSAGE_INTERVAL  = 511,
    MAV_CMD_REQUEST_MESSAGE       = 512
} mav_cmd_id_t;

typedef enum {
    MAV_FRAME_GLOBAL                  = 0,
    MAV_FRAME_LOCAL_NED               = 1,
    MAV_FRAME_MISSION                 = 2,
    MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3,
    MAV_FRAME_GLOBAL_INT              = 5,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
} mav_frame_t;

typedef enum {
    MAV_SEVERITY_EMERGENCY = 0,
    MAV_SEVERITY_ALERT     = 1,
    MAV_SEVERITY_CRITICAL  = 2,
    MAV_SEVERITY_ERROR     = 3,
    MAV_SEVERITY_WARNING   = 4,
    MAV_SEVERITY_NOTICE    = 5,
    MAV_SEVERITY_INFO      = 6,
    MAV_SEVERITY_DEBUG     = 7
} mav_severity_t;

/* SYS_STATUS sensor bitmask (MAV_SYS_STATUS_SENSOR) --- the subset this
 * architecture actually has hardware for. */
typedef enum {
    MAV_SYS_STATUS_SENSOR_3D_GYRO                = 0x00000001,
    MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 0x00000002,
    MAV_SYS_STATUS_SENSOR_3D_MAG                 = 0x00000004,
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 0x00000008,
    MAV_SYS_STATUS_SENSOR_GPS                    = 0x00000020,
    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 0x00000400,
    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 0x00000800,
    MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 0x00001000,
    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 0x00002000,
    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 0x00004000,
    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 0x00008000,
    MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 0x00010000
} mav_sys_status_sensor_t;

typedef enum {
    MAV_PARAM_TYPE_UINT8  = 1,
    MAV_PARAM_TYPE_INT8   = 2,
    MAV_PARAM_TYPE_UINT16 = 3,
    MAV_PARAM_TYPE_INT16  = 4,
    MAV_PARAM_TYPE_UINT32 = 5,
    MAV_PARAM_TYPE_INT32  = 6,
    MAV_PARAM_TYPE_REAL32 = 9
} mav_param_type_t;

/* ============================================================================
 * SECTION 5: Packed Payload Structs --- IN WIRE ORDER
 *
 * `float` never appears here. Every IEEE-754 field is declared as uint32_t
 * holding the raw bit pattern, produced/consumed by fx_to_ieee754() and
 * ieee754_to_fx() in Section 9, so the RV32IM core never links a soft-float
 * routine.
 * ========================================================================= */

#pragma pack(push, 1)

/* HEARTBEAT (ID 0) --- 9 bytes */
typedef struct {
    uint32_t custom_mode;      /* Firmware-specific mode (our flight_mode_t) */
    uint8_t  type;             /* MAV_TYPE */
    uint8_t  autopilot;        /* MAV_AUTOPILOT */
    uint8_t  base_mode;        /* MAV_MODE_FLAG bitmask */
    uint8_t  system_status;    /* MAV_STATE */
    uint8_t  mavlink_version;  /* Always 3 for MAVLink v2 */
} mav_heartbeat_t;
MAV_STATIC_ASSERT(sizeof(mav_heartbeat_t) == 9, heartbeat);
#define MAVLINK_MSG_LEN_HEARTBEAT 9

/* SYS_STATUS (ID 1) --- 31 bytes.
 * The six error/drop counters are part of the base message, not extensions:
 * omitting them shifts battery_remaining and breaks the GCS battery display. */
typedef struct {
    uint32_t sensors_present;
    uint32_t sensors_enabled;
    uint32_t sensors_health;
    uint16_t load;               /* CPU load [0..1000] in 1/10 % */
    uint16_t voltage_battery;    /* mV, UINT16_MAX if unknown */
    int16_t  current_battery;    /* cA, -1 if unknown */
    uint16_t drop_rate_comm;     /* 1/100 % */
    uint16_t errors_comm;
    uint16_t errors_count1;
    uint16_t errors_count2;
    uint16_t errors_count3;
    uint16_t errors_count4;
    int8_t   battery_remaining;  /* percent, -1 if unknown */
} mav_sys_status_t;
MAV_STATIC_ASSERT(sizeof(mav_sys_status_t) == 31, sys_status);
#define MAVLINK_MSG_LEN_SYS_STATUS 31

/* SET_MODE (ID 11) --- 6 bytes (inbound) */
typedef struct {
    uint32_t custom_mode;
    uint8_t  target_system;
    uint8_t  base_mode;
} mav_set_mode_t;
MAV_STATIC_ASSERT(sizeof(mav_set_mode_t) == 6, set_mode);
#define MAVLINK_MSG_LEN_SET_MODE 6

/* PARAM_REQUEST_LIST (ID 21) --- 2 bytes (inbound) */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
} mav_param_request_list_t;
MAV_STATIC_ASSERT(sizeof(mav_param_request_list_t) == 2, param_request_list);
#define MAVLINK_MSG_LEN_PARAM_REQUEST_LIST 2

/* PARAM_VALUE (ID 22) --- 25 bytes (outbound) */
typedef struct {
    uint32_t param_value;     /* IEEE-754 bit pattern */
    uint16_t param_count;
    uint16_t param_index;
    char     param_id[16];    /* NOT null-terminated if exactly 16 chars */
    uint8_t  param_type;      /* MAV_PARAM_TYPE */
} mav_param_value_t;
MAV_STATIC_ASSERT(sizeof(mav_param_value_t) == 25, param_value);
#define MAVLINK_MSG_LEN_PARAM_VALUE 25

/* PARAM_SET (ID 23) --- 23 bytes (inbound) */
typedef struct {
    uint32_t param_value;     /* IEEE-754 bit pattern */
    uint8_t  target_system;
    uint8_t  target_component;
    char     param_id[16];
    uint8_t  param_type;
} mav_param_set_t;
MAV_STATIC_ASSERT(sizeof(mav_param_set_t) == 23, param_set);
#define MAVLINK_MSG_LEN_PARAM_SET 23

/* GPS_RAW_INT (ID 24) --- 30 bytes. Drives the GCS satellite/fix indicator,
 * which is the quickest way to watch the Section 5.7 quality gate from the
 * ground. */
typedef struct {
    uint64_t time_usec;
    int32_t  lat;                 /* degE7 */
    int32_t  lon;                 /* degE7 */
    int32_t  alt;                 /* mm MSL */
    uint16_t eph;                 /* HDOP * 100, UINT16_MAX if unknown */
    uint16_t epv;                 /* VDOP * 100 */
    uint16_t vel;                 /* cm/s ground speed */
    uint16_t cog;                 /* cdeg course over ground, 0..35999 */
    uint8_t  fix_type;            /* 0-1 none, 2 = 2D, 3 = 3D */
    uint8_t  satellites_visible;
} mav_gps_raw_int_t;
MAV_STATIC_ASSERT(sizeof(mav_gps_raw_int_t) == 30, gps_raw_int);
#define MAVLINK_MSG_LEN_GPS_RAW_INT 30

/* ATTITUDE (ID 30) --- 28 bytes */
typedef struct {
    uint32_t time_boot_ms;
    uint32_t roll;        /* IEEE-754 radians */
    uint32_t pitch;
    uint32_t yaw;
    uint32_t rollspeed;   /* IEEE-754 rad/s */
    uint32_t pitchspeed;
    uint32_t yawspeed;
} mav_attitude_t;
MAV_STATIC_ASSERT(sizeof(mav_attitude_t) == 28, attitude);
#define MAVLINK_MSG_LEN_ATTITUDE 28

/* GLOBAL_POSITION_INT (ID 33) --- 28 bytes */
typedef struct {
    uint32_t time_boot_ms;
    int32_t  lat;           /* degE7 */
    int32_t  lon;           /* degE7 */
    int32_t  alt;           /* mm MSL */
    int32_t  relative_alt;  /* mm above home */
    int16_t  vx;            /* cm/s North */
    int16_t  vy;            /* cm/s East */
    int16_t  vz;            /* cm/s Down */
    uint16_t hdg;           /* cdeg 0..35999, UINT16_MAX if unknown */
} mav_global_position_int_t;
MAV_STATIC_ASSERT(sizeof(mav_global_position_int_t) == 28, global_position_int);
#define MAVLINK_MSG_LEN_GLOBAL_POSITION_INT 28

/* MISSION_CURRENT (ID 42) --- 2 bytes (base message; extensions omitted) */
typedef struct {
    uint16_t seq;
} mav_mission_current_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_current_t) == 2, mission_current);
#define MAVLINK_MSG_LEN_MISSION_CURRENT 2

/* MISSION_REQUEST_LIST (ID 43) --- 2 bytes (inbound: GCS wants a download) */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
} mav_mission_request_list_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_request_list_t) == 2, mission_request_list);
#define MAVLINK_MSG_LEN_MISSION_REQUEST_LIST 2

/* MISSION_COUNT (ID 44) --- 4 bytes (both directions) */
typedef struct {
    uint16_t count;
    uint8_t  target_system;
    uint8_t  target_component;
} mav_mission_count_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_count_t) == 4, mission_count);
#define MAVLINK_MSG_LEN_MISSION_COUNT 4

/* MISSION_CLEAR_ALL (ID 45) --- 2 bytes (inbound) */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
} mav_mission_clear_all_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_clear_all_t) == 2, mission_clear_all);
#define MAVLINK_MSG_LEN_MISSION_CLEAR_ALL 2

/* MISSION_ITEM_REACHED (ID 46) --- 2 bytes (outbound) */
typedef struct {
    uint16_t seq;
} mav_mission_item_reached_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_item_reached_t) == 2, mission_item_reached);
#define MAVLINK_MSG_LEN_MISSION_ITEM_REACHED 2

/* MISSION_ACK (ID 47) --- 3 bytes (both directions) */
typedef struct {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t type;               /* MAV_MISSION_RESULT */
} mav_mission_ack_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_ack_t) == 3, mission_ack);
#define MAVLINK_MSG_LEN_MISSION_ACK 3

/* MISSION_REQUEST_INT (ID 51) --- 4 bytes (both directions) */
typedef struct {
    uint16_t seq;
    uint8_t  target_system;
    uint8_t  target_component;
} mav_mission_request_int_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_request_int_t) == 4, mission_request_int);
#define MAVLINK_MSG_LEN_MISSION_REQUEST_INT 4

/* REQUEST_DATA_STREAM (ID 66) --- 6 bytes (inbound, legacy rate request) */
typedef struct {
    uint16_t req_message_rate;
    uint8_t  target_system;
    uint8_t  target_component;
    uint8_t  req_stream_id;
    uint8_t  start_stop;
} mav_request_data_stream_t;
MAV_STATIC_ASSERT(sizeof(mav_request_data_stream_t) == 6, request_data_stream);
#define MAVLINK_MSG_LEN_REQUEST_DATA_STREAM 6

/* MISSION_ITEM_INT (ID 73) --- 37 bytes.
 * NOTE the wire order: the four params come FIRST, then x/y/z. The XML
 * declares target_system/target_component/seq/frame/command first, but v2
 * size-sorting moves every 4-byte field ahead of them. */
typedef struct {
    uint32_t param1;        /* IEEE-754: hold time (NAV_WAYPOINT) */
    uint32_t param2;        /* IEEE-754: acceptance radius [m] */
    uint32_t param3;        /* IEEE-754: pass-through radius */
    uint32_t param4;        /* IEEE-754: desired yaw at waypoint */
    int32_t  x;             /* Lat degE7 (or local X in cm for LOCAL_NED) */
    int32_t  y;             /* Lon degE7 */
    uint32_t z;             /* IEEE-754 altitude [m], frame-relative */
    uint16_t seq;
    uint16_t command;       /* MAV_CMD */
    uint8_t  target_system;
    uint8_t  target_component;
    uint8_t  frame;         /* MAV_FRAME */
    uint8_t  current;       /* 1 if this is the active waypoint */
    uint8_t  autocontinue;
} mav_mission_item_int_t;
MAV_STATIC_ASSERT(sizeof(mav_mission_item_int_t) == 37, mission_item_int);
#define MAVLINK_MSG_LEN_MISSION_ITEM_INT 37

/* VFR_HUD (ID 74) --- 20 bytes */
typedef struct {
    uint32_t airspeed;     /* IEEE-754 m/s */
    uint32_t groundspeed;  /* IEEE-754 m/s */
    uint32_t alt;          /* IEEE-754 m MSL */
    uint32_t climb;        /* IEEE-754 m/s, positive up */
    int16_t  heading;      /* deg, 0..359 */
    uint16_t throttle;     /* percent 0..100 */
} mav_vfr_hud_t;
MAV_STATIC_ASSERT(sizeof(mav_vfr_hud_t) == 20, vfr_hud);
#define MAVLINK_MSG_LEN_VFR_HUD 20

/* COMMAND_LONG (ID 76) --- 33 bytes (inbound) */
typedef struct {
    uint32_t param1;       /* IEEE-754 */
    uint32_t param2;
    uint32_t param3;
    uint32_t param4;
    uint32_t param5;
    uint32_t param6;
    uint32_t param7;
    uint16_t command;      /* MAV_CMD */
    uint8_t  target_system;
    uint8_t  target_component;
    uint8_t  confirmation;
} mav_command_long_t;
MAV_STATIC_ASSERT(sizeof(mav_command_long_t) == 33, command_long);
#define MAVLINK_MSG_LEN_COMMAND_LONG 33

/* COMMAND_ACK (ID 77) --- 3 bytes (outbound) */
typedef struct {
    uint16_t command;
    uint8_t  result;       /* MAV_RESULT */
} mav_command_ack_t;
MAV_STATIC_ASSERT(sizeof(mav_command_ack_t) == 3, command_ack);
#define MAVLINK_MSG_LEN_COMMAND_ACK 3

/* STATUSTEXT (ID 253) --- 51 bytes (outbound). Text is zero-padded and NOT
 * null-terminated when it fills all 50 bytes. */
typedef struct {
    uint8_t severity;      /* MAV_SEVERITY */
    char    text[50];
} mav_statustext_t;
MAV_STATIC_ASSERT(sizeof(mav_statustext_t) == 51, statustext);
#define MAVLINK_MSG_LEN_STATUSTEXT 51

#pragma pack(pop)

/* ============================================================================
 * SECTION 6: Decoded Message Container
 *
 * Filled by mavlink_decode when a complete, CRC-valid frame is received.
 * `payload` is always zero-padded to the full message length, so the caller
 * may copy it straight into the structs above regardless of how aggressively
 * the sender truncated.
 * ========================================================================= */

typedef struct {
    uint32_t msg_id;
    uint8_t  payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint8_t  len;        /* Payload length AS RECEIVED (before zero-padding) */
    uint8_t  seq;
    uint8_t  sysid;      /* Sender system ID */
    uint8_t  compid;     /* Sender component ID */
    bool     valid;
} mav_message_t;

/* ============================================================================
 * SECTION 7: X.25 CRC (CRC-16/MCRF4XX)
 *
 * Computed over: LEN, INCOMPAT, COMPAT, SEQ, SYS, COMP, MSGID[3], PAYLOAD,
 * then the message's CRC_EXTRA seed. STX is excluded. Result is appended
 * little-endian.
 * ========================================================================= */

#define X25_INIT_CRC 0xFFFF

static inline uint16_t crc16_accumulate(uint16_t crc, uint8_t byte) {
    uint8_t tmp = (uint8_t)(byte ^ (uint8_t)(crc & 0xFF));
    tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
    return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8)
                                 ^ ((uint16_t)tmp << 3)
                                 ^ ((uint16_t)tmp >> 4));
}

static inline uint16_t crc16_compute(const uint8_t *buf, size_t len, uint8_t crc_extra) {
    uint16_t crc = X25_INIT_CRC;
    size_t i;
    for (i = 0; i < len; i++) {
        crc = crc16_accumulate(crc, buf[i]);
    }
    return crc16_accumulate(crc, crc_extra);
}

/* ============================================================================
 * SECTION 8: 24-bit message ID pack/unpack (little-endian)
 * ========================================================================= */

static inline void msgid_pack(uint8_t out[3], uint32_t msgid) {
    out[0] = (uint8_t)(msgid & 0xFF);
    out[1] = (uint8_t)((msgid >> 8) & 0xFF);
    out[2] = (uint8_t)((msgid >> 16) & 0xFF);
}

static inline uint32_t msgid_unpack(const uint8_t in[3]) {
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16);
}

/* ============================================================================
 * SECTION 9: Integer-Only IEEE-754 Single-Precision Conversion
 *
 * The MAVLink wire format mandates IEEE-754 floats for several fields, but
 * the ET1031 core has no FPU: writing `(float)x` here would link in GCC's
 * soft-float library and cost hundreds of cycles per field inside the 10 Hz
 * telemetry slot of Section 3.4. These routines do the conversion with shifts
 * and compares only, on Q16.16 values, exact to within one ulp of the 23-bit
 * mantissa (Q16.16 carries at most 31 significant bits; values above 2^23
 * lose the low bits, which for metres and radians is far below sensor noise).
 *
 * Layout: [31] sign | [30:23] biased exponent | [22:0] mantissa
 *   value = (-1)^sign * 1.mantissa * 2^(exponent - 127)
 * A Q16.16 value v represents v * 2^-16, so if the highest set bit of |v| is
 * at position n, the unbiased exponent is (n - 16).
 * ========================================================================= */

#define MAV_IEEE_ZERO       0x00000000u
#define MAV_IEEE_NAN        0x7FC00000u
#define MAV_IEEE_POS_INF    0x7F800000u

/** Position of the highest set bit of a non-zero u32 (0..31). */
static inline int mav_msb_index(uint32_t v) {
    int n = 0;
    if (v & 0xFFFF0000u) { n += 16; v >>= 16; }
    if (v & 0x0000FF00u) { n +=  8; v >>=  8; }
    if (v & 0x000000F0u) { n +=  4; v >>=  4; }
    if (v & 0x0000000Cu) { n +=  2; v >>=  2; }
    if (v & 0x00000002u) { n +=  1; }
    return n;
}

/** Q16.16 -> IEEE-754 single-precision bit pattern. Integer operations only. */
static inline uint32_t fx_to_ieee754(fx_t v) {
    uint32_t sign, mag, mantissa;
    int      n, biased_exp;

    if (v == 0) return MAV_IEEE_ZERO;

    if (v < 0) {
        sign = 0x80000000u;
        /* Negating INT32_MIN overflows in signed arithmetic; widen first. */
        mag  = (uint32_t)(-(int64_t)v);
    } else {
        sign = 0u;
        mag  = (uint32_t)v;
    }

    n          = mav_msb_index(mag);      /* highest set bit, 0..31 */
    biased_exp = n - 16 + 127;            /* Q16.16 scaling folded into exponent */

    /* Drop the implicit leading 1 and align the remaining bits into 23. */
    if (n <= 23) {
        mantissa = (mag << (23 - n)) & 0x007FFFFFu;
    } else {
        mantissa = (mag >> (n - 23)) & 0x007FFFFFu;
    }

    return sign | ((uint32_t)biased_exp << 23) | mantissa;
}

/** IEEE-754 single-precision bit pattern -> Q16.16, saturating. Integer only. */
static inline fx_t ieee754_to_fx(uint32_t bits) {
    uint32_t mant;
    int      exp, shift;
    int64_t  result;
    bool     neg = (bits & 0x80000000u) != 0u;

    exp = (int)((bits >> 23) & 0xFFu);

    if (exp == 0)   return 0;   /* zero or subnormal --- below Q16.16 resolution */
    if (exp == 255) {           /* infinity or NaN --- saturate rather than trap */
        return neg ? (fx_t)0x80000000 : (fx_t)0x7FFFFFFF;
    }

    mant  = (bits & 0x007FFFFFu) | 0x00800000u;   /* restore the implicit 1 */
    /* value = mant * 2^(exp-127-23); want value * 2^16 = mant << (exp-134) */
    shift = exp - 134;

    if (shift >= 0) {
        /* mant occupies 24 bits; anything past shift 7 overflows int32. */
        if (shift > 7) return neg ? (fx_t)0x80000000 : (fx_t)0x7FFFFFFF;
        result = (int64_t)mant << shift;
    } else {
        if (shift < -24) return 0;                /* underflows to zero */
        result = (int64_t)mant >> (-shift);
    }

    if (neg) result = -result;
    if (result >  (int64_t)0x7FFFFFFF)  return (fx_t)0x7FFFFFFF;
    if (result < -(int64_t)0x80000000)  return (fx_t)0x80000000;
    return (fx_t)result;
}

/** Convenience: exact whole-number -> IEEE-754, for integral command params. */
static inline uint32_t int_to_ieee754(int32_t i) {
    return fx_to_ieee754((fx_t)((uint32_t)i << 16));
}

#endif /* MAVLINK_TYPES_H */
