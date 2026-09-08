/**
 * test_mavlink.c
 *
 * PC unit tests for the MAVLink stack. No hardware, no sockets, no GCS.
 *
 * The tests fall into three groups:
 *
 *   1. WIRE CORRECTNESS --- struct sizes, field offsets, CRC, IEEE-754.
 *      These are the failures that produce no error message at all: Mission
 *      Planner simply shows a vehicle at the wrong place, or refuses to
 *      connect, with nothing in any log to say why. They are checked against
 *      constants derived independently of the encoder, so a typo in a struct
 *      cannot make its own test pass.
 *
 *   2. PARSER ROBUSTNESS --- truncation, resynchronisation, corrupted CRC,
 *      unknown message IDs, and 200 000 bytes of pseudorandom noise. A radio
 *      link drops and garbles bytes as a matter of course; a parser that can
 *      be wedged by a bad burst is a parser that stops accepting the pilot's
 *      commands exactly when the link is worst.
 *
 *   3. PROTOCOL CONVERSATIONS --- the mission upload and download handshakes,
 *      deferred command acknowledgement, and parameter read/write, driven
 *      end-to-end by a simulated ground station built from the same encoder.
 *
 * Build:  see Makefile, target `test_mavlink`
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mavlink_types.h"
#include "mavlink_encode.h"
#include "mavlink_decode.h"
#include "mavlink_bridge.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================================
 * Tiny test harness
 * ========================================================================= */

static int tests_run = 0;
static int tests_failed = 0;
static const char *current_group = "";

static void group(const char *name) {
    current_group = name;
    printf("\n=== %s ===\n", name);
}

static void check(int cond, const char *what) {
    tests_run++;
    if (cond) {
        printf("  [ OK ] %s\n", what);
    } else {
        tests_failed++;
        printf("  [FAIL] %s   (%s)\n", what, current_group);
    }
}

static void check_near(double got, double want, double tol, const char *what) {
    tests_run++;
    if (fabs(got - want) <= tol) {
        printf("  [ OK ] %s  (got %.6f, want %.6f)\n", what, got, want);
    } else {
        tests_failed++;
        printf("  [FAIL] %s  (got %.6f, want %.6f, tol %.6f)\n",
               what, got, want, tol);
    }
}

/* ============================================================================
 * Simulated ground station
 *
 * A second encoder/parser pair standing in for Mission Planner. Frames it
 * builds go in through mavlink_bridge_handle_byte(); frames the bridge queues
 * come back out through mavlink_bridge_tx_read() and are parsed here.
 * ========================================================================= */

#define GCS_SYSID   255
#define GCS_COMPID  MAV_COMP_ID_MISSIONPLANNER

typedef struct {
    mav_encoder_t enc;
    mav_parser_t  parser;
    mav_message_t inbox[64];
    int           inbox_count;
} sim_gcs_t;

static void gcs_init(sim_gcs_t *g) {
    mav_encoder_init(&g->enc);
    mav_encoder_set_identity(&g->enc, GCS_SYSID, GCS_COMPID);
    mav_parser_init(&g->parser);
    g->inbox_count = 0;
}

/** Feed a raw frame into the bridge, byte by byte, as a UART would. */
static void gcs_send(mav_bridge_t *br, const uint8_t *frame, int len, uint32_t t_ms) {
    for (int i = 0; i < len; i++) {
        mavlink_bridge_handle_byte(br, frame[i], t_ms);
    }
}

/** Drain everything the bridge queued and parse it into the GCS inbox. */
static int gcs_receive(sim_gcs_t *g, mav_bridge_t *br) {
    uint8_t buf[MAV_TX_FIFO_SIZE];
    uint16_t n = mavlink_bridge_tx_read(br, buf, sizeof(buf));

    g->inbox_count = 0;
    for (uint16_t i = 0; i < n; i++) {
        mav_message_t m;
        if (mav_parse_byte(&g->parser, buf[i], &m)) {
            if (g->inbox_count < (int)(sizeof(g->inbox) / sizeof(g->inbox[0]))) {
                g->inbox[g->inbox_count++] = m;
            }
        }
    }
    return g->inbox_count;
}

/** Find the first message of a given ID in the inbox, or NULL. */
static const mav_message_t *gcs_find(const sim_gcs_t *g, uint32_t msg_id) {
    for (int i = 0; i < g->inbox_count; i++) {
        if (g->inbox[i].msg_id == msg_id) return &g->inbox[i];
    }
    return NULL;
}

/* --- Frame builders for inbound messages the encoder does not wrap -------- */

static int build_command_long(sim_gcs_t *g, uint16_t command,
                              uint32_t p1, uint32_t p2, uint32_t p7,
                              uint8_t *out) {
    mav_command_long_t c;
    memset(&c, 0, sizeof(c));
    c.param1           = p1;
    c.param2           = p2;
    c.param7           = p7;
    c.command          = command;
    c.target_system    = MAV_SYS_ID;
    c.target_component = MAV_COMP_ID_AUTOPILOT;
    c.confirmation     = 0;
    return mav_build_frame(&g->enc, MAVLINK_MSG_ID_COMMAND_LONG,
                           MAVLINK_CRC_EXTRA_COMMAND_LONG,
                           &c, MAVLINK_MSG_LEN_COMMAND_LONG, out);
}

static int build_set_mode(sim_gcs_t *g, uint32_t custom_mode, uint8_t *out) {
    mav_set_mode_t sm;
    sm.custom_mode   = custom_mode;
    sm.target_system = MAV_SYS_ID;
    sm.base_mode     = (uint8_t)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    return mav_build_frame(&g->enc, MAVLINK_MSG_ID_SET_MODE,
                           MAVLINK_CRC_EXTRA_SET_MODE,
                           &sm, MAVLINK_MSG_LEN_SET_MODE, out);
}

static int build_mission_request_list(sim_gcs_t *g, uint8_t *out) {
    mav_mission_request_list_t rl;
    rl.target_system    = MAV_SYS_ID;
    rl.target_component = MAV_COMP_ID_AUTOPILOT;
    return mav_build_frame(&g->enc, MAVLINK_MSG_ID_MISSION_REQUEST_LIST,
                           MAVLINK_CRC_EXTRA_MISSION_REQUEST_LIST,
                           &rl, MAVLINK_MSG_LEN_MISSION_REQUEST_LIST, out);
}

static int build_param_request_list(sim_gcs_t *g, uint8_t *out) {
    mav_param_request_list_t rl;
    rl.target_system    = MAV_SYS_ID;
    rl.target_component = MAV_COMP_ID_AUTOPILOT;
    return mav_build_frame(&g->enc, MAVLINK_MSG_ID_PARAM_REQUEST_LIST,
                           MAVLINK_CRC_EXTRA_PARAM_REQUEST_LIST,
                           &rl, MAVLINK_MSG_LEN_PARAM_REQUEST_LIST, out);
}

static int build_param_set(sim_gcs_t *g, const char *name, fx_t value, uint8_t *out) {
    mav_param_set_t ps;
    memset(&ps, 0, sizeof(ps));
    ps.param_value      = fx_to_ieee754(value);
    ps.target_system    = MAV_SYS_ID;
    ps.target_component = MAV_COMP_ID_AUTOPILOT;
    ps.param_type       = (uint8_t)MAV_PARAM_TYPE_REAL32;
    for (int i = 0; i < 16 && name[i]; i++) ps.param_id[i] = name[i];
    return mav_build_frame(&g->enc, MAVLINK_MSG_ID_PARAM_SET,
                           MAVLINK_CRC_EXTRA_PARAM_SET,
                           &ps, MAVLINK_MSG_LEN_PARAM_SET, out);
}

/* ============================================================================
 * GROUP 1 --- Wire correctness
 * ========================================================================= */

static void test_struct_sizes(void) {
    group("Wire layout: payload struct sizes");

    /* These duplicate the compile-time assertions in mavlink_types.h on
     * purpose: if someone disables the macro, the suite still catches it. */
    check(sizeof(mav_heartbeat_t)           == 9,  "HEARTBEAT is 9 bytes");
    check(sizeof(mav_sys_status_t)          == 31, "SYS_STATUS is 31 bytes");
    check(sizeof(mav_set_mode_t)            == 6,  "SET_MODE is 6 bytes");
    check(sizeof(mav_param_value_t)         == 25, "PARAM_VALUE is 25 bytes");
    check(sizeof(mav_param_set_t)           == 23, "PARAM_SET is 23 bytes");
    check(sizeof(mav_gps_raw_int_t)         == 30, "GPS_RAW_INT is 30 bytes");
    check(sizeof(mav_attitude_t)            == 28, "ATTITUDE is 28 bytes");
    check(sizeof(mav_global_position_int_t) == 28, "GLOBAL_POSITION_INT is 28 bytes");
    check(sizeof(mav_mission_count_t)       == 4,  "MISSION_COUNT is 4 bytes");
    check(sizeof(mav_mission_ack_t)         == 3,  "MISSION_ACK is 3 bytes");
    check(sizeof(mav_mission_request_int_t) == 4,  "MISSION_REQUEST_INT is 4 bytes");
    check(sizeof(mav_mission_item_int_t)    == 37, "MISSION_ITEM_INT is 37 bytes");
    check(sizeof(mav_vfr_hud_t)             == 20, "VFR_HUD is 20 bytes");
    check(sizeof(mav_command_long_t)        == 33, "COMMAND_LONG is 33 bytes");
    check(sizeof(mav_command_ack_t)         == 3,  "COMMAND_ACK is 3 bytes");
    check(sizeof(mav_statustext_t)          == 51, "STATUSTEXT is 51 bytes");
}

static void test_field_offsets(void) {
    group("Wire layout: MAVLink v2 field reordering");

    /* MAVLink v2 sorts payload fields by size, largest first. For
     * MISSION_ITEM_INT that puts param1..param4 ahead of x/y/z even though the
     * XML declares target_system/seq/frame/command first. Getting this wrong
     * is silent: every waypoint lands somewhere in the Atlantic. */
    check(offsetof(mav_mission_item_int_t, param1)       == 0,  "MISSION_ITEM_INT.param1 at offset 0");
    check(offsetof(mav_mission_item_int_t, param4)       == 12, "MISSION_ITEM_INT.param4 at offset 12");
    check(offsetof(mav_mission_item_int_t, x)            == 16, "MISSION_ITEM_INT.x (lat) at offset 16");
    check(offsetof(mav_mission_item_int_t, y)            == 20, "MISSION_ITEM_INT.y (lon) at offset 20");
    check(offsetof(mav_mission_item_int_t, z)            == 24, "MISSION_ITEM_INT.z (alt) at offset 24");
    check(offsetof(mav_mission_item_int_t, seq)          == 28, "MISSION_ITEM_INT.seq at offset 28");
    check(offsetof(mav_mission_item_int_t, command)      == 30, "MISSION_ITEM_INT.command at offset 30");
    check(offsetof(mav_mission_item_int_t, frame)        == 34, "MISSION_ITEM_INT.frame at offset 34");
    check(offsetof(mav_mission_item_int_t, autocontinue) == 36, "MISSION_ITEM_INT.autocontinue at offset 36");

    /* HEARTBEAT: the 4-byte custom_mode leads, ahead of every uint8. */
    check(offsetof(mav_heartbeat_t, custom_mode)     == 0, "HEARTBEAT.custom_mode at offset 0");
    check(offsetof(mav_heartbeat_t, type)            == 4, "HEARTBEAT.type at offset 4");
    check(offsetof(mav_heartbeat_t, base_mode)       == 6, "HEARTBEAT.base_mode at offset 6");
    check(offsetof(mav_heartbeat_t, mavlink_version) == 8, "HEARTBEAT.mavlink_version at offset 8");

    /* SYS_STATUS: battery_remaining is the last byte only if all six
     * error/drop counters are present. */
    check(offsetof(mav_sys_status_t, voltage_battery)   == 14, "SYS_STATUS.voltage_battery at offset 14");
    check(offsetof(mav_sys_status_t, battery_remaining) == 30, "SYS_STATUS.battery_remaining at offset 30");

    /* GLOBAL_POSITION_INT: four int32 then four int16. */
    check(offsetof(mav_global_position_int_t, lat) == 4,  "GLOBAL_POSITION_INT.lat at offset 4");
    check(offsetof(mav_global_position_int_t, vx)  == 20, "GLOBAL_POSITION_INT.vx at offset 20");
    check(offsetof(mav_global_position_int_t, hdg) == 26, "GLOBAL_POSITION_INT.hdg at offset 26");
}

static void test_crc(void) {
    group("X.25 CRC (CRC-16/MCRF4XX)");

    /* Published check value for CRC-16/MCRF4XX: the ASCII string "123456789"
     * with init 0xFFFF and no final XOR gives 0x6F91. This is an external
     * reference, so it validates the CRC independently of anything else here. */
    const uint8_t vec[] = "123456789";
    uint16_t crc = X25_INIT_CRC;
    for (int i = 0; i < 9; i++) crc = crc16_accumulate(crc, vec[i]);
    check(crc == 0x6F91, "check value for \"123456789\" is 0x6F91");

    /* crc16_compute() must equal an open-coded accumulate loop plus seed. */
    const uint8_t data[] = { 0x01, 0x02, 0x03, 0xFD, 0x00, 0xFF };
    uint16_t manual = X25_INIT_CRC;
    for (int i = 0; i < 6; i++) manual = crc16_accumulate(manual, data[i]);
    manual = crc16_accumulate(manual, 50);
    check(crc16_compute(data, 6, 50) == manual, "crc16_compute matches manual accumulation");

    /* CRC_EXTRA must actually participate: two different seeds over identical
     * bytes must differ, or a dialect mismatch would go undetected. */
    check(crc16_compute(data, 6, 50) != crc16_compute(data, 6, 39),
          "different CRC_EXTRA seeds give different CRCs");
}

/** Reinterpret an IEEE-754 bit pattern as a host float, for comparison only. */
static float bits_to_float(uint32_t bits) {
    union { uint32_t u; float f; } cvt;
    cvt.u = bits;
    return cvt.f;
}

static uint32_t float_to_bits(float f) {
    union { uint32_t u; float f; } cvt;
    cvt.f = f;
    return cvt.u;
}

static void test_ieee754(void) {
    group("Integer-only IEEE-754 conversion");

    /* Exact powers of two must be bit-exact, not merely close. */
    check(fx_to_ieee754(0)               == 0x00000000u, "0.0 encodes to 0x00000000");
    check(fx_to_ieee754(FX_ONE)          == 0x3F800000u, "1.0 encodes to 0x3F800000");
    check(fx_to_ieee754(-FX_ONE)         == 0xBF800000u, "-1.0 encodes to 0xBF800000");
    check(fx_to_ieee754(FX_ONE * 2)      == 0x40000000u, "2.0 encodes to 0x40000000");
    check(fx_to_ieee754(FX_ONE / 2)      == 0x3F000000u, "0.5 encodes to 0x3F000000");
    check(fx_to_ieee754(FX_ONE / 4)      == 0x3E800000u, "0.25 encodes to 0x3E800000");

    /* Round-trip back through Q16.16 must be lossless for these. */
    check(ieee754_to_fx(0x3F800000u) == FX_ONE,      "0x3F800000 decodes to 1.0");
    check(ieee754_to_fx(0xBF800000u) == -FX_ONE,     "0xBF800000 decodes to -1.0");
    check(ieee754_to_fx(0x00000000u) == 0,           "0x00000000 decodes to 0.0");

    /* Against the host FPU across the range the telemetry path actually uses:
     * angles in radians, speeds in m/s, altitudes in metres. Tolerance is one
     * Q16.16 ulp (1/65536) --- the fixed-point value itself is the limit, not
     * the conversion. */
    const float samples[] = {
        0.0f, 1.0f, -1.0f, 3.14159f, -3.14159f, 0.017453f, 9.81f, -9.81f,
        0.001f, 123.456f, -123.456f, 1000.0f, -1000.0f, 0.5f, 32767.0f
    };
    int worst_ok = 1;
    double worst = 0.0;
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
        fx_t     q       = fx_from_float(samples[i]);
        uint32_t bits    = fx_to_ieee754(q);
        double   decoded = (double)bits_to_float(bits);
        double   expect  = (double)fx_to_float(q);
        double   err     = fabs(decoded - expect);
        if (err > worst) worst = err;
        /* Relative tolerance above 2^23, absolute below. */
        double tol = (fabs(expect) > 8388608.0 / 65536.0)
                   ? fabs(expect) * 1.2e-7 : 1.0 / 65536.0;
        if (err > tol) {
            worst_ok = 0;
            printf("         sample %.6f: encoded reads back as %.9f\n",
                   (double)samples[i], decoded);
        }
    }
    check(worst_ok, "fx_to_ieee754 matches the host FPU across the flight range");
    printf("         worst absolute error: %.3e\n", worst);

    /* Decode direction, from host-produced bit patterns. */
    int decode_ok = 1;
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
        if (fabsf(samples[i]) > 32000.0f) continue;   /* outside Q16.16 range */
        fx_t   q   = ieee754_to_fx(float_to_bits(samples[i]));
        double err = fabs((double)fx_to_float(q) - (double)samples[i]);
        if (err > 1.0 / 65536.0) {
            decode_ok = 0;
            printf("         sample %.6f decoded to %.9f\n",
                   (double)samples[i], (double)fx_to_float(q));
        }
    }
    check(decode_ok, "ieee754_to_fx matches the host FPU across the flight range");

    /* Saturation rather than wraparound on the pathological inputs a GCS or a
     * corrupted frame can deliver. */
    check(ieee754_to_fx(0x7F800000u) == (fx_t)0x7FFFFFFF, "+inf saturates to FX_MAX");
    check(ieee754_to_fx(0xFF800000u) == (fx_t)0x80000000, "-inf saturates to FX_MIN");
    check(ieee754_to_fx(0x7FC00000u) == (fx_t)0x7FFFFFFF, "NaN saturates rather than wrapping");
    check(ieee754_to_fx(0x00000001u) == 0,                "subnormal decodes to 0");
    check(ieee754_to_fx(float_to_bits(1.0e9f)) == (fx_t)0x7FFFFFFF,
          "1e9 (beyond Q16.16) saturates to FX_MAX");
}

/* ============================================================================
 * GROUP 2 --- Parser robustness
 * ========================================================================= */

static void test_frame_roundtrip(void) {
    group("Encode -> decode round-trip");

    mav_encoder_t enc;
    mav_parser_t  parser;
    mav_message_t msg;
    uint8_t       frame[MAVLINK_MAX_FRAME_LEN];

    mav_encoder_init(&enc);
    mav_parser_init(&parser);

    /* --- HEARTBEAT ------------------------------------------------------ */
    int n = mav_encode_heartbeat(&enc, true, (uint8_t)MODE_MISSION,
                                 (uint8_t)MAV_STATE_ACTIVE, frame);
    check(frame[0] == MAVLINK_V2_STX, "frame starts with the v2 magic 0xFD");
    check(frame[2] == 0, "incompat_flags is 0 (frame is unsigned)");
    check(frame[5] == MAV_SYS_ID, "sysid field carries our system ID");
    check(msgid_unpack(&frame[7]) == MAVLINK_MSG_ID_HEARTBEAT, "msgid field reads back as HEARTBEAT");

    int got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "HEARTBEAT parses");

    mav_heartbeat_t hb;
    check(mav_msg_heartbeat(&msg, &hb), "HEARTBEAT extracts");
#ifdef MAV_ARDUPILOT_COMPAT
    /* Compat build remaps MODE_MISSION onto ArduCopter AUTO (custom_mode 3). */
    check(hb.custom_mode == 3u, "custom_mode is ArduCopter AUTO under compat");
    check(hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA, "autopilot is ArduPilotMega under compat");
#else
    check(hb.custom_mode == (uint32_t)MODE_MISSION, "custom_mode survives as MODE_MISSION");
    check(hb.autopilot == MAV_AUTOPILOT_GENERIC, "autopilot is GENERIC in the honest build");
#endif
    check(hb.type == MAV_TYPE_QUADROTOR, "type is QUADROTOR");
    check(hb.mavlink_version == 3, "mavlink_version is 3");
    check((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0, "armed flag set when armed");
    check((hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0,
          "CUSTOM_MODE_ENABLED set, so the GCS reads custom_mode");
    check((hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) != 0,
          "AUTO flag set in MODE_MISSION");

    /* Disarmed heartbeat must clear the armed bit --- a stuck bit here would
     * show a disarmed vehicle as armed on the GCS. */
    n = mav_encode_heartbeat(&enc, false, (uint8_t)MODE_STABILIZE,
                             (uint8_t)MAV_STATE_STANDBY, frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got && mav_msg_heartbeat(&msg, &hb), "disarmed HEARTBEAT parses");
    check((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0, "armed flag clear when disarmed");
    check((hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) == 0, "AUTO flag clear in STABILIZE");

    /* --- ATTITUDE ------------------------------------------------------- */
    fx_t roll  = fx_from_float(0.35f);
    fx_t pitch = fx_from_float(-0.20f);
    fx_t yaw   = fx_from_float(1.57f);
    n = mav_encode_attitude(&enc, 123456u, roll, pitch, yaw,
                            fx_from_float(0.01f), fx_from_float(-0.02f),
                            fx_from_float(0.03f), frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "ATTITUDE parses");

    mav_attitude_t att;
    memcpy(&att, msg.payload, sizeof(att));
    check(att.time_boot_ms == 123456u, "ATTITUDE time_boot_ms survives");
    check_near((double)bits_to_float(att.roll),  fx_to_float(roll),  1e-4, "ATTITUDE roll survives");
    check_near((double)bits_to_float(att.pitch), fx_to_float(pitch), 1e-4, "ATTITUDE pitch survives");
    check_near((double)bits_to_float(att.yaw),   fx_to_float(yaw),   1e-4, "ATTITUDE yaw survives");

    /* --- GLOBAL_POSITION_INT -------------------------------------------- */
    n = mav_encode_global_position_int(&enc, 5000u,
                                       129716000, 775946000,   /* degE7 */
                                       920000, 15000,          /* mm */
                                       fx_from_float(3.5f), fx_from_float(-1.25f),
                                       fx_from_float(0.5f), 4500, frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "GLOBAL_POSITION_INT parses");

    mav_global_position_int_t gp;
    memcpy(&gp, msg.payload, sizeof(gp));
    check(gp.lat == 129716000, "latitude survives exactly (degE7, no float rounding)");
    check(gp.lon == 775946000, "longitude survives exactly");
    check(gp.relative_alt == 15000, "relative_alt survives");
    check(gp.vx == 350,  "north velocity converts to 350 cm/s");
    check(gp.vy == -125, "east velocity converts to -125 cm/s");
    check(gp.hdg == 4500, "heading survives as centidegrees");

    /* --- MISSION_ITEM_INT ------------------------------------------------ */
    n = mav_encode_mission_item_int(&enc, 2, (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                    (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0, 1, 129720000, 775950000,
                                    fx_from_float(25.0f), fx_from_float(4.0f),
                                    MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "MISSION_ITEM_INT parses");

    mav_mission_item_int_t it;
    check(mav_msg_mission_item_int(&msg, &it), "MISSION_ITEM_INT extracts");
    check(it.seq == 2, "waypoint seq survives");
    check(it.x == 129720000, "waypoint latitude survives exactly");
    check(it.y == 775950000, "waypoint longitude survives exactly");
    check(it.command == MAV_CMD_NAV_WAYPOINT, "waypoint command survives");
    check(it.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, "waypoint frame survives");
    check_near((double)bits_to_float(it.z), 25.0, 1e-3, "waypoint altitude survives");
    check_near((double)bits_to_float(it.param2), 4.0, 1e-3, "acceptance radius survives in param2");

    /* --- STATUSTEXT ------------------------------------------------------ */
    n = mav_encode_statustext(&enc, (uint8_t)MAV_SEVERITY_WARNING,
                              "ARM DENIED: GYRO MOVING", frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "STATUSTEXT parses");
    mav_statustext_t st;
    memcpy(&st, msg.payload, sizeof(st));
    check(st.severity == MAV_SEVERITY_WARNING, "STATUSTEXT severity survives");
    check(memcmp(st.text, "ARM DENIED: GYRO MOVING", 23) == 0, "STATUSTEXT body survives");

    check(parser.stats.crc_errors == 0, "no CRC errors across the round-trip suite");
    check(parser.stats.unknown_msgid == 0, "no unknown message IDs across the round-trip suite");
}

static void test_truncation(void) {
    group("MAVLink v2 payload truncation and zero-padding");

    mav_encoder_t enc;
    mav_parser_t  parser;
    mav_message_t msg;
    uint8_t       frame[MAVLINK_MAX_FRAME_LEN];

    mav_encoder_init(&enc);
    mav_parser_init(&parser);

    /* MISSION_ACK(ACCEPTED) is three bytes of which the last two are zero, so
     * it must go out shorter than its declared length. */
    int n = mav_encode_mission_ack(&enc, 0, 0, (uint8_t)MAV_MISSION_ACCEPTED, frame);
    check(frame[1] < MAVLINK_MSG_LEN_MISSION_ACK, "trailing zero bytes are trimmed on the wire");
    check(frame[1] >= 1, "at least one payload byte is always kept");

    int got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "a truncated frame still parses");

    mav_mission_ack_t ack;
    check(mav_msg_mission_ack(&msg, &ack), "truncated MISSION_ACK extracts");
    check(ack.type == MAV_MISSION_ACCEPTED, "the trimmed-away fields read back as zero");
    check(msg.len < MAVLINK_MSG_LEN_MISSION_ACK, "received length reflects the truncation");

    /* A deliberately over-truncated MISSION_ITEM_INT: hand-build a frame that
     * carries only the first 17 bytes (through the latitude) and confirm the
     * remaining fields zero-fill rather than picking up stale memory. This is
     * what a minimal GCS actually sends for a waypoint at altitude zero. */
    mav_encoder_t enc2;
    mav_encoder_init(&enc2);
    mav_mission_item_int_t partial;
    memset(&partial, 0, sizeof(partial));
    partial.x = 129716000;
    n = mav_build_frame(&enc2, MAVLINK_MSG_ID_MISSION_ITEM_INT,
                        MAVLINK_CRC_EXTRA_MISSION_ITEM_INT,
                        &partial, MAVLINK_MSG_LEN_MISSION_ITEM_INT, frame);
    check(frame[1] == 20, "an all-zero-tail MISSION_ITEM_INT trims to 20 bytes");

    /* Poison the parser's scratch with a previous long frame first. */
    mav_encoder_t enc3;
    mav_encoder_init(&enc3);
    uint8_t noise[MAVLINK_MAX_FRAME_LEN];
    int nn = mav_encode_statustext(&enc3, 7, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", noise);
    for (int i = 0; i < nn; i++) mav_parse_byte(&parser, noise[i], &msg);

    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "over-truncated MISSION_ITEM_INT parses");

    mav_mission_item_int_t it;
    check(mav_msg_mission_item_int(&msg, &it), "over-truncated MISSION_ITEM_INT extracts");
    check(it.x == 129716000, "the field that was sent survives");
    check(it.y == 0 && it.z == 0, "fields past the truncation point are zero, not stale");
    check(it.seq == 0 && it.command == 0 && it.autocontinue == 0,
          "every trailing field zero-fills");
}

static void test_resync_and_corruption(void) {
    group("Parser resynchronisation and error detection");

    mav_encoder_t enc;
    mav_parser_t  parser;
    mav_message_t msg;
    uint8_t       frame[MAVLINK_MAX_FRAME_LEN];

    mav_encoder_init(&enc);
    mav_parser_init(&parser);

    /* The dialect-length bound the parser relies on must match the table. */
    check(mav_dialect_max_payload() <= MAVLINK_MAX_DIALECT_PAYLOAD,
          "MAVLINK_MAX_DIALECT_PAYLOAD covers every message in the dialect table");
    printf("         dialect: %u messages, longest payload %u bytes (bound %u)\n",
           mav_dialect_size(), mav_dialect_max_payload(), MAVLINK_MAX_DIALECT_PAYLOAD);

    /* Garbage before a good frame: the parser must find the frame anyway.
     * The junk deliberately contains a stray 0xFD followed by 0x99 --- a
     * length of 153, larger than anything in our dialect. Without the length
     * bound the parser would swallow the next 153 bytes, taking the real
     * frame with it. */
    const uint8_t junk[] = { 0x00, 0xAB, 0xFF, 0x12, 0xFD, 0x99, 0x01, 0x7E };
    for (int i = 0; i < (int)sizeof(junk); i++) mav_parse_byte(&parser, junk[i], &msg);

    int n = mav_encode_heartbeat(&enc, false, (uint8_t)MODE_STABILIZE,
                                 (uint8_t)MAV_STATE_STANDBY, frame);
    int got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "a valid frame is found after leading garbage");

    /* A stray start byte immediately followed by a real one must not eat the
     * real frame: the rejected length byte is re-examined as a start marker. */
    mav_parser_init(&parser);
    mav_parse_byte(&parser, 0xFD, &msg);        /* false start */
    n = mav_encode_heartbeat(&enc, true, (uint8_t)MODE_ALT_HOLD,
                             (uint8_t)MAV_STATE_ACTIVE, frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "a false start byte immediately before a real frame is discarded, not fatal");

    /* Corrupted payload: the CRC must reject it and nothing must surface. */
    uint32_t crc_before = parser.stats.crc_errors;
    n = mav_encode_attitude(&enc, 1000u, fx_from_float(0.1f), 0, 0, 0, 0, 0, frame);
    frame[12] ^= 0xFF;                       /* flip a payload byte */
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(!got, "a frame with a corrupted payload is rejected");
    check(parser.stats.crc_errors == crc_before + 1, "the CRC error is counted");

    /* The parser must not be wedged by that: the next good frame parses. */
    n = mav_encode_heartbeat(&enc, false, (uint8_t)MODE_STABILIZE,
                             (uint8_t)MAV_STATE_STANDBY, frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "the parser recovers immediately after a CRC failure");

    /* A message ID outside our dialect cannot be CRC-checked, so it must be
     * counted and discarded rather than guessed at. */
    uint32_t unknown_before = parser.stats.unknown_msgid;
    uint8_t body[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    n = mav_build_frame(&enc, 12345u, 77, body, 8, frame);
    got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(!got, "an unknown message ID does not surface as a message");
    check(parser.stats.unknown_msgid == unknown_before + 1, "the unknown message ID is counted");

    /* A frame cut off mid-stream unavoidably consumes the head of whatever
     * follows it --- the parser is still counting payload bytes and has no way
     * to know the sender stopped. No MAVLink parser can avoid that. What must
     * hold is that the loss is bounded to one frame: the frame after next has
     * to parse, or a single dropped byte would desynchronise the link
     * permanently. */
    n = mav_encode_attitude(&enc, 2000u, 0, 0, 0, 0, 0, 0, frame);
    for (int i = 0; i < n - 4; i++) mav_parse_byte(&parser, frame[i], &msg);  /* cut short */

    int recovered_after = -1;
    for (int k = 0; k < 3 && recovered_after < 0; k++) {
        n = mav_encode_heartbeat(&enc, true, (uint8_t)MODE_ALT_HOLD,
                                 (uint8_t)MAV_STATE_ACTIVE, frame);
        for (int i = 0; i < n; i++) {
            if (mav_parse_byte(&parser, frame[i], &msg)) recovered_after = k;
        }
    }
    check(recovered_after >= 0 && recovered_after <= 1,
          "a frame cut off mid-stream costs at most one following frame");
    printf("         resynchronised after %d lost frame(s)\n", recovered_after);
}

static void test_sequence_accounting(void) {
    group("Link statistics");

    mav_encoder_t enc;
    mav_parser_t  parser;
    mav_message_t msg;
    uint8_t       frame[MAVLINK_MAX_FRAME_LEN];

    mav_encoder_init(&enc);
    mav_parser_init(&parser);

    for (int k = 0; k < 5; k++) {
        int n = mav_encode_heartbeat(&enc, false, 1, 3, frame);
        for (int i = 0; i < n; i++) mav_parse_byte(&parser, frame[i], &msg);
    }
    check(parser.stats.packets_received == 5, "five frames counted as received");
    check(parser.stats.packets_lost == 0, "no loss reported on a clean sequence");

    /* Skip three sequence numbers by discarding three encoded frames. */
    for (int k = 0; k < 3; k++) mav_encode_heartbeat(&enc, false, 1, 3, frame);
    int n = mav_encode_heartbeat(&enc, false, 1, 3, frame);
    for (int i = 0; i < n; i++) mav_parse_byte(&parser, frame[i], &msg);
    check(parser.stats.packets_lost == 3, "a three-frame gap is reported as three lost");
}

static void test_v1_compatibility(void) {
    group("Legacy MAVLink v1 frames");

    /* Mission Planner opens a link speaking v1 until it sees a v2 frame from
     * us. Refusing v1 outright would stall that handshake, so the parser must
     * accept both. Build a v1 HEARTBEAT by hand. */
    mav_parser_t  parser;
    mav_message_t msg;
    mav_parser_init(&parser);

    mav_heartbeat_t hb;
    hb.custom_mode     = (uint32_t)MODE_POS_HOLD;
    hb.type            = 6;              /* GCS */
    hb.autopilot       = 8;              /* INVALID, as a GCS reports */
    hb.base_mode       = 0;
    hb.system_status   = 4;
    hb.mavlink_version = 3;

    uint8_t frame[64];
    int p = 0;
    frame[p++] = MAVLINK_V1_STX;
    frame[p++] = MAVLINK_MSG_LEN_HEARTBEAT;   /* v1 does not truncate */
    frame[p++] = 7;                           /* seq */
    frame[p++] = GCS_SYSID;
    frame[p++] = GCS_COMPID;
    frame[p++] = MAVLINK_MSG_ID_HEARTBEAT;    /* single-byte msgid */
    memcpy(&frame[p], &hb, MAVLINK_MSG_LEN_HEARTBEAT);
    p += MAVLINK_MSG_LEN_HEARTBEAT;

    uint16_t crc = crc16_compute(&frame[1], (size_t)(p - 1), MAVLINK_CRC_EXTRA_HEARTBEAT);
    frame[p++] = (uint8_t)(crc & 0xFF);
    frame[p++] = (uint8_t)(crc >> 8);

    int got = 0;
    for (int i = 0; i < p; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "a MAVLink v1 frame parses");
    check(msg.sysid == GCS_SYSID, "v1 sender system ID is recovered");
    check(msg.msg_id == MAVLINK_MSG_ID_HEARTBEAT, "v1 message ID is recovered");

    mav_heartbeat_t out;
    check(mav_msg_heartbeat(&msg, &out), "v1 HEARTBEAT extracts");
    check(out.custom_mode == (uint32_t)MODE_POS_HOLD, "v1 payload fields are correct");
}

/** Deterministic xorshift32 --- reproducible fuzz, no libc rand() dependence. */
static uint32_t rng_state = 0x1234ABCDu;
static uint8_t rng_byte(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return (uint8_t)(rng_state & 0xFFu);
}

static void test_fuzz(void) {
    group("Fuzz: 200 000 bytes of noise");

    mav_parser_t  parser;
    mav_message_t msg;
    mav_parser_init(&parser);

    int surfaced = 0;
    for (int i = 0; i < 200000; i++) {
        if (mav_parse_byte(&parser, rng_byte(), &msg)) surfaced++;
    }

    /* Random bytes can, very occasionally, form a frame whose 16-bit CRC
     * happens to match --- roughly one in 65 536 per candidate start byte, so
     * a handful of hits across 200 000 bytes is expected, not a defect. What
     * matters is that nothing crashes, nothing runs away, and the parser is
     * still usable afterwards. */
    printf("         %d spurious frames accepted, %u CRC errors, %u unknown IDs\n",
           surfaced, parser.stats.crc_errors, parser.stats.unknown_msgid);
    check(surfaced <= 5, "at most a handful of chance CRC collisions");
    check(parser.stats.crc_errors > 0, "the fuzz stream is actually reaching the CRC check");

    /* And the parser still works. */
    mav_encoder_t enc;
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    mav_encoder_init(&enc);
    int n = mav_encode_heartbeat(&enc, false, 1, 3, frame);
    int got = 0;
    for (int i = 0; i < n; i++) if (mav_parse_byte(&parser, frame[i], &msg)) got = 1;
    check(got, "the parser still accepts a good frame after the fuzz run");
}

/* ============================================================================
 * GROUP 3 --- Protocol conversations through the bridge
 * ========================================================================= */

/* IISc, Bangalore --- the home position used throughout the bridge tests. */
#define HOME_LAT_E7  129716000
#define HOME_LON_E7  775946000
#define HOME_ALT_MM     920000

static void test_ned_conversion(void) {
    group("Local NED <-> geodetic conversion (Section 2.5)");

    fx_t cos_lat = mav_cos_lat_e7(HOME_LAT_E7);
    check_near(fx_to_float(cos_lat), cos(12.9716 * M_PI / 180.0), 0.002,
               "cos(latitude) from CORDIC matches the host libm");

    /* 100 m north is 8993 units of degE7: 100 / 0.011119492664. Checking
     * against that independently-derived number, rather than against the
     * inverse function, is what makes this a real test of the formula. */
    fx_t north, east;
    mav_lla_to_ned(HOME_LAT_E7 + 8993, HOME_LON_E7,
                   HOME_LAT_E7, HOME_LON_E7, cos_lat, &north, &east);
    check_near(fx_to_float(north), 100.0, 0.1, "8993 degE7 north resolves to 100 m");
    check_near(fx_to_float(east), 0.0, 0.01, "pure north offset gives zero east");

    /* East is scaled by cos(latitude): 50 m at 12.97 deg needs 4615 degE7. */
    mav_lla_to_ned(HOME_LAT_E7, HOME_LON_E7 + 4615,
                   HOME_LAT_E7, HOME_LON_E7, cos_lat, &north, &east);
    check_near(fx_to_float(east), 50.0, 0.3, "4615 degE7 east resolves to 50 m");
    check_near(fx_to_float(north), 0.0, 0.01, "pure east offset gives zero north");

    /* Round-trip: what the mission executor flies must be what the GCS drew. */
    int32_t lat, lon;
    fx_t n2, e2;
    mav_ned_to_lla(fx_from_float(250.0f), fx_from_float(-175.0f),
                   HOME_LAT_E7, HOME_LON_E7, cos_lat, &lat, &lon);
    mav_lla_to_ned(lat, lon, HOME_LAT_E7, HOME_LON_E7, cos_lat, &n2, &e2);
    check_near(fx_to_float(n2), 250.0,  0.2, "NED -> LLA -> NED preserves north");
    check_near(fx_to_float(e2), -175.0, 0.2, "NED -> LLA -> NED preserves east");

    /* Sign conventions: north-east positive, matching the estimator. */
    mav_lla_to_ned(HOME_LAT_E7 - 8993, HOME_LON_E7 - 4615,
                   HOME_LAT_E7, HOME_LON_E7, cos_lat, &north, &east);
    check(north < 0 && east < 0, "south-west of home gives negative north and east");
}

/* End-to-end proof that a point drawn on a ground station's map arrives in the
 * mission frame at the right place. Where the group above checks the formula
 * against hand-derived degE7 constants, this one models the FULL ground-station
 * path: start from a known ground offset in metres, compute the lat/lon a map
 * shows there with independent double-precision geodesy (no firmware code), and
 * confirm the firmware's integer mav_lla_to_ned() returns those same metres.
 * These are the numbers a user reads back in the "mission uploaded" echo. */
static void test_waypoint_projection(void) {
    group("Waypoint projection: GCS map click -> mission frame");

    const double home_lat = 12.9716, home_lon = 77.5946;
    const double m_per_deg_lat = 111194.9266;
    double m_per_deg_lon = m_per_deg_lat * cos(home_lat * M_PI / 180.0);
    fx_t cos_lat = mav_cos_lat_e7(HOME_LAT_E7);

    /* {north m, east m, label} -- small map clicks through kilometre legs. */
    struct { double n, e; const char *label; } pts[] = {
        {   5.0,    0.0, "5 m North click resolves to 5 m north" },
        {   0.0,    5.0, "5 m East click resolves to 5 m east" },
        {  15.0,   10.0, "15 N / 10 E click lands where drawn" },
        { 100.0,    0.0, "100 m North is exact" },
        {   0.0,  100.0, "100 m East is exact" },
        { 250.0,  250.0, "250 N / 250 E diagonal is exact" },
        {-100.0, -100.0, "100 S / 100 W stays exact and signed" },
        {1000.0,    0.0, "1 km North holds precision at distance" },
        {   0.0, 1000.0, "1 km East holds precision at distance" },
    };
    int npts = (int)(sizeof(pts) / sizeof(pts[0]));

    for (int i = 0; i < npts; i++) {
        double lat = home_lat + pts[i].n / m_per_deg_lat;
        double lon = home_lon + pts[i].e / m_per_deg_lon;
        int32_t lat_e7 = (int32_t)(lat * 1e7 + (lat >= 0 ? 0.5 : -0.5));
        int32_t lon_e7 = (int32_t)(lon * 1e7 + (lon >= 0 ? 0.5 : -0.5));

        fx_t north_fx, east_fx;
        mav_lla_to_ned(lat_e7, lon_e7, HOME_LAT_E7, HOME_LON_E7, cos_lat,
                       &north_fx, &east_fx);

        double fn = fx_to_float(north_fx), fe = fx_to_float(east_fx);
        double err = sqrt((fn - pts[i].n) * (fn - pts[i].n) +
                          (fe - pts[i].e) * (fe - pts[i].e));
        /* 5 cm ceiling: comfortably tighter than the acceptance radius the
         * sequencer flies to, so any real regression trips this. */
        check(err < 0.05, pts[i].label);
    }
}

static void test_mission_upload(void) {
    group("Mission upload handshake (Section 10.2)");

    sim_gcs_t   gcs;
    mav_bridge_t br;
    mission_t   active;
    uint8_t     frame[MAVLINK_MAX_FRAME_LEN];
    mav_events_t ev;
    uint32_t    t = 1000;

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    /* The four waypoints the "operator" drew, in local NED metres. */
    const float wp_n[4] = {  50.0f, 150.0f, 150.0f,   0.0f };
    const float wp_e[4] = {   0.0f,   0.0f, 100.0f,   0.0f };
    const float wp_alt[4] = { 20.0f, 30.0f,  30.0f,  20.0f };

    /* Step 1: MISSION_COUNT(4) --- "I have four waypoints to send". */
    int n = mav_encode_mission_count(&gcs.enc, 4, MAV_SYS_ID,
                                     MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    const mav_message_t *m = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_REQUEST_INT);
    check(m != NULL, "MISSION_COUNT is answered with MISSION_REQUEST_INT");

    mav_mission_request_int_t rq;
    check(m && mav_msg_mission_request_int(m, &rq), "the request extracts");
    check(m && rq.seq == 0, "the vehicle asks for waypoint 0 first");

    /* Steps 2-5: send each item as it is requested. */
    for (int k = 0; k < 4; k++) {
        int32_t lat, lon;
        mav_ned_to_lla(fx_from_float(wp_n[k]), fx_from_float(wp_e[k]),
                       HOME_LAT_E7, HOME_LON_E7, mav_cos_lat_e7(HOME_LAT_E7),
                       &lat, &lon);

        t += 20;
        n = mav_encode_mission_item_int(&gcs.enc, (uint16_t)k,
                                        (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                        (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                        0, 1, lat, lon,
                                        fx_from_float(wp_alt[k]),
                                        fx_from_float(3.0f),
                                        MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
        gcs_send(&br, frame, n, t);
        gcs_receive(&gcs, &br);

        if (k < 3) {
            const mav_message_t *r = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_REQUEST_INT);
            int ok = (r != NULL) && mav_msg_mission_request_int(r, &rq) && (rq.seq == k + 1);
            char label[64];
            sprintf(label, "waypoint %d is followed by a request for %d", k, k + 1);
            check(ok, label);
        }
    }

    /* Step 6: the transfer closes with MISSION_ACK(ACCEPTED). */
    const mav_message_t *a = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK);
    check(a != NULL, "the last waypoint is answered with MISSION_ACK");
    mav_mission_ack_t ack;
    check(a && mav_msg_mission_ack(a, &ack), "the ack extracts");
    check(a && ack.type == MAV_MISSION_ACCEPTED, "the ack reports ACCEPTED");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_REQUEST_INT) == NULL,
          "no further waypoint is requested after the last one");

    /* The mission actually landed in the flight core's mission_t. */
    check(active.count == 4, "four waypoints are stored in the active mission");
    check(mission_is_loaded(&active), "the core reports a mission is loaded");
    check(active.current_index == 0, "the sequencer starts at waypoint 0");

    check_near(fx_to_float(active.waypoints[0].north), 50.0,  0.2, "waypoint 0 north");
    check_near(fx_to_float(active.waypoints[1].north), 150.0, 0.2, "waypoint 1 north");
    check_near(fx_to_float(active.waypoints[2].east),  100.0, 0.3, "waypoint 2 east");
    check_near(fx_to_float(active.waypoints[3].north), 0.0,   0.2, "waypoint 3 returns to home north");

    /* Altitude 20 m up must be stored as down = -20 m: the sign convention
     * between MAVLink (altitude up) and the NED frame (down positive) is the
     * single most consequential conversion in this file. */
    check_near(fx_to_float(active.waypoints[0].down), -20.0, 0.01,
               "waypoint 0 altitude 20 m becomes down = -20 m");
    check_near(fx_to_float(active.waypoints[1].down), -30.0, 0.01,
               "waypoint 1 altitude 30 m becomes down = -30 m");
    check_near(fx_to_float(active.waypoints[0].acceptance_radius), 3.0, 0.01,
               "acceptance radius comes through from param2");

    mavlink_bridge_poll_events(&br, &ev);
    check(ev.mission_uploaded, "the mission_uploaded event is raised");
}

static void test_home_relatch(void) {
    group("Re-latching home (Section 2.5) with a mission loaded");

    sim_gcs_t    gcs;
    mav_bridge_t br;
    mission_t    active;
    uint8_t      frame[MAVLINK_MAX_FRAME_LEN];
    uint32_t     t = 1000;

    /* The scenario: power on beside the car, upload a mission there, then
     * carry the vehicle to the launch point and arm. Home re-latches at
     * arming, as Section 2.5 requires. The waypoints must stay on the ground
     * where the operator drew them --- not move with the origin. */
    const int32_t car_lat  = HOME_LAT_E7;
    const int32_t car_lon  = HOME_LON_E7;
    /* Launch point: 400 m north, 250 m east of the car. */
    const int32_t pad_lat  = HOME_LAT_E7 + 35972;   /* 400 m / 0.011119 */
    const int32_t pad_lon  = HOME_LON_E7 + 23073;   /* 250 m, cos-corrected */

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, car_lat, car_lon, HOME_ALT_MM);

    /* Two waypoints at fixed places on the ground, expressed as the ground
     * station would send them: absolute coordinates. */
    const int32_t wp_lat[2] = { HOME_LAT_E7 + 8993, HOME_LAT_E7 + 17986 };
    const int32_t wp_lon[2] = { HOME_LON_E7,        HOME_LON_E7 + 4615  };

    int n = mav_encode_mission_count(&gcs.enc, 2, MAV_SYS_ID,
                                     MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    for (int k = 0; k < 2; k++) {
        t += 20;
        n = mav_encode_mission_item_int(&gcs.enc, (uint16_t)k,
                                        (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                        (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                        0, 1, wp_lat[k], wp_lon[k],
                                        fx_from_float(25.0f), fx_from_float(3.0f),
                                        MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
        gcs_send(&br, frame, n, t);
        gcs_receive(&gcs, &br);
    }
    check(active.count == 2, "mission uploaded against the power-on position");
    check_near(fx_to_float(active.waypoints[0].north), 100.0, 0.3,
               "waypoint 0 is 100 m north of the car");

    /* Now move home to the launch pad, as arming would. */
    mavlink_bridge_set_home(&br, pad_lat, pad_lon, HOME_ALT_MM);

    /* Waypoint 0 is 100 m north of the car, and the pad is 400 m north of the
     * car, so relative to the new origin it must now read 300 m SOUTH. If the
     * mission had merely been left alone it would still say +100 m north, and
     * the vehicle would fly 400 m past the target. */
    check_near(fx_to_float(active.waypoints[0].north), -300.0, 1.0,
               "waypoint 0 re-projects to 300 m south of the new origin");
    check_near(fx_to_float(active.waypoints[0].east), -250.0, 1.5,
               "waypoint 0 re-projects to 250 m west of the new origin");

    /* The altitude is relative to home and must NOT be re-projected: "25 m
     * above home" means the same thing wherever home is. */
    check_near(fx_to_float(active.waypoints[0].down), -25.0, 0.01,
               "waypoint altitude is unchanged by the move");
    check_near(fx_to_float(active.waypoints[0].acceptance_radius), 3.0, 0.01,
               "acceptance radius is unchanged by the move");

    /* The absolute ground position must be identical before and after. */
    fx_t nn, ee;
    int32_t back_lat, back_lon;
    mav_ned_to_lla(active.waypoints[0].north, active.waypoints[0].east,
                   pad_lat, pad_lon, mav_cos_lat_e7(pad_lat), &back_lat, &back_lon);
    mav_lla_to_ned(back_lat, back_lon, car_lat, car_lon,
                   mav_cos_lat_e7(car_lat), &nn, &ee);
    check_near(fx_to_float(nn), 100.0, 1.5,
               "the waypoint is still at the same place on the ground");

    /* A download after the move must return the original coordinates exactly,
     * so a ground station comparing what it uploaded sees no drift. */
    gcs_init(&gcs);
    n = mav_encode_mission_request_int(&gcs.enc, 0, MAV_SYS_ID,
                                       MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t + 100);
    gcs_receive(&gcs, &br);
    const mav_message_t *im = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ITEM_INT);
    mav_mission_item_int_t it;
    check(im && mav_msg_mission_item_int(im, &it), "waypoint downloads after the move");
    check(im && it.x == wp_lat[0] && it.y == wp_lon[0],
          "the download returns the exact coordinates that were uploaded");

    /* Re-latching to the same position must be a no-op, not a slow drift
     * through repeated re-projection. */
    fx_t before = active.waypoints[1].north;
    for (int k = 0; k < 50; k++) {
        mavlink_bridge_set_home(&br, pad_lat, pad_lon, HOME_ALT_MM);
    }
    check(active.waypoints[1].north == before,
          "re-latching the same position 50 times changes nothing");

    /* And the origin can be read back, so the GPS driver can use the same one. */
    int32_t glat = 0, glon = 0, galt = 0;
    check(mavlink_bridge_get_home(&br, &glat, &glon, &galt), "home reads back");
    check(glat == pad_lat && glon == pad_lon, "home reads back as the launch pad");

    mav_bridge_t fresh;
    mavlink_bridge_init(&fresh, &active);
    check(!mavlink_bridge_get_home(&fresh, &glat, &glon, &galt),
          "a fresh bridge reports no home");
}

static void test_mission_upload_failures(void) {
    group("Mission upload: refusals and partial transfers");

    sim_gcs_t    gcs;
    mav_bridge_t br;
    mission_t    active;
    uint8_t      frame[MAVLINK_MAX_FRAME_LEN];
    uint32_t     t = 1000;

    /* --- No home latched: the local frame does not exist yet ------------- */
    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);          /* deliberately no set_home */

    int n = mav_encode_mission_count(&gcs.enc, 2, MAV_SYS_ID,
                                     MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    const mav_message_t *a = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK);
    mav_mission_ack_t ack;
    check(a != NULL, "an upload with no home is answered immediately");
    check(a && mav_msg_mission_ack(a, &ack) && ack.type == MAV_MISSION_DENIED,
          "it is refused with MAV_MISSION_DENIED, not stored against origin (0,0)");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_REQUEST_INT) == NULL,
          "no waypoint is requested for a refused upload");

    /* --- More waypoints than storage ------------------------------------ */
    gcs_init(&gcs);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    n = mav_encode_mission_count(&gcs.enc, MAX_MISSION_WAYPOINTS + 1,
                                 MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);
    a = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK);
    check(a && mav_msg_mission_ack(a, &ack) && ack.type == MAV_MISSION_NO_SPACE,
          "an oversized mission is refused with NO_SPACE");

    /* --- An unsupported frame type -------------------------------------- */
    gcs_init(&gcs);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    n = mav_encode_mission_count(&gcs.enc, 1, MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    n = mav_encode_mission_item_int(&gcs.enc, 0, (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                    (uint8_t)MAV_FRAME_GLOBAL_INT,   /* absolute MSL */
                                    0, 1, HOME_LAT_E7, HOME_LON_E7,
                                    fx_from_float(920.0f), fx_from_float(3.0f),
                                    MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t + 20);
    gcs_receive(&gcs, &br);
    a = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK);
    check(a && mav_msg_mission_ack(a, &ack) && ack.type == MAV_MISSION_UNSUPPORTED_FRAME,
          "an absolute-MSL waypoint is refused rather than flown as relative");

    /* --- A transfer that dies halfway must not disturb a loaded mission -- */
    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    /* Load a known-good two-waypoint mission first. */
    waypoint_t preset[2] = {
        { fx_from_float(10.0f), 0, fx_from_float(-15.0f), fx_from_float(2.0f) },
        { fx_from_float(20.0f), 0, fx_from_float(-15.0f), fx_from_float(2.0f) }
    };
    mission_load(&active, preset, 2, false);

    n = mav_encode_mission_count(&gcs.enc, 3, MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    /* Send only the first of three, then let the retry budget run out. */
    n = mav_encode_mission_item_int(&gcs.enc, 0, (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                    (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0, 1, HOME_LAT_E7 + 1000, HOME_LON_E7,
                                    fx_from_float(40.0f), fx_from_float(3.0f),
                                    MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t + 20);
    gcs_receive(&gcs, &br);

    check(active.count == 2, "the previous mission is untouched while a transfer is in flight");

    for (uint32_t k = 1; k <= MAV_MISSION_MAX_RETRIES + 2; k++) {
        mavlink_bridge_update(&br, t + 20 + k * (MAV_MISSION_RETRY_MS + 100));
    }
    gcs_receive(&gcs, &br);

    check(active.count == 2, "an abandoned upload leaves the loaded mission intact");
    check_near(fx_to_float(active.waypoints[1].north), 20.0, 0.01,
               "the previously loaded waypoints are unchanged");
    a = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK);
    check(a && mav_msg_mission_ack(a, &ack) && ack.type == MAV_MISSION_OPERATION_CANCELLED,
          "the abandoned transfer is closed with OPERATION_CANCELLED");

    /* --- Retransmission of a duplicate item ------------------------------ */
    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    n = mav_encode_mission_count(&gcs.enc, 2, MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    n = mav_encode_mission_item_int(&gcs.enc, 0, (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                    (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0, 1, HOME_LAT_E7 + 1000, HOME_LON_E7,
                                    fx_from_float(10.0f), fx_from_float(3.0f),
                                    MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t + 20);
    gcs_receive(&gcs, &br);

    /* The GCS did not see our request and resends item 0. */
    gcs_send(&br, frame, n, t + 40);
    gcs_receive(&gcs, &br);
    const mav_message_t *r = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_REQUEST_INT);
    mav_mission_request_int_t rq;
    check(r && mav_msg_mission_request_int(r, &rq) && rq.seq == 1,
          "a duplicate item re-requests the item we are actually waiting for");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK) == NULL,
          "a duplicate does not abort the transfer");

    /* --- An out-of-order item must abort -------------------------------- */
    n = mav_encode_mission_item_int(&gcs.enc, 7, (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                    (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0, 1, HOME_LAT_E7, HOME_LON_E7,
                                    fx_from_float(10.0f), fx_from_float(3.0f),
                                    MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
    gcs_send(&br, frame, n, t + 60);
    gcs_receive(&gcs, &br);
    a = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ACK);
    check(a && mav_msg_mission_ack(a, &ack) && ack.type == MAV_MISSION_INVALID_SEQUENCE,
          "an out-of-order waypoint aborts with INVALID_SEQUENCE");
}

static void test_mission_download(void) {
    group("Mission download handshake");

    sim_gcs_t    gcs;
    mav_bridge_t br;
    mission_t    active;
    uint8_t      frame[MAVLINK_MAX_FRAME_LEN];
    uint32_t     t = 5000;

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    waypoint_t wps[3] = {
        { fx_from_float( 75.0f), fx_from_float(  0.0f), fx_from_float(-20.0f), fx_from_float(3.0f) },
        { fx_from_float( 75.0f), fx_from_float( 60.0f), fx_from_float(-35.0f), fx_from_float(3.0f) },
        { fx_from_float(  0.0f), fx_from_float(  0.0f), fx_from_float(-20.0f), fx_from_float(3.0f) }
    };
    mission_load(&active, wps, 3, false);

    /* Step 1: MISSION_REQUEST_LIST -> MISSION_COUNT. */
    int n = build_mission_request_list(&gcs, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);

    const mav_message_t *m = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_COUNT);
    mav_mission_count_t mc;
    check(m != NULL, "MISSION_REQUEST_LIST is answered with MISSION_COUNT");
    check(m && mav_msg_mission_count(m, &mc) && mc.count == 3,
          "the reported count matches the loaded mission");

    /* Step 2: request each item and check it comes back where it started. */
    for (int k = 0; k < 3; k++) {
        t += 20;
        n = mav_encode_mission_request_int(&gcs.enc, (uint16_t)k,
                                           MAV_SYS_ID, MAV_COMP_ID_AUTOPILOT, frame);
        gcs_send(&br, frame, n, t);
        gcs_receive(&gcs, &br);

        const mav_message_t *im = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ITEM_INT);
        mav_mission_item_int_t it;
        char label[80];

        sprintf(label, "waypoint %d is returned", k);
        check(im && mav_msg_mission_item_int(im, &it), label);
        if (!im) continue;

        sprintf(label, "waypoint %d has the right sequence number", k);
        check(it.seq == (uint16_t)k, label);

        sprintf(label, "waypoint %d uses a relative-altitude frame", k);
        check(it.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, label);

        /* Convert back to NED and compare against what was loaded. */
        fx_t nn, ee;
        mav_lla_to_ned(it.x, it.y, HOME_LAT_E7, HOME_LON_E7,
                       mav_cos_lat_e7(HOME_LAT_E7), &nn, &ee);
        sprintf(label, "waypoint %d north survives the download", k);
        check_near(fx_to_float(nn), fx_to_float(wps[k].north), 0.3, label);
        sprintf(label, "waypoint %d east survives the download", k);
        check_near(fx_to_float(ee), fx_to_float(wps[k].east), 0.3, label);
        sprintf(label, "waypoint %d altitude is sent up-positive", k);
        check_near((double)bits_to_float(it.z), -fx_to_float(wps[k].down), 0.01, label);
    }

    /* Step 3: an empty mission must report a count of zero, not an error. */
    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    n = build_mission_request_list(&gcs, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);
    m = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_COUNT);
    check(m && mav_msg_mission_count(m, &mc) && mc.count == 0,
          "an empty mission downloads as MISSION_COUNT(0)");
}

static void test_command_handling(void) {
    group("Command handling and deferred acknowledgement");

    sim_gcs_t    gcs;
    mav_bridge_t br;
    mission_t    active;
    mav_events_t ev;
    uint8_t      frame[MAVLINK_MAX_FRAME_LEN];
    uint32_t     t = 2000;

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);

    /* --- Arm: the event is raised but no ACK is sent yet ----------------- */
    int n = build_command_long(&gcs, (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM,
                               int_to_ieee754(1), 0, 0, frame);
    gcs_send(&br, frame, n, t);

    mavlink_bridge_poll_events(&br, &ev);
    check(ev.arm_request, "an arm command raises the arm event");
    check(ev.arm_value, "the event asks to arm, not disarm");

    gcs_receive(&gcs, &br);
    check(gcs_find(&gcs, MAVLINK_MSG_ID_COMMAND_ACK) == NULL,
          "no ACK is sent before the flight core has ruled on it");

    /* The flight core refuses --- the GCS must be told the truth, with a
     * reason the operator can act on. */
    mavlink_bridge_command_result(&br, (uint8_t)MAV_RESULT_DENIED,
                                  "ARM DENIED: GYRO MOVING");
    gcs_receive(&gcs, &br);

    const mav_message_t *m = gcs_find(&gcs, MAVLINK_MSG_ID_COMMAND_ACK);
    mav_command_ack_t ack;
    check(m != NULL, "the ACK is sent once the core reports back");
    if (m) memcpy(&ack, m->payload, sizeof(ack));
    check(m && ack.command == MAV_CMD_COMPONENT_ARM_DISARM, "the ACK names the right command");
    check(m && ack.result == MAV_RESULT_DENIED, "the ACK reports the real verdict, not ACCEPTED");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_STATUSTEXT) != NULL,
          "the denial reason is surfaced as STATUSTEXT");

    /* --- Disarm ---------------------------------------------------------- */
    t += 100;
    n = build_command_long(&gcs, (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);
    check(ev.arm_request && !ev.arm_value, "param1 = 0 is decoded as disarm");
    mavlink_bridge_command_result(&br, (uint8_t)MAV_RESULT_ACCEPTED, NULL);
    gcs_receive(&gcs, &br);

    /* --- Mode change via COMMAND_LONG ------------------------------------ */
    t += 100;
    n = build_command_long(&gcs, (uint16_t)MAV_CMD_DO_SET_MODE,
                           int_to_ieee754(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                           int_to_ieee754(MODE_POS_HOLD), 0, frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);
    check(ev.mode_request, "DO_SET_MODE raises the mode event");
    check(ev.mode_value == MODE_POS_HOLD, "the requested mode is POS_HOLD");
    mavlink_bridge_command_result(&br, (uint8_t)MAV_RESULT_ACCEPTED, NULL);
    gcs_receive(&gcs, &br);

    /* --- Mode change via the SET_MODE message ---------------------------- */
    t += 100;
    n = build_set_mode(&gcs, (uint32_t)MODE_ALT_HOLD, frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);
    check(ev.mode_request && ev.mode_value == MODE_ALT_HOLD,
          "the SET_MODE message also raises the mode event");

    /* --- Takeoff carries its altitude in param7 -------------------------- */
    t += 100;
    n = build_command_long(&gcs, (uint16_t)MAV_CMD_NAV_TAKEOFF, 0, 0,
                           fx_to_ieee754(fx_from_float(12.5f)), frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);
    check(ev.takeoff_request, "NAV_TAKEOFF raises the takeoff event");
    check_near(fx_to_float(ev.takeoff_alt_m), 12.5, 0.01, "takeoff altitude comes from param7");
    mavlink_bridge_command_result(&br, (uint8_t)MAV_RESULT_ACCEPTED, NULL);
    gcs_receive(&gcs, &br);

    /* --- RTL and MISSION_START ------------------------------------------- */
    t += 100;
    n = build_command_long(&gcs, (uint16_t)MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);
    check(ev.rtl_request, "RETURN_TO_LAUNCH raises the RTL event");
    mavlink_bridge_command_result(&br, (uint8_t)MAV_RESULT_ACCEPTED, NULL);
    gcs_receive(&gcs, &br);

    t += 100;
    n = build_command_long(&gcs, (uint16_t)MAV_CMD_MISSION_START, 0, 0, 0, frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);
    check(ev.mission_start_request, "MISSION_START raises the mission-start event");
    mavlink_bridge_command_result(&br, (uint8_t)MAV_RESULT_ACCEPTED, NULL);
    gcs_receive(&gcs, &br);

    /* --- An unsupported command is answered at once, not deferred -------- */
    t += 100;
    n = build_command_long(&gcs, 9999, 0, 0, 0, frame);
    gcs_send(&br, frame, n, t);
    gcs_receive(&gcs, &br);
    m = gcs_find(&gcs, MAVLINK_MSG_ID_COMMAND_ACK);
    check(m != NULL, "an unsupported command is acknowledged immediately");
    if (m) memcpy(&ack, m->payload, sizeof(ack));
    check(m && ack.result == MAV_RESULT_UNSUPPORTED, "it is answered UNSUPPORTED");
    mavlink_bridge_poll_events(&br, &ev);
    check(!ev.arm_request && !ev.mode_request,
          "an unsupported command raises no event in the flight core");

    /* --- A core that never answers must not leave the GCS hanging -------- */
    t += 100;
    n = build_command_long(&gcs, (uint16_t)MAV_CMD_COMPONENT_ARM_DISARM,
                           int_to_ieee754(1), 0, 0, frame);
    gcs_send(&br, frame, n, t);
    mavlink_bridge_poll_events(&br, &ev);      /* core reads it but never replies */
    gcs_receive(&gcs, &br);
    check(gcs_find(&gcs, MAVLINK_MSG_ID_COMMAND_ACK) == NULL, "still no ACK while pending");

    mavlink_bridge_update(&br, t + MAV_CMD_ACK_TIMEOUT_MS + 100);
    gcs_receive(&gcs, &br);
    m = gcs_find(&gcs, MAVLINK_MSG_ID_COMMAND_ACK);
    check(m != NULL, "an unanswered command times out into an ACK");
    if (m) memcpy(&ack, m->payload, sizeof(ack));
    check(m && ack.result == MAV_RESULT_TEMPORARILY_REJECTED,
          "the timeout answers TEMPORARILY_REJECTED so the GCS can retry");
}

static void test_parameters(void) {
    group("Parameter read and write");

    sim_gcs_t    gcs;
    mav_bridge_t br;
    mission_t    active;
    uint8_t      frame[MAVLINK_MAX_FRAME_LEN];
    uint32_t     t = 3000;

    /* A stand-in for the Section 7 gain storage. */
    fx_t kp_roll  = fx_from_float(4.5f);
    fx_t ki_roll  = fx_from_float(0.05f);
    fx_t kd_roll  = fx_from_float(0.12f);
    fx_t kp_pitch = fx_from_float(4.5f);

    const mav_param_entry_t table[] = {
        { "RATE_RLL_P", &kp_roll  },
        { "RATE_RLL_I", &ki_roll  },
        { "RATE_RLL_D", &kd_roll  },
        { "RATE_PIT_P", &kp_pitch }
    };

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_param_table(&br, table, 4);

    /* PARAM_REQUEST_LIST streams the table out, paced across update() calls. */
    int n = build_param_request_list(&gcs, frame);
    gcs_send(&br, frame, n, t);

    int seen = 0;
    fx_t recovered[4] = { 0, 0, 0, 0 };
    for (int pass = 0; pass < 6; pass++) {
        mavlink_bridge_update(&br, t + (uint32_t)pass * 100u);
        gcs_receive(&gcs, &br);
        for (int i = 0; i < gcs.inbox_count; i++) {
            if (gcs.inbox[i].msg_id != MAVLINK_MSG_ID_PARAM_VALUE) continue;
            mav_param_value_t pv;
            memcpy(&pv, gcs.inbox[i].payload, sizeof(pv));
            if (pv.param_index < 4) {
                recovered[pv.param_index] = ieee754_to_fx(pv.param_value);
                seen++;
            }
            check(pv.param_count == 4, "PARAM_VALUE reports the full table size");
        }
    }
    check(seen >= 4, "every parameter in the table is streamed out");
    check_near(fx_to_float(recovered[0]), 4.5,  0.001, "RATE_RLL_P value is correct");
    check_near(fx_to_float(recovered[1]), 0.05, 0.001, "RATE_RLL_I value is correct");
    check_near(fx_to_float(recovered[2]), 0.12, 0.001, "RATE_RLL_D value is correct");

    /* PARAM_SET writes through the live pointer and echoes the stored value. */
    n = build_param_set(&gcs, "RATE_RLL_P", fx_from_float(6.25f), frame);
    gcs_send(&br, frame, n, t + 1000);
    gcs_receive(&gcs, &br);

    check_near(fx_to_float(kp_roll), 6.25, 0.001, "PARAM_SET updates the live gain");
    check_near(fx_to_float(kp_pitch), 4.5, 0.001, "PARAM_SET does not disturb other gains");

    const mav_message_t *m = gcs_find(&gcs, MAVLINK_MSG_ID_PARAM_VALUE);
    check(m != NULL, "PARAM_SET is echoed back as PARAM_VALUE");
    if (m) {
        mav_param_value_t pv;
        memcpy(&pv, m->payload, sizeof(pv));
        check_near((double)fx_to_float(ieee754_to_fx(pv.param_value)), 6.25, 0.001,
                   "the echo carries the value actually stored");
        check(memcmp(pv.param_id, "RATE_RLL_P", 10) == 0, "the echo names the right parameter");
    }

    /* An unknown parameter name must be ignored, not matched by prefix. */
    n = build_param_set(&gcs, "RATE_RLL", fx_from_float(99.0f), frame);
    gcs_send(&br, frame, n, t + 2000);
    gcs_receive(&gcs, &br);
    check_near(fx_to_float(kp_roll), 6.25, 0.001,
               "a prefix of a real parameter name does not match");
}

static void test_telemetry_tiers(void) {
    group("Telemetry tiers and link health");

    sim_gcs_t    gcs;
    mav_bridge_t br;
    mission_t    active;
    mav_flight_snapshot_t s;

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);
    mavlink_bridge_set_home(&br, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    memset(&s, 0, sizeof(s));
    s.time_boot_ms = 42000;
    s.roll         = fx_from_float(0.10f);
    s.pitch        = fx_from_float(-0.05f);
    s.yaw          = fx_from_float(1.5708f);      /* 90 degrees, due east */
    s.altitude_rel = fx_from_float(30.0f);
    s.climb_rate   = fx_from_float(1.5f);
    s.pos_n        = fx_from_float(120.0f);
    s.pos_e        = fx_from_float(-45.0f);
    s.vel_n        = fx_from_float(3.0f);
    s.vel_e        = fx_from_float(4.0f);         /* |v| = 5 m/s exactly */
    s.vel_d        = fx_from_float(-1.5f);
    s.gps_lat_e7   = HOME_LAT_E7;
    s.gps_lon_e7   = HOME_LON_E7;
    s.gps_alt_mm_msl = HOME_ALT_MM;
    s.gps_fix_type = 3;
    s.gps_num_sats = 11;
    s.gps_eph      = 120;
    s.mode         = MODE_POS_HOLD;
    s.armed        = true;
    s.sys_state    = (uint8_t)MAV_STATE_ACTIVE;
    s.battery_mv   = 11400;
    s.battery_pct  = 78;
    s.cpu_load     = 320;
    s.throttle_pct = 55;
    s.sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                        MAV_SYS_STATUS_SENSOR_GPS;
    s.sensors_enabled = s.sensors_present;
    s.sensors_health  = s.sensors_present;

    mavlink_bridge_tick_1hz(&br, &s);
    mavlink_bridge_tick_10hz(&br, &s);
    mavlink_bridge_tick_5hz(&br, &s);
    gcs_receive(&gcs, &br);

    check(gcs_find(&gcs, MAVLINK_MSG_ID_HEARTBEAT)  != NULL, "the 1 Hz tier sends HEARTBEAT");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_SYS_STATUS) != NULL, "the 1 Hz tier sends SYS_STATUS");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_ATTITUDE)   != NULL, "the 10 Hz tier sends ATTITUDE");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_GLOBAL_POSITION_INT) != NULL,
          "the 5 Hz tier sends GLOBAL_POSITION_INT");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_GPS_RAW_INT) != NULL, "the 5 Hz tier sends GPS_RAW_INT");
    check(gcs_find(&gcs, MAVLINK_MSG_ID_VFR_HUD)     != NULL, "the 5 Hz tier sends VFR_HUD");

    /* SYS_STATUS battery must survive at the right offset --- this is the
     * field the missing error counters used to displace. */
    const mav_message_t *m = gcs_find(&gcs, MAVLINK_MSG_ID_SYS_STATUS);
    if (m) {
        mav_sys_status_t st;
        memcpy(&st, m->payload, sizeof(st));
        check(st.voltage_battery == 11400, "SYS_STATUS reports 11.4 V");
        check(st.battery_remaining == 78, "SYS_STATUS reports 78 % remaining");
        check(st.load == 320, "SYS_STATUS reports the CPU load");
        check(st.sensors_health == s.sensors_present, "SYS_STATUS reports sensor health");
    }

    /* The map position must come from the fused estimator, resolved through
     * the same home reference the mission frame uses. */
    m = gcs_find(&gcs, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    if (m) {
        mav_global_position_int_t gp;
        memcpy(&gp, m->payload, sizeof(gp));

        fx_t nn, ee;
        mav_lla_to_ned(gp.lat, gp.lon, HOME_LAT_E7, HOME_LON_E7,
                       mav_cos_lat_e7(HOME_LAT_E7), &nn, &ee);
        check_near(fx_to_float(nn), 120.0,  0.3, "reported latitude reflects the fused north position");
        check_near(fx_to_float(ee), -45.0,  0.3, "reported longitude reflects the fused east position");
        check(gp.relative_alt == 30000, "relative altitude is reported in millimetres");
        check(gp.alt == HOME_ALT_MM + 30000, "MSL altitude adds the home elevation");
        check(gp.vx == 300 && gp.vy == 400, "NED velocity is reported in cm/s");
        check(gp.hdg > 8900 && gp.hdg < 9100, "yaw of pi/2 reports as ~90 degrees heading");
    }

    /* VFR_HUD ground speed is the horizontal magnitude: 3-4-5. */
    m = gcs_find(&gcs, MAVLINK_MSG_ID_VFR_HUD);
    if (m) {
        mav_vfr_hud_t hud;
        memcpy(&hud, m->payload, sizeof(hud));
        check_near((double)bits_to_float(hud.groundspeed), 5.0, 0.05,
                   "ground speed is the horizontal velocity magnitude");
        check_near((double)bits_to_float(hud.alt), 30.0, 0.01, "HUD altitude is relative to home");
        check_near((double)bits_to_float(hud.climb), 1.5, 0.01, "HUD climb rate is positive up");
        check(hud.throttle == 55, "HUD throttle percentage survives");
    }

    /* GPS_RAW_INT carries the raw receiver values, not the fused ones. */
    m = gcs_find(&gcs, MAVLINK_MSG_ID_GPS_RAW_INT);
    if (m) {
        mav_gps_raw_int_t gr;
        memcpy(&gr, m->payload, sizeof(gr));
        check(gr.fix_type == 3, "GPS fix type is reported");
        check(gr.satellites_visible == 11, "satellite count is reported");
        check(gr.lat == HOME_LAT_E7, "GPS_RAW_INT carries the raw receiver latitude");
    }

    /* MISSION_CURRENT only while a mission is running. */
    check(gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_CURRENT) == NULL,
          "MISSION_CURRENT is not sent in POS_HOLD");
    s.mode = MODE_MISSION;
    s.mission_current_seq = 2;
    mavlink_bridge_tick_5hz(&br, &s);
    gcs_receive(&gcs, &br);
    m = gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_CURRENT);
    check(m != NULL, "MISSION_CURRENT is sent in MODE_MISSION");
    if (m) {
        mav_mission_current_t mc;
        memcpy(&mc, m->payload, sizeof(mc));
        check(mc.seq == 2, "MISSION_CURRENT reports the active waypoint index");
    }

    /* Waypoint-reached notification. */
    mavlink_bridge_notify_waypoint_reached(&br, 2);
    gcs_receive(&gcs, &br);
    check(gcs_find(&gcs, MAVLINK_MSG_ID_MISSION_ITEM_REACHED) != NULL,
          "reaching a waypoint sends MISSION_ITEM_REACHED");

    /* --- Link loss (Section 11.2) ---------------------------------------- */
    mav_bridge_t br2;
    mavlink_bridge_init(&br2, &active);
    check(mavlink_bridge_link_lost(&br2, 0), "the link counts as lost before any GCS message");

    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    mav_encoder_t gcs_enc;
    mav_encoder_init(&gcs_enc);
    mav_encoder_set_identity(&gcs_enc, GCS_SYSID, GCS_COMPID);
    int n = mav_encode_heartbeat(&gcs_enc, false, 0, 4, frame);
    gcs_send(&br2, frame, n, 10000);

    check(!mavlink_bridge_link_lost(&br2, 10000), "a fresh heartbeat clears the link-lost flag");
    check(!mavlink_bridge_link_lost(&br2, 10000 + MAV_GCS_TIMEOUT_MS - 1),
          "the link is still up just before the timeout");
    check(mavlink_bridge_link_lost(&br2, 10000 + MAV_GCS_TIMEOUT_MS + 1),
          "the link is reported lost after the timeout");
    check(br2.gcs_sysid == GCS_SYSID, "the bridge learns the GCS system ID from its first frame");
}

static void test_tx_fifo(void) {
    group("Transmit FIFO overflow behaviour");

    mav_bridge_t br;
    mission_t    active;
    mav_flight_snapshot_t s;
    sim_gcs_t    gcs;

    gcs_init(&gcs);
    mission_clear(&active);
    mavlink_bridge_init(&br, &active);

    memset(&s, 0, sizeof(s));
    s.roll = fx_from_float(0.25f);
    s.mode = MODE_STABILIZE;

    /* Flood the FIFO without draining it: a slow or stalled UART. */
    for (int i = 0; i < 200; i++) mavlink_bridge_tick_10hz(&br, &s);

    check(br.tx_dropped_frames > 0, "frames are dropped once the FIFO is full");
    check(mavlink_bridge_tx_pending(&br) < MAV_TX_FIFO_SIZE, "the FIFO never reports past its size");

    /* Whatever survived must still be whole frames --- a half-written frame
     * would cost the GCS a resynchronisation, which is worse than a gap. */
    int parsed = gcs_receive(&gcs, &br);
    check(parsed > 0, "the surviving bytes still parse as complete frames");
    check(gcs.parser.stats.crc_errors == 0, "no partial frame was ever queued");
    printf("         %d whole frames recovered, %u dropped\n", parsed, br.tx_dropped_frames);

    /* Once drained, the bridge works normally again. */
    mavlink_bridge_tick_10hz(&br, &s);
    parsed = gcs_receive(&gcs, &br);
    check(parsed == 1, "the bridge resumes normally once the FIFO is drained");
}

/* ============================================================================
 * main
 * ========================================================================= */

int main(void) {
    printf("========================================================\n");
    printf(" MAVLink v2 stack --- unit tests\n");
    printf(" Aries V3 fixed-point flight controller\n");
    printf("========================================================\n");

    test_struct_sizes();
    test_field_offsets();
    test_crc();
    test_ieee754();

    test_frame_roundtrip();
    test_truncation();
    test_resync_and_corruption();
    test_sequence_accounting();
    test_v1_compatibility();
    test_fuzz();

    test_ned_conversion();
    test_waypoint_projection();
    test_mission_upload();
    test_home_relatch();
    test_mission_upload_failures();
    test_mission_download();
    test_command_handling();
    test_parameters();
    test_telemetry_tiers();
    test_tx_fifo();

    printf("\n========================================================\n");
    printf(" %d checks run, %d failed\n", tests_run, tests_failed);
    printf(" %s\n", tests_failed == 0 ? "ALL TESTS PASSED" : "*** FAILURES PRESENT ***");
    printf("========================================================\n");

    return tests_failed == 0 ? 0 : 1;
}
