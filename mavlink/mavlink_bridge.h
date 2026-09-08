/**
 * mavlink_bridge.h
 *
 * MAVLink Ground-Control Bridge --- ties the encoder and decoder to flight state
 *
 * This is the only file in the MAVLink stack that knows what a waypoint or a
 * flight mode means. The encoder packs bytes, the decoder unpacks them; the
 * bridge runs the conversations built out of them:
 *
 *   * periodic telemetry, called from the superloop tiers of Section 3.4;
 *   * the mission upload/download handshake of Section 10.2, staged and only
 *     committed once complete;
 *   * command handling (arm, disarm, mode change, RTL, mission start), which
 *     is *deferred* to the flight core rather than acted on here;
 *   * parameter read/write, so PID gains can be tuned from the GCS;
 *   * link-loss detection feeding the Section 11.2 guardrail matrix.
 *
 * ---------------------------------------------------------------------------
 * TWO RULES THIS MODULE HOLDS TO
 * ---------------------------------------------------------------------------
 *
 * 1. The bridge NEVER changes flight state. A COMMAND_LONG asking to arm
 *    raises an event; the superloop calls attempt_arm() and reports the real
 *    outcome back with mavlink_bridge_command_result(). The COMMAND_ACK the
 *    GCS receives is therefore the actual verdict of the Section 11.1 arming
 *    interlocks, not an optimistic guess. An "Arm" button that reports success
 *    while the interlocks refused would be worse than no button.
 *
 * 2. The bridge NEVER touches hardware. Outbound bytes accumulate in an
 *    internal FIFO that the caller drains into whatever UART it likes:
 *
 *        n = mavlink_bridge_tx_read(&bridge, buf, sizeof(buf));
 *        taurus_uart_write(UART_GCS, buf, n);   // the only hardware line
 *
 *    A FIFO rather than an out-parameter because inbound bytes can generate
 *    replies at unpredictable moments --- a MISSION_ITEM_INT arriving mid-byte
 *    -stream must trigger the next MISSION_REQUEST_INT immediately, and there
 *    is no out-buffer in scope inside a UART receive interrupt.
 */

#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#include "mavlink_types.h"
#include "mavlink_encode.h"
#include "mavlink_decode.h"

/* From the flight core (build with -I../core) */
#include "flight_modes.h"
#include "mission.h"

/* ============================================================================
 * Tunables
 * ========================================================================= */

/** Outbound FIFO. Sized to hold roughly one second of the 10 Hz ATTITUDE
 *  stream plus a heartbeat burst, so a caller that drains once per superloop
 *  pass never loses a frame. */
#define MAV_TX_FIFO_SIZE            1024

/** How long the bridge waits for the flight core to rule on a command before
 *  answering the GCS itself with TEMPORARILY_REJECTED. */
#define MAV_CMD_ACK_TIMEOUT_MS      1000u

/** Mission transfer: retransmit interval and give-up count, per the MAVLink
 *  mission protocol's recommended 1.5 s / 5 attempts. */
#define MAV_MISSION_RETRY_MS        1500u
#define MAV_MISSION_MAX_RETRIES     5u

/** Section 11.2: no GCS heartbeat for this long counts as ground-link loss.
 *  Note this is the *telemetry* link, not the RC link --- per the guardrail
 *  matrix, losing it does not by itself abort a mission. */
#define MAV_GCS_TIMEOUT_MS          5000u

/** PARAM_VALUE frames emitted per mavlink_bridge_update() call while streaming
 *  a parameter list, so a 30-parameter dump cannot flood the FIFO in one go. */
#define MAV_PARAMS_PER_UPDATE       3u

/* ============================================================================
 * Flight State Snapshot
 *
 * Filled by the caller immediately before a tick. Passing a snapshot rather
 * than pointers into the live estimator state keeps the bridge free of any
 * assumption about what is updating concurrently in an interrupt.
 *
 * Every quantity is Q16.16 or a raw integer --- there is no float anywhere on
 * this path.
 * ========================================================================= */

typedef struct {
    uint32_t time_boot_ms;

    /* Attitude estimate (Sections 6.2--6.4) */
    fx_t roll, pitch, yaw;                  /* rad */
    fx_t rollspeed, pitchspeed, yawspeed;   /* rad/s, body frame */

    /* Altitude estimate (Section 6.5) */
    fx_t altitude_rel;      /* m above home, positive up */
    fx_t climb_rate;        /* m/s, positive up */

    /* Horizontal estimate (Section 6.6), local NED from home */
    fx_t pos_n, pos_e;      /* m */
    fx_t vel_n, vel_e, vel_d; /* m/s, NED (vel_d positive down) */

    /* Raw GPS, for the GCS fix/satellite indicator (Section 5.7 gate) */
    int32_t gps_lat_e7, gps_lon_e7;
    int32_t gps_alt_mm_msl;
    uint8_t gps_fix_type;       /* 0/1 none, 2 = 2D, 3 = 3D */
    uint8_t gps_num_sats;
    uint16_t gps_eph;           /* HDOP * 100, UINT16_MAX if unknown */

    /* Mode and safety */
    flight_mode_t mode;
    bool     armed;
    uint8_t  sys_state;         /* MAV_STATE */
    uint16_t mission_current_seq;

    /* Housekeeping */
    uint16_t battery_mv;
    int8_t   battery_pct;
    uint16_t cpu_load;          /* 0..1000, 1/10 % --- Section 12 headroom */
    uint16_t throttle_pct;      /* 0..100 */
    uint32_t sensors_present;
    uint32_t sensors_enabled;
    uint32_t sensors_health;
} mav_flight_snapshot_t;

/* ============================================================================
 * Events raised to the flight core
 *
 * Polled and cleared by mavlink_bridge_poll_events(). Anything that changes
 * vehicle behaviour appears here rather than being applied inside the bridge.
 * ========================================================================= */

typedef struct {
    bool arm_request;           /* GCS asked to arm/disarm  */
    bool arm_value;             /*   true = arm, false = disarm */

    bool mode_request;          /* GCS asked for a mode change */
    flight_mode_t mode_value;

    bool mission_start_request; /* MAV_CMD_MISSION_START */
    bool rtl_request;           /* MAV_CMD_NAV_RETURN_TO_LAUNCH */

    bool takeoff_request;       /* MAV_CMD_NAV_TAKEOFF */
    fx_t takeoff_alt_m;

    bool mission_uploaded;      /* A new mission was committed to the core */
    bool mission_cleared;       /* The GCS cleared the mission */
} mav_events_t;

/* ============================================================================
 * Parameter table
 *
 * The caller supplies an array of live pointers (typically the PID gains of
 * Section 7). PARAM_REQUEST_LIST streams them out; PARAM_SET writes through
 * the pointer and echoes the stored value back, which is how Mission Planner
 * confirms a write took effect.
 *
 * Values are Q16.16 and are advertised as REAL32; the conversion is exact for
 * gains of the magnitude this architecture uses.
 * ========================================================================= */

typedef struct {
    const char *name;   /* up to 16 chars, GCS-visible */
    fx_t       *value;  /* live pointer into the flight core's gain storage */
} mav_param_entry_t;

/* ============================================================================
 * Mission transfer state
 * ========================================================================= */

typedef enum {
    MAV_XFER_IDLE = 0,
    MAV_XFER_UPLOAD,     /* Receiving items from the GCS */
    MAV_XFER_DOWNLOAD    /* Sending items to the GCS */
} mav_xfer_state_t;

/* ============================================================================
 * Bridge instance --- one per GCS link, entirely statically sized
 * ========================================================================= */

typedef struct {
    mav_encoder_t enc;
    mav_parser_t  parser;

    /* Peer identity, learned from the first message the GCS sends. Replies go
     * back to that system/component rather than to a hardcoded ID. */
    uint8_t  gcs_sysid;
    uint8_t  gcs_compid;
    bool     gcs_seen;
    uint32_t last_gcs_msg_ms;

    /* Home / local NED origin (Section 2.5). */
    int32_t  home_lat_e7;
    int32_t  home_lon_e7;
    int32_t  home_alt_mm_msl;
    fx_t     cos_lat_home;      /* cached; East scaling depends on it */
    bool     home_valid;

    /* Geodetic originals of each uploaded waypoint.
     *
     * The flight core stores waypoints as metres from home, which makes home
     * the origin of the map: move it and every waypoint silently moves with
     * it. Keeping what the ground station actually sent means home can be
     * re-latched at arming, as Section 2.5 requires, and the mission
     * re-projected against the new origin instead of drifting. Costs 512
     * bytes; the alternative is a mission that means something different
     * after arming than it did at upload. */
    int32_t  wp_lat_e7[MAX_MISSION_WAYPOINTS];
    int32_t  wp_lon_e7[MAX_MISSION_WAYPOINTS];
    uint8_t  wp_geo_count;

    /* Mission transfer */
    mav_xfer_state_t xfer_state;
    uint16_t xfer_count;        /* Items in the transfer */
    uint16_t xfer_seq;          /* Next item expected (upload) or sent (download) */
    uint32_t xfer_last_ms;
    uint8_t  xfer_retries;
    uint8_t  xfer_peer_sys;
    uint8_t  xfer_peer_comp;
    mission_t staging;          /* Upload lands here; promoted only when complete */

    /* Deferred COMMAND_ACK */
    bool     cmd_pending;
    uint16_t cmd_pending_id;
    uint32_t cmd_pending_ms;
    uint8_t  cmd_peer_sys;
    uint8_t  cmd_peer_comp;

    /* Parameter streaming */
    const mav_param_entry_t *params;
    uint16_t param_count;
    bool     param_stream_active;
    uint16_t param_stream_index;

    /* Outbound FIFO */
    uint8_t  tx_fifo[MAV_TX_FIFO_SIZE];
    uint16_t tx_head;           /* next byte to read out */
    uint16_t tx_tail;           /* next slot to write into */
    uint32_t tx_dropped_frames;

    /* Live mission owned by the flight core; the bridge reads it for
     * downloads and calls mission_load() on it when an upload completes. */
    mission_t *active_mission;

    mav_events_t events;
} mav_bridge_t;

/* ============================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @param active_mission The flight core's live mission_t. The bridge reads it
 *        to serve downloads and commits uploads into it via mission_load().
 *        May be NULL if mission transfer is not wanted.
 */
void mavlink_bridge_init(mav_bridge_t *br, mission_t *active_mission);

/** Register the GCS-tunable parameter table (may be NULL to disable). */
void mavlink_bridge_set_param_table(mav_bridge_t *br,
                                    const mav_param_entry_t *table,
                                    uint16_t count);

/**
 * Latch or re-latch the local NED origin (Section 2.5).
 *
 * Waypoints arriving from the ground station in degE7 are converted against
 * this origin, and cannot be converted before it exists --- an upload
 * attempted with no home is rejected with MAV_MISSION_DENIED.
 *
 * Calling this again with a different position RE-PROJECTS any loaded
 * mission from the coordinates the ground station originally sent, so the
 * waypoints stay where the operator drew them on the map. That is what makes
 * it safe to follow Section 2.5 and re-latch at arming even though the ground
 * station had to upload the mission earlier: powering on beside the car and
 * arming a kilometre away moves the origin, not the mission.
 *
 * Re-projection only covers waypoints this bridge received. A mission loaded
 * directly by the flight core has no geodetic record to re-project from and
 * is left alone.
 */
void mavlink_bridge_set_home(mav_bridge_t *br,
                             int32_t lat_e7, int32_t lon_e7, int32_t alt_mm_msl);

static inline bool mavlink_bridge_home_valid(const mav_bridge_t *br) {
    return br->home_valid;
}

/**
 * Read back the latched origin.
 *
 * The GPS driver must convert its fixes against exactly this origin. Two
 * copies of "home" that drift apart would put the mission frame and the
 * estimator frame in different places, and the vehicle would fly a
 * consistent, confident, wrong distance from every waypoint.
 *
 * @return false if no origin has been latched yet.
 */
bool mavlink_bridge_get_home(const mav_bridge_t *br,
                             int32_t *out_lat_e7, int32_t *out_lon_e7,
                             int32_t *out_alt_mm_msl);

/* ============================================================================
 * Inbound path --- call for every byte received from the GCS UART
 * ========================================================================= */

/** Feed one received byte. Safe to call from a UART receive interrupt. */
void mavlink_bridge_handle_byte(mav_bridge_t *br, uint8_t byte, uint32_t now_ms);

/** Read and clear accumulated events. Call once per medium-rate tick. */
void mavlink_bridge_poll_events(mav_bridge_t *br, mav_events_t *out);

/**
 * Report the flight core's verdict on the command most recently raised as an
 * event, sending the COMMAND_ACK the GCS is waiting on.
 *
 * @param result MAV_RESULT_ACCEPTED, _DENIED, _TEMPORARILY_REJECTED, ...
 * @param text   Optional STATUSTEXT for the GCS message pane --- pass
 *               arming_result_name() here so a refused arm says *why*.
 *               NULL to send nothing.
 */
void mavlink_bridge_command_result(mav_bridge_t *br, uint8_t result, const char *text);

/* ============================================================================
 * Outbound path --- call from the superloop tiers of Section 3.4
 * ========================================================================= */

/** 1 Hz slow tier: HEARTBEAT + SYS_STATUS. */
void mavlink_bridge_tick_1hz(mav_bridge_t *br, const mav_flight_snapshot_t *s);

/** 10 Hz medium tier: ATTITUDE. */
void mavlink_bridge_tick_10hz(mav_bridge_t *br, const mav_flight_snapshot_t *s);

/** 5 Hz navigation tier: GLOBAL_POSITION_INT + GPS_RAW_INT + VFR_HUD
 *  (+ MISSION_CURRENT while a mission is running). */
void mavlink_bridge_tick_5hz(mav_bridge_t *br, const mav_flight_snapshot_t *s);

/**
 * Housekeeping: mission-transfer retransmits and timeouts, deferred-ACK
 * expiry, and paced parameter streaming. Call at 2 Hz or faster; it does
 * nothing when there is nothing outstanding.
 */
void mavlink_bridge_update(mav_bridge_t *br, uint32_t now_ms);

/** Event notification: a waypoint was accepted by the Section 10.3 sequencer. */
void mavlink_bridge_notify_waypoint_reached(mav_bridge_t *br, uint16_t seq);

/** Push a free-text message to the GCS log pane (failsafes, arming denials). */
void mavlink_bridge_send_statustext(mav_bridge_t *br, uint8_t severity, const char *text);

/* ============================================================================
 * Transmit FIFO --- drain into the UART
 * ========================================================================= */

/** Bytes currently queued for transmission. */
uint16_t mavlink_bridge_tx_pending(const mav_bridge_t *br);

/** Copy up to `max` queued bytes into `out`; returns how many were copied. */
uint16_t mavlink_bridge_tx_read(mav_bridge_t *br, uint8_t *out, uint16_t max);

/* ============================================================================
 * Link health (Section 11.2)
 * ========================================================================= */

/** True if no GCS message has arrived within MAV_GCS_TIMEOUT_MS. */
bool mavlink_bridge_link_lost(const mav_bridge_t *br, uint32_t now_ms);

/** Read-only access to parser statistics for logging. */
static inline const mav_link_stats_t *mavlink_bridge_stats(const mav_bridge_t *br) {
    return &br->parser.stats;
}

/* ============================================================================
 * Local NED <-> geodetic conversion (Section 2.5)
 *
 * Exposed because the GPS driver needs exactly the same transformation when it
 * converts an incoming fix, and having two implementations of it drift apart
 * would silently offset the mission frame from the estimator frame.
 *
 * Flat-earth equirectangular approximation, integer-only:
 *     N = dlat_e7 * 0.0111194927 m
 *     E = dlon_e7 * 0.0111194927 m * cos(lat_home)
 * ========================================================================= */

/** Cosine of a latitude given in degE7, as Q16.16. Uses core CORDIC. */
fx_t mav_cos_lat_e7(int32_t lat_e7);

/** degE7 offsets from home -> local NED metres (Q16.16). */
void mav_lla_to_ned(int32_t lat_e7, int32_t lon_e7,
                    int32_t home_lat_e7, int32_t home_lon_e7,
                    fx_t cos_lat_home,
                    fx_t *out_north, fx_t *out_east);

/** Local NED metres (Q16.16) -> absolute degE7. */
void mav_ned_to_lla(fx_t north, fx_t east,
                    int32_t home_lat_e7, int32_t home_lon_e7,
                    fx_t cos_lat_home,
                    int32_t *out_lat_e7, int32_t *out_lon_e7);

#endif /* MAVLINK_BRIDGE_H */
