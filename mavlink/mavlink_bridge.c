/**
 * mavlink_bridge.c
 *
 * MAVLink Ground-Control Bridge --- implementation.
 *
 * See mavlink_bridge.h for the two invariants this file exists to hold:
 * it never changes flight state, and it never touches hardware.
 */

#include "mavlink_bridge.h"

/* ============================================================================
 * Local NED <-> geodetic conversion (Section 2.5)
 *
 * Constants, all exact to the precision Q16.16 can carry:
 *
 *   metres per degE7 = (pi/180) * 6371000 * 1e-7 = 0.011119492664 m
 *   MAV_M_PER_DEGE7_Q32  = 0.011119492664 * 2^32   (metres -> Q16.16 needs >>16)
 *   MAV_DEGE7_PER_M_Q32  = 89.93204        * 2^16 ... * 2^16  (inverse)
 *   MAV_RAD_PER_DEGE7_Q32= (pi/180) * 1e-7 * 2^32
 *
 * These are the same numbers Section 2.5 derives; they are computed once per
 * GPS fix or once per waypoint, never inside the fast loop.
 * ========================================================================= */

#define MAV_M_PER_DEGE7_Q32     47757863    /* 0.011119492664 * 2^32 */
#define MAV_DEGE7_PER_M_Q32      5893786    /* (1/0.011119492664) * 2^16, then <<16 */
#define MAV_RAD_PER_DEGE7_Q32     491227    /* (pi/180) * 1e-7 * 2^32 */

fx_t mav_cos_lat_e7(int32_t lat_e7) {
    fx_t lat_rad, s, c;

    /* degE7 -> radians in Q16.16, integer-only. */
    lat_rad = (fx_t)(((int64_t)lat_e7 * MAV_RAD_PER_DEGE7_Q32) >> 32);

    cordic_sincos(lat_rad, &s, &c);

    /* Guard the divide in mav_ned_to_lla: |lat| > ~89.9 deg is outside this
     * architecture's flight envelope, but a zero here would be a division
     * fault rather than merely a wrong answer. */
    if (c < 1024) c = 1024;   /* ~0.0156 */
    return c;
}

void mav_lla_to_ned(int32_t lat_e7, int32_t lon_e7,
                    int32_t home_lat_e7, int32_t home_lon_e7,
                    fx_t cos_lat_home,
                    fx_t *out_north, fx_t *out_east) {
    int32_t dlat = lat_e7 - home_lat_e7;
    int32_t dlon = lon_e7 - home_lon_e7;

    fx_t north = (fx_t)(((int64_t)dlat * MAV_M_PER_DEGE7_Q32) >> 16);
    fx_t east  = (fx_t)(((int64_t)dlon * MAV_M_PER_DEGE7_Q32) >> 16);

    /* Longitude degrees shrink with latitude; north does not. */
    east = fx_mul(east, cos_lat_home);

    *out_north = north;
    *out_east  = east;
}

void mav_ned_to_lla(fx_t north, fx_t east,
                    int32_t home_lat_e7, int32_t home_lon_e7,
                    fx_t cos_lat_home,
                    int32_t *out_lat_e7, int32_t *out_lon_e7) {
    fx_t east_equatorial = fx_div(east, cos_lat_home);

    int32_t dlat = (int32_t)(((int64_t)north           * MAV_DEGE7_PER_M_Q32) >> 32);
    int32_t dlon = (int32_t)(((int64_t)east_equatorial * MAV_DEGE7_PER_M_Q32) >> 32);

    *out_lat_e7 = home_lat_e7 + dlat;
    *out_lon_e7 = home_lon_e7 + dlon;
}

/* ============================================================================
 * Transmit FIFO
 *
 * A frame is either queued whole or dropped whole. A half-written frame on the
 * wire would cost the GCS a CRC failure and a resynchronisation, which is a
 * worse outcome than a missed telemetry sample.
 * ========================================================================= */

static uint16_t fifo_used(const mav_bridge_t *br) {
    return (uint16_t)((br->tx_tail - br->tx_head + MAV_TX_FIFO_SIZE) % MAV_TX_FIFO_SIZE);
}

static uint16_t fifo_free(const mav_bridge_t *br) {
    /* One slot is always left empty so head == tail unambiguously means empty. */
    return (uint16_t)(MAV_TX_FIFO_SIZE - 1u - fifo_used(br));
}

static void fifo_push_frame(mav_bridge_t *br, const uint8_t *frame, uint8_t len) {
    uint8_t i;

    if (len == 0) return;

    if (fifo_free(br) < (uint16_t)len) {
        br->tx_dropped_frames++;
        return;
    }
    for (i = 0; i < len; i++) {
        br->tx_fifo[br->tx_tail] = frame[i];
        br->tx_tail = (uint16_t)((br->tx_tail + 1u) % MAV_TX_FIFO_SIZE);
    }
}

uint16_t mavlink_bridge_tx_pending(const mav_bridge_t *br) {
    return fifo_used(br);
}

uint16_t mavlink_bridge_tx_read(mav_bridge_t *br, uint8_t *out, uint16_t max) {
    uint16_t n = fifo_used(br);
    uint16_t i;

    if (n > max) n = max;
    for (i = 0; i < n; i++) {
        out[i] = br->tx_fifo[br->tx_head];
        br->tx_head = (uint16_t)((br->tx_head + 1u) % MAV_TX_FIFO_SIZE);
    }
    return n;
}

/* ============================================================================
 * Lifecycle
 * ========================================================================= */

void mavlink_bridge_init(mav_bridge_t *br, mission_t *active_mission) {
    uint16_t i;

    mav_encoder_init(&br->enc);
    mav_parser_init(&br->parser);

    br->gcs_sysid       = MAV_SYS_ID;
    br->gcs_compid      = MAV_COMP_ID_MISSIONPLANNER;
    br->gcs_seen        = false;
    br->last_gcs_msg_ms = 0;

    br->home_lat_e7    = 0;
    br->home_lon_e7    = 0;
    br->home_alt_mm_msl= 0;
    br->cos_lat_home   = FX_ONE;
    br->home_valid     = false;

    br->xfer_state   = MAV_XFER_IDLE;
    br->xfer_count   = 0;
    br->xfer_seq     = 0;
    br->xfer_last_ms = 0;
    br->xfer_retries = 0;
    br->xfer_peer_sys  = MAV_SYS_ID;
    br->xfer_peer_comp = MAV_COMP_ID_MISSIONPLANNER;
    br->wp_geo_count   = 0;
    mission_clear(&br->staging);

    br->cmd_pending    = false;
    br->cmd_pending_id = 0;
    br->cmd_pending_ms = 0;
    br->cmd_peer_sys   = MAV_SYS_ID;
    br->cmd_peer_comp  = MAV_COMP_ID_MISSIONPLANNER;

    br->params              = 0;
    br->param_count         = 0;
    br->param_stream_active = false;
    br->param_stream_index  = 0;

    br->tx_head           = 0;
    br->tx_tail           = 0;
    br->tx_dropped_frames = 0;
    for (i = 0; i < MAV_TX_FIFO_SIZE; i++) br->tx_fifo[i] = 0;

    br->active_mission = active_mission;

    mavlink_bridge_poll_events(br, 0);   /* zero the event block */
}

void mavlink_bridge_set_param_table(mav_bridge_t *br,
                                    const mav_param_entry_t *table,
                                    uint16_t count) {
    br->params      = table;
    br->param_count = count;
}

/**
 * Re-derive every stored waypoint's NED offset from the coordinates the
 * ground station originally sent, against the current origin.
 *
 * Waypoints live in the flight core as metres from home. Move home without
 * doing this and the whole mission translates by however far home moved,
 * silently and with no error anywhere --- the vehicle flies a perfectly
 * executed mission in the wrong field.
 */
static void reproject_mission(mav_bridge_t *br) {
    uint8_t n, i;

    if (!br->active_mission || !br->home_valid) return;

    n = br->wp_geo_count;
    if (n > br->active_mission->count) n = br->active_mission->count;

    for (i = 0; i < n; i++) {
        fx_t north, east;
        mav_lla_to_ned(br->wp_lat_e7[i], br->wp_lon_e7[i],
                       br->home_lat_e7, br->home_lon_e7, br->cos_lat_home,
                       &north, &east);
        br->active_mission->waypoints[i].north = north;
        br->active_mission->waypoints[i].east  = east;
        /* `down` is already expressed relative to home and stays as drawn:
         * "20 m above home" means the same thing wherever home is. */
    }
}

void mavlink_bridge_set_home(mav_bridge_t *br,
                             int32_t lat_e7, int32_t lon_e7, int32_t alt_mm_msl) {
    bool moved = (!br->home_valid) ||
                 (br->home_lat_e7 != lat_e7) ||
                 (br->home_lon_e7 != lon_e7);

    br->home_lat_e7     = lat_e7;
    br->home_lon_e7     = lon_e7;
    br->home_alt_mm_msl = alt_mm_msl;
    br->cos_lat_home    = mav_cos_lat_e7(lat_e7);
    br->home_valid      = true;

    if (moved) {
        reproject_mission(br);
    }
}

bool mavlink_bridge_get_home(const mav_bridge_t *br,
                             int32_t *out_lat_e7, int32_t *out_lon_e7,
                             int32_t *out_alt_mm_msl) {
    if (!br->home_valid) return false;
    if (out_lat_e7)     *out_lat_e7     = br->home_lat_e7;
    if (out_lon_e7)     *out_lon_e7     = br->home_lon_e7;
    if (out_alt_mm_msl) *out_alt_mm_msl = br->home_alt_mm_msl;
    return true;
}

/* ============================================================================
 * Small helpers
 * ========================================================================= */

static void clear_events(mav_events_t *e) {
    e->arm_request           = false;
    e->arm_value             = false;
    e->mode_request          = false;
    e->mode_value            = MODE_DISARMED;
    e->mission_start_request = false;
    e->rtl_request           = false;
    e->takeoff_request       = false;
    e->takeoff_alt_m         = 0;
    e->mission_uploaded      = false;
    e->mission_cleared       = false;
}

void mavlink_bridge_poll_events(mav_bridge_t *br, mav_events_t *out) {
    if (out) *out = br->events;
    clear_events(&br->events);
}

void mavlink_bridge_send_statustext(mav_bridge_t *br, uint8_t severity, const char *text) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n = mav_encode_statustext(&br->enc, severity, text, frame);
    fifo_push_frame(br, frame, n);
}

void mavlink_bridge_notify_waypoint_reached(mav_bridge_t *br, uint16_t seq) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n = mav_encode_mission_item_reached(&br->enc, seq, frame);
    fifo_push_frame(br, frame, n);
}

bool mavlink_bridge_link_lost(const mav_bridge_t *br, uint32_t now_ms) {
    if (!br->gcs_seen) return true;
    return (uint32_t)(now_ms - br->last_gcs_msg_ms) > MAV_GCS_TIMEOUT_MS;
}

/* ============================================================================
 * Outbound telemetry tiers
 * ========================================================================= */

void mavlink_bridge_tick_1hz(mav_bridge_t *br, const mav_flight_snapshot_t *s) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n;

    n = mav_encode_heartbeat(&br->enc, s->armed, (uint8_t)s->mode,
                             s->sys_state, frame);
    fifo_push_frame(br, frame, n);

    n = mav_encode_sys_status(&br->enc, s->battery_mv, s->battery_pct,
                              s->cpu_load,
                              s->sensors_present, s->sensors_enabled,
                              s->sensors_health, frame);
    fifo_push_frame(br, frame, n);
}

void mavlink_bridge_tick_10hz(mav_bridge_t *br, const mav_flight_snapshot_t *s) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n = mav_encode_attitude(&br->enc, s->time_boot_ms,
                                    s->roll, s->pitch, s->yaw,
                                    s->rollspeed, s->pitchspeed, s->yawspeed,
                                    frame);
    fifo_push_frame(br, frame, n);
}

/**
 * Scale a Q16.16 value by an integer factor and return a plain integer.
 *
 * Done in int64 rather than with fx_mul because the intermediate overflows
 * Q16.16 long before the result overflows int32: an altitude of 100 m in
 * millimetres is 100000, comfortably an int32, but 100000 expressed in Q16.16
 * is 6.55e9 and does not fit at all.
 */
static int32_t fx_scale_i32(fx_t v, int32_t scale) {
    int64_t product = (int64_t)v * (int64_t)scale;
    /* Round the magnitude, then reapply the sign: >> floors rather than
     * truncating, so rounding a negative in place would bias it downward. */
    if (product >= 0) {
        return (int32_t)((product + (1 << 15)) >> 16);
    }
    return (int32_t)(-(((-product) + (1 << 15)) >> 16));
}

/** Yaw in radians (Q16.16) -> heading in centidegrees, wrapped to 0..35999. */
static uint16_t yaw_to_cdeg(fx_t yaw_rad) {
    /* centideg = rad * (18000/pi) = rad * 5729.5779513.
     * That constant does not fit Q16.16, so scale in int64 directly:
     *   cdeg = (yaw * C) >> 32,  C = round(5729.5779513 * 2^16) = 375493622
     * which cancels the Q16.16 scaling of `yaw` in the same shift. */
    int64_t cdeg = ((int64_t)yaw_rad * 375493622LL) >> 32;
    while (cdeg < 0)      cdeg += 36000;
    while (cdeg >= 36000) cdeg -= 36000;
    return (uint16_t)cdeg;
}

void mavlink_bridge_tick_5hz(mav_bridge_t *br, const mav_flight_snapshot_t *s) {
    uint8_t  frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t  n;
    int32_t  lat_e7, lon_e7;
    int32_t  alt_mm_msl, rel_alt_mm;
    uint16_t hdg_cdeg;
    fx_t     groundspeed;

    /* The map icon follows the *estimator*, not the raw receiver: converting
     * the fused NED position back through the same home reference the mission
     * frame uses is what keeps the displayed track and the flown track
     * identical. Before home is latched there is no local frame, so fall back
     * to the raw fix. */
    if (br->home_valid) {
        mav_ned_to_lla(s->pos_n, s->pos_e,
                       br->home_lat_e7, br->home_lon_e7, br->cos_lat_home,
                       &lat_e7, &lon_e7);
        rel_alt_mm = fx_scale_i32(s->altitude_rel, 1000);
        alt_mm_msl = br->home_alt_mm_msl + rel_alt_mm;
    } else {
        lat_e7     = s->gps_lat_e7;
        lon_e7     = s->gps_lon_e7;
        rel_alt_mm = fx_scale_i32(s->altitude_rel, 1000);
        alt_mm_msl = s->gps_alt_mm_msl;
    }

    hdg_cdeg = yaw_to_cdeg(s->yaw);

    n = mav_encode_global_position_int(&br->enc, s->time_boot_ms,
                                       lat_e7, lon_e7, alt_mm_msl, rel_alt_mm,
                                       s->vel_n, s->vel_e, s->vel_d,
                                       hdg_cdeg, frame);
    fifo_push_frame(br, frame, n);

    /* Ground speed = |horizontal velocity|. fx_sqrt of a sum of squares is the
     * same primitive the Section 10.3 sequencer uses for waypoint distance. */
    groundspeed = fx_sqrt(fx_add(fx_mul(s->vel_n, s->vel_n),
                                 fx_mul(s->vel_e, s->vel_e)));

    n = mav_encode_gps_raw_int(&br->enc,
                               (uint64_t)s->time_boot_ms * 1000u,
                               s->gps_lat_e7, s->gps_lon_e7, s->gps_alt_mm_msl,
                               s->gps_fix_type, s->gps_num_sats,
                               s->gps_eph,
                               (uint16_t)fx_scale_i32(groundspeed, 100),
                               hdg_cdeg, frame);
    fifo_push_frame(br, frame, n);

    n = mav_encode_vfr_hud(&br->enc, groundspeed, s->altitude_rel, s->climb_rate,
                           (int16_t)(hdg_cdeg / 100u), s->throttle_pct, frame);
    fifo_push_frame(br, frame, n);

    if (s->mode == MODE_MISSION || s->mode == MODE_RTL) {
        n = mav_encode_mission_current(&br->enc, s->mission_current_seq, frame);
        fifo_push_frame(br, frame, n);
    }
}

/* ============================================================================
 * Mission transfer
 * ========================================================================= */

static void send_mission_request(mav_bridge_t *br, uint16_t seq, uint32_t now_ms) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n = mav_encode_mission_request_int(&br->enc, seq,
                                               br->xfer_peer_sys, br->xfer_peer_comp,
                                               frame);
    fifo_push_frame(br, frame, n);
    br->xfer_last_ms = now_ms;
}

static void send_mission_ack(mav_bridge_t *br, uint8_t result) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n = mav_encode_mission_ack(&br->enc, br->xfer_peer_sys,
                                       br->xfer_peer_comp, result, frame);
    fifo_push_frame(br, frame, n);
}

static void abort_transfer(mav_bridge_t *br, uint8_t result) {
    send_mission_ack(br, result);
    br->xfer_state   = MAV_XFER_IDLE;
    br->xfer_count   = 0;
    br->xfer_seq     = 0;
    br->xfer_retries = 0;
}

/**
 * Convert one MISSION_ITEM_INT into a local-NED waypoint_t (Section 2.5).
 *
 * Only frames whose altitude is relative to home are accepted: an absolute-MSL
 * waypoint would need a survey-grade home elevation this architecture does not
 * have, and silently treating MSL as relative is exactly the class of mistake
 * that flies a vehicle into terrain.
 */
static bool item_to_waypoint(const mav_bridge_t *br,
                             const mav_mission_item_int_t *it,
                             waypoint_t *out) {
    fx_t north, east, alt_m, radius;

    if (it->frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
        it->frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        return false;
    }

    mav_lla_to_ned(it->x, it->y,
                   br->home_lat_e7, br->home_lon_e7, br->cos_lat_home,
                   &north, &east);

    alt_m  = ieee754_to_fx(it->z);
    radius = ieee754_to_fx(it->param2);

    /* A GCS that leaves the acceptance radius at zero gets the architecture's
     * default rather than a waypoint that can never be reached. */
    if (radius < MIN_ACCEPTANCE_RADIUS) radius = DEFAULT_ACCEPTANCE_RADIUS;

    out->north             = north;
    out->east              = east;
    out->down              = fx_neg(alt_m);   /* NED: down is negative altitude */
    out->acceptance_radius = radius;
    return true;
}

static void send_mission_item(mav_bridge_t *br, uint16_t seq) {
    uint8_t  frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t  n;
    int32_t  lat_e7, lon_e7;
    const waypoint_t *wp;

    if (!br->active_mission || seq >= br->active_mission->count) {
        abort_transfer(br, MAV_MISSION_INVALID_SEQUENCE);
        return;
    }

    wp = &br->active_mission->waypoints[seq];

    /* Prefer the coordinates the ground station sent over re-deriving them
     * from NED: the round trip through metres and back costs a few
     * centimetres, and a station that compares an uploaded mission against the
     * one it reads back would flag that as a mismatch. */
    if (seq < br->wp_geo_count) {
        lat_e7 = br->wp_lat_e7[seq];
        lon_e7 = br->wp_lon_e7[seq];
    } else {
        mav_ned_to_lla(wp->north, wp->east,
                       br->home_lat_e7, br->home_lon_e7, br->cos_lat_home,
                       &lat_e7, &lon_e7);
    }

    n = mav_encode_mission_item_int(&br->enc, seq,
                                    (uint16_t)MAV_CMD_NAV_WAYPOINT,
                                    (uint8_t)MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    (uint8_t)((seq == br->active_mission->current_index) ? 1 : 0),
                                    1,                       /* autocontinue */
                                    lat_e7, lon_e7,
                                    fx_neg(wp->down),        /* down -> altitude */
                                    wp->acceptance_radius,
                                    br->xfer_peer_sys, br->xfer_peer_comp,
                                    frame);
    fifo_push_frame(br, frame, n);
}

/* ============================================================================
 * Inbound message handling
 * ========================================================================= */

static void handle_command_long(mav_bridge_t *br, const mav_message_t *m, uint32_t now_ms) {
    mav_command_long_t cmd;
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n;

    if (!mav_msg_command_long(m, &cmd)) return;

    /* Only one command may be outstanding: the flight core answers them one at
     * a time, and a second pending ACK would be attributed to the wrong one. */
    if (br->cmd_pending) {
        n = mav_encode_command_ack(&br->enc, cmd.command,
                                   (uint8_t)MAV_RESULT_TEMPORARILY_REJECTED, frame);
        fifo_push_frame(br, frame, n);
        return;
    }

    switch (cmd.command) {

    case MAV_CMD_COMPONENT_ARM_DISARM:
        br->events.arm_request = true;
        /* param1: 1.0 = arm, 0.0 = disarm. Compare in Q16.16 against 0.5 so a
         * GCS sending 0.999999 still reads as "arm". */
        br->events.arm_value = (ieee754_to_fx(cmd.param1) > (FX_ONE / 2));
        break;

    case MAV_CMD_DO_SET_MODE:
        br->events.mode_request = true;
        br->events.mode_value   = (flight_mode_t)fx_to_int(ieee754_to_fx(cmd.param2));
        break;

    case MAV_CMD_MISSION_START:
        br->events.mission_start_request = true;
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        br->events.rtl_request = true;
        break;

    case MAV_CMD_NAV_TAKEOFF:
        br->events.takeoff_request = true;
        br->events.takeoff_alt_m   = ieee754_to_fx(cmd.param7);
        break;

#ifdef MAV_ARDUPILOT_COMPAT
    case MAV_CMD_DO_SET_HOME:
        /* A ground station in ArduCopter mode periodically pushes home to the
         * current position. This firmware latches home itself at arming
         * (Section 2.5) and does not honour the override --- but answering
         * UNSUPPORTED makes the GCS pop a "can't update home" warning every
         * few seconds. Accept-and-ignore keeps the UI quiet without moving the
         * flight frame. Only compiled in the compat build; the honest board
         * build still reports UNSUPPORTED, which is the truth. */
        n = mav_encode_command_ack(&br->enc, cmd.command,
                                   (uint8_t)MAV_RESULT_ACCEPTED, frame);
        fifo_push_frame(br, frame, n);
        return;
#endif

    default:
        /* Unsupported commands are answered immediately; the flight core is
         * never told about them. */
        n = mav_encode_command_ack(&br->enc, cmd.command,
                                   (uint8_t)MAV_RESULT_UNSUPPORTED, frame);
        fifo_push_frame(br, frame, n);
        return;
    }

    /* Defer the ACK until the flight core rules on it. */
    br->cmd_pending    = true;
    br->cmd_pending_id = cmd.command;
    br->cmd_pending_ms = now_ms;
    br->cmd_peer_sys   = m->sysid;
    br->cmd_peer_comp  = m->compid;
}

static void handle_mission_count(mav_bridge_t *br, const mav_message_t *m, uint32_t now_ms) {
    mav_mission_count_t mc;

    if (!mav_msg_mission_count(m, &mc)) return;

    br->xfer_peer_sys  = m->sysid;
    br->xfer_peer_comp = m->compid;

    /* Count zero is the GCS's way of saying "delete the mission". */
    if (mc.count == 0) {
        if (br->active_mission) mission_clear(br->active_mission);
        mission_clear(&br->staging);
        br->wp_geo_count = 0;
        br->events.mission_cleared = true;
        br->xfer_state = MAV_XFER_IDLE;
        send_mission_ack(br, (uint8_t)MAV_MISSION_ACCEPTED);
        return;
    }

    if (mc.count > MAX_MISSION_WAYPOINTS) {
        br->xfer_state = MAV_XFER_IDLE;
        send_mission_ack(br, (uint8_t)MAV_MISSION_NO_SPACE);
        return;
    }

    /* Waypoints arrive in degE7 and are stored in local NED. With no home
     * latched there is no local frame to convert into, so refuse rather than
     * store waypoints against an origin of (0,0). */
    if (!br->home_valid) {
        br->xfer_state = MAV_XFER_IDLE;
        send_mission_ack(br, (uint8_t)MAV_MISSION_DENIED);
        return;
    }

    mission_clear(&br->staging);
    br->xfer_state   = MAV_XFER_UPLOAD;
    br->xfer_count   = mc.count;
    br->xfer_seq     = 0;
    br->xfer_retries = 0;
    send_mission_request(br, 0, now_ms);
}

static void handle_mission_item(mav_bridge_t *br, const mav_message_t *m, uint32_t now_ms) {
    mav_mission_item_int_t it;
    waypoint_t wp;

    if (!mav_msg_mission_item_int(m, &it)) return;
    if (br->xfer_state != MAV_XFER_UPLOAD) return;

    /* A duplicate of the item we already stored means our request was lost on
     * the way out; re-request the one we are actually waiting for. */
    if (it.seq + 1u == br->xfer_seq) {
        send_mission_request(br, br->xfer_seq, now_ms);
        return;
    }

    if (it.seq != br->xfer_seq) {
        abort_transfer(br, (uint8_t)MAV_MISSION_INVALID_SEQUENCE);
        return;
    }

    if (!item_to_waypoint(br, &it, &wp)) {
        abort_transfer(br, (uint8_t)MAV_MISSION_UNSUPPORTED_FRAME);
        return;
    }

    br->staging.waypoints[br->xfer_seq] = wp;
    /* Keep what the ground station actually sent, so the mission can be
     * re-projected if home is re-latched at arming. */
    br->wp_lat_e7[br->xfer_seq] = it.x;
    br->wp_lon_e7[br->xfer_seq] = it.y;
    br->xfer_seq++;
    br->xfer_retries = 0;

    if (br->xfer_seq < br->xfer_count) {
        send_mission_request(br, br->xfer_seq, now_ms);
        return;
    }

    /* Complete. Only now is the live mission touched --- a transfer that dies
     * halfway leaves the previously loaded mission intact, which matters if
     * the GCS drops out while the vehicle is already flying one. */
    br->staging.count         = (uint8_t)br->xfer_count;
    br->staging.current_index = 0;
    br->staging.loop          = false;

    if (br->active_mission) {
        mission_load(br->active_mission, br->staging.waypoints,
                     (uint8_t)br->xfer_count, false);
    }

    br->wp_geo_count = (uint8_t)br->xfer_count;

    br->xfer_state = MAV_XFER_IDLE;
    br->events.mission_uploaded = true;
    send_mission_ack(br, (uint8_t)MAV_MISSION_ACCEPTED);
}

static void handle_mission_request_list(mav_bridge_t *br, const mav_message_t *m,
                                        uint32_t now_ms) {
    mav_mission_request_list_t rl;
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n;
    uint16_t count;

    if (!mav_msg_mission_request_list(m, &rl)) return;

    br->xfer_peer_sys  = m->sysid;
    br->xfer_peer_comp = m->compid;

    count = br->active_mission ? (uint16_t)br->active_mission->count : 0u;

    n = mav_encode_mission_count(&br->enc, count,
                                 br->xfer_peer_sys, br->xfer_peer_comp, frame);
    fifo_push_frame(br, frame, n);

    if (count > 0) {
        br->xfer_state   = MAV_XFER_DOWNLOAD;
        br->xfer_count   = count;
        br->xfer_seq     = 0;
        br->xfer_retries = 0;
        br->xfer_last_ms = now_ms;
    } else {
        br->xfer_state = MAV_XFER_IDLE;
    }
}

static void handle_mission_request_int(mav_bridge_t *br, const mav_message_t *m,
                                       uint32_t now_ms) {
    mav_mission_request_int_t rq;

    if (!mav_msg_mission_request_int(m, &rq)) return;

    br->xfer_peer_sys  = m->sysid;
    br->xfer_peer_comp = m->compid;
    br->xfer_last_ms   = now_ms;
    br->xfer_seq       = rq.seq;

    send_mission_item(br, rq.seq);
}

static void handle_mission_clear_all(mav_bridge_t *br, const mav_message_t *m) {
    mav_mission_clear_all_t ca;

    if (!mav_msg_mission_clear_all(m, &ca)) return;

    br->xfer_peer_sys  = m->sysid;
    br->xfer_peer_comp = m->compid;

    if (br->active_mission) mission_clear(br->active_mission);
    mission_clear(&br->staging);
    br->wp_geo_count = 0;
    br->xfer_state = MAV_XFER_IDLE;
    br->events.mission_cleared = true;
    send_mission_ack(br, (uint8_t)MAV_MISSION_ACCEPTED);
}

static void handle_set_mode(mav_bridge_t *br, const mav_message_t *m) {
    mav_set_mode_t sm;

    if (!mav_msg_set_mode(m, &sm)) return;
    if ((sm.base_mode & (uint8_t)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) == 0) return;

    /* SET_MODE carries no ACK in the protocol, so this raises the event and
     * says nothing further; the GCS learns the result from the next HEARTBEAT. */
    br->events.mode_request = true;
    br->events.mode_value   = (flight_mode_t)sm.custom_mode;
}

static void handle_param_set(mav_bridge_t *br, const mav_message_t *m) {
    mav_param_set_t ps;
    uint8_t  frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t  n;
    uint16_t i;
    uint8_t  j;

    if (!mav_msg_param_set(m, &ps)) return;
    if (!br->params) return;

    for (i = 0; i < br->param_count; i++) {
        bool match = true;
        for (j = 0; j < 16u; j++) {
            char a = br->params[i].name[j];
            char b = ps.param_id[j];
            if (a != b) { match = false; break; }
            if (a == '\0') break;      /* both terminated at the same place */
        }
        if (!match) continue;

        *(br->params[i].value) = ieee754_to_fx(ps.param_value);

        /* Echo the stored value back: the GCS compares it against what it
         * sent and reports a mismatch, so this is the write confirmation. */
        n = mav_encode_param_value(&br->enc, br->params[i].name,
                                   *(br->params[i].value),
                                   (uint8_t)MAV_PARAM_TYPE_REAL32,
                                   i, br->param_count, frame);
        fifo_push_frame(br, frame, n);
        return;
    }
}

static void dispatch_message(mav_bridge_t *br, const mav_message_t *m, uint32_t now_ms) {
    switch (m->msg_id) {

    case MAVLINK_MSG_ID_HEARTBEAT:
        /* Nothing to do beyond the liveness bookkeeping already done by the
         * caller --- but this is the message that keeps the Section 11.2
         * ground-link timer fed. */
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(br, m, now_ms);
        break;

    case MAVLINK_MSG_ID_SET_MODE:
        handle_set_mode(br, m);
        break;

    case MAVLINK_MSG_ID_MISSION_COUNT:
        handle_mission_count(br, m, now_ms);
        break;

    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        handle_mission_item(br, m, now_ms);
        break;

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        handle_mission_request_list(br, m, now_ms);
        break;

    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        handle_mission_request_int(br, m, now_ms);
        break;

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        handle_mission_clear_all(br, m);
        break;

    case MAVLINK_MSG_ID_MISSION_ACK:
        /* Closes a download we were serving. */
        if (br->xfer_state == MAV_XFER_DOWNLOAD) br->xfer_state = MAV_XFER_IDLE;
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        if (br->params && br->param_count > 0) {
            br->param_stream_active = true;
            br->param_stream_index  = 0;
        }
        break;

    case MAVLINK_MSG_ID_PARAM_SET:
        handle_param_set(br, m);
        break;

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        /* Rates are fixed by the Section 3.4 superloop tiers, which cannot be
         * renegotiated at run time. Accepting the message silently is correct;
         * the GCS treats absence of a refusal as "rates already suitable". */
        break;

    default:
        break;
    }
}

void mavlink_bridge_handle_byte(mav_bridge_t *br, uint8_t byte, uint32_t now_ms) {
    mav_message_t msg;

    if (!mav_parse_byte(&br->parser, byte, &msg)) return;

    /* Learn who we are talking to from the first valid frame, and keep the
     * ground-link liveness timer fed from every one after that. */
    if (!br->gcs_seen) {
        br->gcs_sysid  = msg.sysid;
        br->gcs_compid = msg.compid;
        br->gcs_seen   = true;
    }
    br->last_gcs_msg_ms = now_ms;

    dispatch_message(br, &msg, now_ms);
}

/* ============================================================================
 * Deferred command result
 * ========================================================================= */

void mavlink_bridge_command_result(mav_bridge_t *br, uint8_t result, const char *text) {
    uint8_t frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t n;

    if (!br->cmd_pending) return;

    n = mav_encode_command_ack(&br->enc, br->cmd_pending_id, result, frame);
    fifo_push_frame(br, frame, n);

    if (text) {
        uint8_t severity = (result == (uint8_t)MAV_RESULT_ACCEPTED)
                         ? (uint8_t)MAV_SEVERITY_INFO
                         : (uint8_t)MAV_SEVERITY_WARNING;
        mavlink_bridge_send_statustext(br, severity, text);
    }

    br->cmd_pending = false;
}

/* ============================================================================
 * Housekeeping
 * ========================================================================= */

void mavlink_bridge_update(mav_bridge_t *br, uint32_t now_ms) {
    uint8_t  frame[MAVLINK_MAX_FRAME_LEN];
    uint8_t  n;
    uint16_t sent;

    /* --- Deferred ACK expiry ------------------------------------------- */
    if (br->cmd_pending &&
        (uint32_t)(now_ms - br->cmd_pending_ms) > MAV_CMD_ACK_TIMEOUT_MS) {
        n = mav_encode_command_ack(&br->enc, br->cmd_pending_id,
                                   (uint8_t)MAV_RESULT_TEMPORARILY_REJECTED, frame);
        fifo_push_frame(br, frame, n);
        br->cmd_pending = false;
    }

    /* --- Mission upload retransmit -------------------------------------- */
    if (br->xfer_state == MAV_XFER_UPLOAD &&
        (uint32_t)(now_ms - br->xfer_last_ms) > MAV_MISSION_RETRY_MS) {
        if (br->xfer_retries >= MAV_MISSION_MAX_RETRIES) {
            /* Give up. The staged mission is discarded; whatever the vehicle
             * was already holding is untouched. */
            mission_clear(&br->staging);
            abort_transfer(br, (uint8_t)MAV_MISSION_OPERATION_CANCELLED);
        } else {
            br->xfer_retries++;
            send_mission_request(br, br->xfer_seq, now_ms);
        }
    }

    /* --- Mission download stall ----------------------------------------- */
    if (br->xfer_state == MAV_XFER_DOWNLOAD &&
        (uint32_t)(now_ms - br->xfer_last_ms) > (MAV_MISSION_RETRY_MS * 4u)) {
        /* The GCS stopped asking. Nothing to retransmit --- it drives this
         * direction --- so just release the transfer slot. */
        br->xfer_state = MAV_XFER_IDLE;
    }

    /* --- Parameter list streaming --------------------------------------- */
    if (br->param_stream_active && br->params) {
        for (sent = 0; sent < MAV_PARAMS_PER_UPDATE; sent++) {
            if (br->param_stream_index >= br->param_count) {
                br->param_stream_active = false;
                break;
            }
            n = mav_encode_param_value(&br->enc,
                                       br->params[br->param_stream_index].name,
                                       *(br->params[br->param_stream_index].value),
                                       (uint8_t)MAV_PARAM_TYPE_REAL32,
                                       br->param_stream_index,
                                       br->param_count, frame);
            fifo_push_frame(br, frame, n);
            br->param_stream_index++;
        }
    }
}
