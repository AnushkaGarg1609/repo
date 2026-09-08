/**
 * mavlink_encode.c
 *
 * MAVLink v2 Message Encoder --- implementation.
 *
 * Every function here is a pure byte-shuffler: it reads flight state, writes a
 * frame, and touches no hardware. The only arithmetic beyond copying is the
 * Q16.16 -> IEEE-754 conversion in mavlink_types.h and a handful of integer
 * scale factors (m/s -> cm/s, and so on).
 */

#include "mavlink_encode.h"

/* ============================================================================
 * Local primitives --- avoids a <string.h> dependency on the target
 * ========================================================================= */

static void mav_memcpy(void *dst, const void *src, uint8_t n) {
    uint8_t       *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    uint8_t        i;
    for (i = 0; i < n; i++) d[i] = s[i];
}

static void mav_memset(void *dst, uint8_t v, uint8_t n) {
    uint8_t *d = (uint8_t *)dst;
    uint8_t  i;
    for (i = 0; i < n; i++) d[i] = v;
}

/**
 * MAVLink v2 payload truncation: drop trailing zero bytes.
 *
 * The receiver zero-pads back to the message's known length, so this is
 * lossless. It typically saves 10-20 bytes per frame, which on a 57600 baud
 * SiK link is the difference between the 10 Hz ATTITUDE stream fitting in the
 * budget and not. At least one byte is always kept, per the spec.
 */
static uint8_t mav_trim_payload(const uint8_t *payload, uint8_t len) {
    while (len > 1u && payload[len - 1u] == 0u) {
        len--;
    }
    return len;
}

/* Round-half-away-from-zero conversion of a Q16.16 value scaled by `scale`,
 * used for the integer wire fields (cm/s, mm, centidegrees). Kept in int64 so
 * that a large altitude in millimetres cannot overflow mid-computation.
 *
 * The negative branch cannot simply subtract the rounding term and shift: >>
 * on a negative value floors rather than truncating, so -125.0 would come out
 * as -126. Magnitude is rounded first, then the sign is reapplied. */
static int32_t fx_scale_to_int(fx_t v, int32_t scale) {
    int64_t product = (int64_t)v * (int64_t)scale;
    if (product >= 0) {
        return (int32_t)((product + (1 << 15)) >> 16);
    }
    return (int32_t)(-((( -product) + (1 << 15)) >> 16));
}

static int16_t clamp_i16(int32_t v) {
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

/* ============================================================================
 * Encoder lifecycle
 * ========================================================================= */

void mav_encoder_init(mav_encoder_t *enc) {
    enc->seq    = 0;
    enc->sysid  = MAV_SYS_ID;
    enc->compid = MAV_COMP_ID_AUTOPILOT;
}

void mav_encoder_set_identity(mav_encoder_t *enc, uint8_t sysid, uint8_t compid) {
    enc->sysid  = sysid;
    enc->compid = compid;
}

/* ============================================================================
 * Frame builder
 *
 * Layout written:
 *   [0]     STX  0xFD
 *   [1]     LEN  (truncated payload length)
 *   [2]     INCOMPAT_FLAGS (0 --- we never sign)
 *   [3]     COMPAT_FLAGS   (0)
 *   [4]     SEQ
 *   [5]     SYSID
 *   [6]     COMPID
 *   [7..9]  MSGID, 24-bit little-endian
 *   [10..]  PAYLOAD
 *   [..+2]  CRC16, little-endian, over bytes [1..end-of-payload] + CRC_EXTRA
 * ========================================================================= */

uint8_t mav_build_frame(mav_encoder_t *enc,
                        uint32_t    msg_id,
                        uint8_t     crc_extra,
                        const void *payload,
                        uint8_t     payload_len,
                        uint8_t    *out_buf) {
    uint8_t  trimmed;
    uint16_t crc;
    uint8_t  total;

    trimmed = mav_trim_payload((const uint8_t *)payload, payload_len);

    out_buf[0] = MAVLINK_V2_STX;
    out_buf[1] = trimmed;
    out_buf[2] = 0;                 /* incompat_flags: unsigned frame */
    out_buf[3] = 0;                 /* compat_flags */
    out_buf[4] = enc->seq++;        /* wraps naturally at 256 */
    out_buf[5] = enc->sysid;
    out_buf[6] = enc->compid;
    msgid_pack(&out_buf[7], msg_id);

    mav_memcpy(&out_buf[MAVLINK_NUM_HEADER_BYTES], payload, trimmed);

    /* CRC covers everything after STX, up to and including the payload. */
    crc = crc16_compute(&out_buf[1],
                        (size_t)(MAVLINK_NUM_HEADER_BYTES - 1 + trimmed),
                        crc_extra);

    total = (uint8_t)(MAVLINK_NUM_HEADER_BYTES + trimmed);
    out_buf[total]     = (uint8_t)(crc & 0xFF);
    out_buf[total + 1] = (uint8_t)(crc >> 8);

    return (uint8_t)(total + MAVLINK_NUM_CRC_BYTES);
}

/* ============================================================================
 * Telemetry encoders
 * ========================================================================= */

#ifdef MAV_ARDUPILOT_COMPAT
/* Map this firmware's flight_mode_t onto ArduCopter's custom_mode numbers.
 *
 *   this firmware        ArduCopter (name shown by the GCS)
 *   -----------------    ---------------------------------
 *   0 DISARMED       ->  0  STABILIZE
 *   1 STABILIZE      ->  0  STABILIZE
 *   2 ALT_HOLD       ->  2  ALT_HOLD
 *   3 POS_HOLD       -> 16  POSHOLD
 *   4 MISSION        ->  3  AUTO
 *   5 RTL            ->  6  RTL
 */
static uint32_t mav_to_ardupilot_mode(uint8_t flight_mode) {
    switch (flight_mode) {
    case 2:  return 2u;   /* ALT_HOLD */
    case 3:  return 16u;  /* POSHOLD  */
    case 4:  return 3u;   /* AUTO     */
    case 5:  return 6u;   /* RTL      */
    case 0:  /* DISARMED  -> STABILIZE */
    case 1:  /* STABILIZE -> STABILIZE */
    default: return 0u;
    }
}
#endif

uint8_t mav_encode_heartbeat(mav_encoder_t *enc,
                             bool     armed,
                             uint8_t  flight_mode,
                             uint8_t  sys_state,
                             uint8_t *out_buf) {
    mav_heartbeat_t hb;
    uint8_t base_mode;

    /* CUSTOM_MODE_ENABLED is what tells the GCS to read custom_mode at all;
     * without it Mission Planner ignores the field and shows "Unknown". */
    base_mode = (uint8_t)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    if (armed) base_mode |= (uint8_t)MAV_MODE_FLAG_SAFETY_ARMED;

    /* Advertise the autonomy level of the current mode, mirroring the
     * Section 9.1 taxonomy. Manual input is always enabled because the
     * Override Supremacy Rule (Section 9.2) never disables the sticks. */
    base_mode |= (uint8_t)MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (flight_mode >= 1u) base_mode |= (uint8_t)MAV_MODE_FLAG_STABILIZE_ENABLED;
    if (flight_mode >= 3u) base_mode |= (uint8_t)MAV_MODE_FLAG_GUIDED_ENABLED;
    if (flight_mode >= 4u) base_mode |= (uint8_t)MAV_MODE_FLAG_AUTO_ENABLED;

#ifdef MAV_ARDUPILOT_COMPAT
    /* Impersonate ArduCopter so a ground station offers its full copter UI.
     * The flight controller's own flight_mode_t is remapped onto ArduCopter's
     * custom_mode numbers; the ground station then labels each with the right
     * name and enables the matching actions. Modes with no exact ArduCopter
     * equivalent map to the nearest one (DISARMED shows as STABILIZE --- the
     * armed flag, not the mode, conveys disarmed). */
    hb.custom_mode     = (uint32_t)mav_to_ardupilot_mode(flight_mode);
    hb.autopilot       = (uint8_t)MAV_AUTOPILOT_ARDUPILOTMEGA;
#else
    hb.custom_mode     = (uint32_t)flight_mode;
    hb.autopilot       = (uint8_t)MAV_AUTOPILOT_GENERIC;
#endif
    hb.type            = (uint8_t)MAV_TYPE_QUADROTOR;
    hb.base_mode       = base_mode;
    hb.system_status   = sys_state;
    hb.mavlink_version = MAVLINK_VERSION_FIELD;

    return mav_build_frame(enc, MAVLINK_MSG_ID_HEARTBEAT,
                           MAVLINK_CRC_EXTRA_HEARTBEAT,
                           &hb, MAVLINK_MSG_LEN_HEARTBEAT, out_buf);
}

uint8_t mav_encode_sys_status(mav_encoder_t *enc,
                              uint16_t battery_mv,
                              int8_t   battery_pct,
                              uint16_t cpu_load,
                              uint32_t sensors_present,
                              uint32_t sensors_enabled,
                              uint32_t sensors_health,
                              uint8_t *out_buf) {
    mav_sys_status_t st;

    mav_memset(&st, 0, (uint8_t)sizeof(st));
    st.sensors_present   = sensors_present;
    st.sensors_enabled   = sensors_enabled;
    st.sensors_health    = sensors_health;
    st.load              = cpu_load;
    st.voltage_battery   = battery_mv;
    st.current_battery   = -1;              /* no current sensor assumed */
    st.battery_remaining = battery_pct;

    return mav_build_frame(enc, MAVLINK_MSG_ID_SYS_STATUS,
                           MAVLINK_CRC_EXTRA_SYS_STATUS,
                           &st, MAVLINK_MSG_LEN_SYS_STATUS, out_buf);
}

uint8_t mav_encode_attitude(mav_encoder_t *enc,
                            uint32_t time_boot_ms,
                            fx_t roll,      fx_t pitch,      fx_t yaw,
                            fx_t rollspeed, fx_t pitchspeed, fx_t yawspeed,
                            uint8_t *out_buf) {
    mav_attitude_t att;

    att.time_boot_ms = time_boot_ms;
    att.roll         = fx_to_ieee754(roll);
    att.pitch        = fx_to_ieee754(pitch);
    att.yaw          = fx_to_ieee754(yaw);
    att.rollspeed    = fx_to_ieee754(rollspeed);
    att.pitchspeed   = fx_to_ieee754(pitchspeed);
    att.yawspeed     = fx_to_ieee754(yawspeed);

    return mav_build_frame(enc, MAVLINK_MSG_ID_ATTITUDE,
                           MAVLINK_CRC_EXTRA_ATTITUDE,
                           &att, MAVLINK_MSG_LEN_ATTITUDE, out_buf);
}

uint8_t mav_encode_global_position_int(mav_encoder_t *enc,
                                       uint32_t time_boot_ms,
                                       int32_t  lat_e7, int32_t lon_e7,
                                       int32_t  alt_mm_msl, int32_t rel_alt_mm,
                                       fx_t     vn, fx_t ve, fx_t vd,
                                       uint16_t hdg_cdeg,
                                       uint8_t *out_buf) {
    mav_global_position_int_t pos;

    pos.time_boot_ms = time_boot_ms;
    pos.lat          = lat_e7;
    pos.lon          = lon_e7;
    pos.alt          = alt_mm_msl;
    pos.relative_alt = rel_alt_mm;
    pos.vx           = clamp_i16(fx_scale_to_int(vn, 100));  /* m/s -> cm/s */
    pos.vy           = clamp_i16(fx_scale_to_int(ve, 100));
    pos.vz           = clamp_i16(fx_scale_to_int(vd, 100));
    pos.hdg          = hdg_cdeg;

    return mav_build_frame(enc, MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                           MAVLINK_CRC_EXTRA_GLOBAL_POSITION_INT,
                           &pos, MAVLINK_MSG_LEN_GLOBAL_POSITION_INT, out_buf);
}

uint8_t mav_encode_gps_raw_int(mav_encoder_t *enc,
                               uint64_t time_usec,
                               int32_t  lat_e7, int32_t lon_e7, int32_t alt_mm_msl,
                               uint8_t  fix_type, uint8_t satellites_visible,
                               uint16_t eph, uint16_t vel_cms, uint16_t cog_cdeg,
                               uint8_t *out_buf) {
    mav_gps_raw_int_t gps;

    gps.time_usec          = time_usec;
    gps.lat                = lat_e7;
    gps.lon                = lon_e7;
    gps.alt                = alt_mm_msl;
    gps.eph                = eph;
    gps.epv                = 0xFFFF;        /* VDOP not tracked */
    gps.vel                = vel_cms;
    gps.cog                = cog_cdeg;
    gps.fix_type           = fix_type;
    gps.satellites_visible = satellites_visible;

    return mav_build_frame(enc, MAVLINK_MSG_ID_GPS_RAW_INT,
                           MAVLINK_CRC_EXTRA_GPS_RAW_INT,
                           &gps, MAVLINK_MSG_LEN_GPS_RAW_INT, out_buf);
}

uint8_t mav_encode_vfr_hud(mav_encoder_t *enc,
                           fx_t     groundspeed,
                           fx_t     alt,
                           fx_t     climb,
                           int16_t  heading_deg,
                           uint16_t throttle_pct,
                           uint8_t *out_buf) {
    mav_vfr_hud_t hud;

    hud.airspeed    = fx_to_ieee754(groundspeed);  /* no pitot: mirror ground speed */
    hud.groundspeed = fx_to_ieee754(groundspeed);
    hud.alt         = fx_to_ieee754(alt);
    hud.climb       = fx_to_ieee754(climb);
    hud.heading     = heading_deg;
    hud.throttle    = throttle_pct;

    return mav_build_frame(enc, MAVLINK_MSG_ID_VFR_HUD,
                           MAVLINK_CRC_EXTRA_VFR_HUD,
                           &hud, MAVLINK_MSG_LEN_VFR_HUD, out_buf);
}

uint8_t mav_encode_statustext(mav_encoder_t *enc,
                              uint8_t     severity,
                              const char *text,
                              uint8_t    *out_buf) {
    mav_statustext_t st;
    uint8_t i;

    mav_memset(&st, 0, (uint8_t)sizeof(st));
    st.severity = severity;
    for (i = 0; i < 50u && text[i] != '\0'; i++) {
        st.text[i] = text[i];
    }

    return mav_build_frame(enc, MAVLINK_MSG_ID_STATUSTEXT,
                           MAVLINK_CRC_EXTRA_STATUSTEXT,
                           &st, MAVLINK_MSG_LEN_STATUSTEXT, out_buf);
}

/* ============================================================================
 * Protocol responses
 * ========================================================================= */

uint8_t mav_encode_command_ack(mav_encoder_t *enc,
                               uint16_t command,
                               uint8_t  result,
                               uint8_t *out_buf) {
    mav_command_ack_t ack;

    ack.command = command;
    ack.result  = result;

    return mav_build_frame(enc, MAVLINK_MSG_ID_COMMAND_ACK,
                           MAVLINK_CRC_EXTRA_COMMAND_ACK,
                           &ack, MAVLINK_MSG_LEN_COMMAND_ACK, out_buf);
}

uint8_t mav_encode_mission_count(mav_encoder_t *enc,
                                 uint16_t count,
                                 uint8_t  target_sys,
                                 uint8_t  target_comp,
                                 uint8_t *out_buf) {
    mav_mission_count_t mc;

    mc.count            = count;
    mc.target_system    = target_sys;
    mc.target_component = target_comp;

    return mav_build_frame(enc, MAVLINK_MSG_ID_MISSION_COUNT,
                           MAVLINK_CRC_EXTRA_MISSION_COUNT,
                           &mc, MAVLINK_MSG_LEN_MISSION_COUNT, out_buf);
}

uint8_t mav_encode_mission_request_int(mav_encoder_t *enc,
                                       uint16_t seq,
                                       uint8_t  target_sys,
                                       uint8_t  target_comp,
                                       uint8_t *out_buf) {
    mav_mission_request_int_t rq;

    rq.seq              = seq;
    rq.target_system    = target_sys;
    rq.target_component = target_comp;

    return mav_build_frame(enc, MAVLINK_MSG_ID_MISSION_REQUEST_INT,
                           MAVLINK_CRC_EXTRA_MISSION_REQUEST_INT,
                           &rq, MAVLINK_MSG_LEN_MISSION_REQUEST_INT, out_buf);
}

uint8_t mav_encode_mission_item_int(mav_encoder_t *enc,
                                    uint16_t seq,
                                    uint16_t command,
                                    uint8_t  frame,
                                    uint8_t  current,
                                    uint8_t  autocontinue,
                                    int32_t  lat_e7, int32_t lon_e7,
                                    fx_t     alt_m,
                                    fx_t     accept_radius,
                                    uint8_t  target_sys, uint8_t target_comp,
                                    uint8_t *out_buf) {
    mav_mission_item_int_t it;

    it.param1           = MAV_IEEE_ZERO;                   /* hold time: 0 s */
    it.param2           = fx_to_ieee754(accept_radius);    /* acceptance radius */
    it.param3           = MAV_IEEE_ZERO;                   /* pass-through radius */
    it.param4           = MAV_IEEE_ZERO;                   /* yaw: unconstrained */
    it.x                = lat_e7;
    it.y                = lon_e7;
    it.z                = fx_to_ieee754(alt_m);
    it.seq              = seq;
    it.command          = command;
    it.target_system    = target_sys;
    it.target_component = target_comp;
    it.frame            = frame;
    it.current          = current;
    it.autocontinue     = autocontinue;

    return mav_build_frame(enc, MAVLINK_MSG_ID_MISSION_ITEM_INT,
                           MAVLINK_CRC_EXTRA_MISSION_ITEM_INT,
                           &it, MAVLINK_MSG_LEN_MISSION_ITEM_INT, out_buf);
}

uint8_t mav_encode_mission_ack(mav_encoder_t *enc,
                               uint8_t  target_sys,
                               uint8_t  target_comp,
                               uint8_t  result,
                               uint8_t *out_buf) {
    mav_mission_ack_t ack;

    ack.target_system    = target_sys;
    ack.target_component = target_comp;
    ack.type             = result;

    return mav_build_frame(enc, MAVLINK_MSG_ID_MISSION_ACK,
                           MAVLINK_CRC_EXTRA_MISSION_ACK,
                           &ack, MAVLINK_MSG_LEN_MISSION_ACK, out_buf);
}

uint8_t mav_encode_mission_current(mav_encoder_t *enc,
                                   uint16_t seq,
                                   uint8_t *out_buf) {
    mav_mission_current_t mc;

    mc.seq = seq;

    return mav_build_frame(enc, MAVLINK_MSG_ID_MISSION_CURRENT,
                           MAVLINK_CRC_EXTRA_MISSION_CURRENT,
                           &mc, MAVLINK_MSG_LEN_MISSION_CURRENT, out_buf);
}

uint8_t mav_encode_mission_item_reached(mav_encoder_t *enc,
                                        uint16_t seq,
                                        uint8_t *out_buf) {
    mav_mission_item_reached_t r;

    r.seq = seq;

    return mav_build_frame(enc, MAVLINK_MSG_ID_MISSION_ITEM_REACHED,
                           MAVLINK_CRC_EXTRA_MISSION_ITEM_REACHED,
                           &r, MAVLINK_MSG_LEN_MISSION_ITEM_REACHED, out_buf);
}

uint8_t mav_encode_param_value(mav_encoder_t *enc,
                               const char *name,
                               fx_t        value,
                               uint8_t     param_type,
                               uint16_t    index,
                               uint16_t    count,
                               uint8_t    *out_buf) {
    mav_param_value_t pv;
    uint8_t i;

    mav_memset(&pv, 0, (uint8_t)sizeof(pv));
    pv.param_value = fx_to_ieee754(value);
    pv.param_count = count;
    pv.param_index = index;
    pv.param_type  = param_type;
    for (i = 0; i < 16u && name[i] != '\0'; i++) {
        pv.param_id[i] = name[i];
    }

    return mav_build_frame(enc, MAVLINK_MSG_ID_PARAM_VALUE,
                           MAVLINK_CRC_EXTRA_PARAM_VALUE,
                           &pv, MAVLINK_MSG_LEN_PARAM_VALUE, out_buf);
}
