/**
 * mavlink_encode.h
 *
 * MAVLink v2 Message Encoder --- Outbound Telemetry & Protocol Responses
 *
 * Each function takes flight state as input and fills a caller-supplied byte
 * buffer with one complete, ready-to-transmit MAVLink v2 frame.
 *
 * No hardware calls. No malloc. No floating-point arithmetic. Caller owns the
 * buffer, which must be at least MAVLINK_MAX_FRAME_LEN bytes.
 *
 * ---------------------------------------------------------------------------
 * WHY THESE SIGNATURES TAKE fx_t AND INTEGERS, NOT float
 * ---------------------------------------------------------------------------
 * The flight core (core/) computes everything in Q16.16 and the ET1031 has no
 * FPU, so accepting `float` here would force a soft-float conversion at every
 * call site inside the 10 Hz telemetry slot. Angles, speeds and altitudes are
 * therefore passed as fx_t and converted to IEEE-754 bit patterns by
 * fx_to_ieee754() in mavlink_types.h using shifts alone.
 *
 * Latitude and longitude are passed as int32_t degE7 --- the GPS receiver's
 * native units and MAVLink's wire units --- because neither float nor Q16.16
 * can hold nine significant digits. No conversion happens on this path at all.
 */

#ifndef MAVLINK_ENCODE_H
#define MAVLINK_ENCODE_H

#include "mavlink_types.h"

/* ============================================================================
 * Encoder State
 *
 * One instance per outgoing link. The sequence counter increments on every
 * transmitted frame and wraps at 256; the GCS uses gaps in it to compute the
 * link drop rate shown in its status bar.
 * ========================================================================= */

typedef struct {
    uint8_t seq;      /* Rolling sequence number [0..255], auto-wraps */
    uint8_t sysid;    /* Our system ID    (default MAV_SYS_ID) */
    uint8_t compid;   /* Our component ID (default MAV_COMP_ID_AUTOPILOT) */
} mav_encoder_t;

void mav_encoder_init(mav_encoder_t *enc);

/** Override the default system/component identity (rarely needed). */
void mav_encoder_set_identity(mav_encoder_t *enc, uint8_t sysid, uint8_t compid);

/* ============================================================================
 * Frame Builder (exposed for testing and for messages not wrapped below)
 *
 * Packs header + (truncated) payload + CRC into out_buf and returns the total
 * frame length in bytes. Trailing zero bytes of the payload are dropped per
 * the MAVLink v2 truncation rule, and the CRC covers only the bytes actually
 * sent.
 * ========================================================================= */

uint8_t mav_build_frame(mav_encoder_t *enc,
                        uint32_t    msg_id,
                        uint8_t     crc_extra,
                        const void *payload,
                        uint8_t     payload_len,
                        uint8_t    *out_buf);

/* ============================================================================
 * Outbound Telemetry Encoders
 * ========================================================================= */

/**
 * HEARTBEAT (ID 0) --- 1 Hz. The one message the GCS will not connect without.
 *   armed        : true if motors are armed (arming.h ARMING_ARMED)
 *   flight_mode  : our flight_mode_t, sent verbatim as custom_mode
 *                  0=DISARMED 1=STABILIZE 2=ALT_HOLD 3=POS_HOLD 4=MISSION 5=RTL
 *   sys_state    : MAV_STATE (STANDBY when disarmed, ACTIVE when flying,
 *                  CRITICAL/EMERGENCY on failsafe --- Section 11.2)
 */
uint8_t mav_encode_heartbeat(mav_encoder_t *enc,
                             bool     armed,
                             uint8_t  flight_mode,
                             uint8_t  sys_state,
                             uint8_t *out_buf);

/**
 * SYS_STATUS (ID 1) --- 1 Hz. Battery and per-sensor health flags.
 *   battery_mv   : pack voltage in millivolts (11100 = 11.1 V), 0xFFFF unknown
 *   battery_pct  : 0..100, or -1 if not estimated
 *   cpu_load     : 0..1000 in 1/10 %, i.e. the Section 12 loop-budget headroom
 *   sensors_*    : MAV_SYS_STATUS_SENSOR bitmasks
 */
uint8_t mav_encode_sys_status(mav_encoder_t *enc,
                              uint16_t battery_mv,
                              int8_t   battery_pct,
                              uint16_t cpu_load,
                              uint32_t sensors_present,
                              uint32_t sensors_enabled,
                              uint32_t sensors_health,
                              uint8_t *out_buf);

/**
 * ATTITUDE (ID 30) --- 10 Hz. Drives the GCS artificial horizon.
 *   roll/pitch/yaw : radians, Q16.16 (straight from attitude_t)
 *   *speed         : body angular rates, rad/s, Q16.16 (straight from the gyro)
 */
uint8_t mav_encode_attitude(mav_encoder_t *enc,
                            uint32_t time_boot_ms,
                            fx_t roll,      fx_t pitch,      fx_t yaw,
                            fx_t rollspeed, fx_t pitchspeed, fx_t yawspeed,
                            uint8_t *out_buf);

/**
 * GLOBAL_POSITION_INT (ID 33) --- 5 Hz. Moves the icon on the GCS map.
 *   lat_e7/lon_e7  : degrees * 1e7 (GPS-native, no conversion)
 *   alt_mm_msl     : millimetres above mean sea level
 *   rel_alt_mm     : millimetres above home
 *   vn/ve/vd       : NED velocity, m/s, Q16.16 (converted to cm/s internally)
 *   hdg_cdeg       : heading in centidegrees 0..35999, or UINT16_MAX if unknown
 */
uint8_t mav_encode_global_position_int(mav_encoder_t *enc,
                                       uint32_t time_boot_ms,
                                       int32_t  lat_e7, int32_t lon_e7,
                                       int32_t  alt_mm_msl, int32_t rel_alt_mm,
                                       fx_t     vn, fx_t ve, fx_t vd,
                                       uint16_t hdg_cdeg,
                                       uint8_t *out_buf);

/**
 * GPS_RAW_INT (ID 24) --- 5 Hz. Satellite count and fix type, i.e. the
 * Section 5.7 quality gate made visible on the ground.
 *   fix_type       : 0/1 = none, 2 = 2D, 3 = 3D
 *   eph            : HDOP * 100, or UINT16_MAX if unknown
 *   vel_cms        : ground speed, cm/s
 *   cog_cdeg       : course over ground, centidegrees 0..35999
 */
uint8_t mav_encode_gps_raw_int(mav_encoder_t *enc,
                               uint64_t time_usec,
                               int32_t  lat_e7, int32_t lon_e7, int32_t alt_mm_msl,
                               uint8_t  fix_type, uint8_t satellites_visible,
                               uint16_t eph, uint16_t vel_cms, uint16_t cog_cdeg,
                               uint8_t *out_buf);

/**
 * VFR_HUD (ID 74) --- 5 Hz. The GCS heads-up numeric readout.
 *   groundspeed / alt / climb : m/s, m MSL, m/s (Q16.16); climb positive up
 *   heading_deg               : 0..359
 *   throttle_pct              : 0..100
 */
uint8_t mav_encode_vfr_hud(mav_encoder_t *enc,
                           fx_t     groundspeed,
                           fx_t     alt,
                           fx_t     climb,
                           int16_t  heading_deg,
                           uint16_t throttle_pct,
                           uint8_t *out_buf);

/**
 * STATUSTEXT (ID 253) --- event-driven. Text appears in the GCS message pane;
 * the natural place to surface arming denials (arming_result_name()) and
 * failsafe trips. `text` is copied and truncated at 50 bytes.
 */
uint8_t mav_encode_statustext(mav_encoder_t *enc,
                              uint8_t     severity,
                              const char *text,
                              uint8_t    *out_buf);

/* ============================================================================
 * Protocol Responses (mission and command handshakes)
 * ========================================================================= */

/** COMMAND_ACK (ID 77) --- mandatory reply to every COMMAND_LONG received. */
uint8_t mav_encode_command_ack(mav_encoder_t *enc,
                               uint16_t command,
                               uint8_t  result,      /* MAV_RESULT */
                               uint8_t *out_buf);

/** MISSION_COUNT (ID 44) --- "I hold N waypoints", opens a download. */
uint8_t mav_encode_mission_count(mav_encoder_t *enc,
                                 uint16_t count,
                                 uint8_t  target_sys,
                                 uint8_t  target_comp,
                                 uint8_t *out_buf);

/** MISSION_REQUEST_INT (ID 51) --- "send me waypoint seq", drives an upload. */
uint8_t mav_encode_mission_request_int(mav_encoder_t *enc,
                                       uint16_t seq,
                                       uint8_t  target_sys,
                                       uint8_t  target_comp,
                                       uint8_t *out_buf);

/**
 * MISSION_ITEM_INT (ID 73) --- one waypoint, sent during a download.
 *   lat_e7/lon_e7 : degrees * 1e7
 *   alt_m         : metres above home, Q16.16
 *   accept_radius : acceptance radius in metres, Q16.16 (param2)
 */
uint8_t mav_encode_mission_item_int(mav_encoder_t *enc,
                                    uint16_t seq,
                                    uint16_t command,       /* MAV_CMD */
                                    uint8_t  frame,         /* MAV_FRAME */
                                    uint8_t  current,
                                    uint8_t  autocontinue,
                                    int32_t  lat_e7, int32_t lon_e7,
                                    fx_t     alt_m,
                                    fx_t     accept_radius,
                                    uint8_t  target_sys, uint8_t target_comp,
                                    uint8_t *out_buf);

/** MISSION_ACK (ID 47) --- terminates an upload or a download. */
uint8_t mav_encode_mission_ack(mav_encoder_t *enc,
                               uint8_t  target_sys,
                               uint8_t  target_comp,
                               uint8_t  result,     /* MAV_MISSION_RESULT */
                               uint8_t *out_buf);

/** MISSION_CURRENT (ID 42) --- which waypoint the sequencer is tracking. */
uint8_t mav_encode_mission_current(mav_encoder_t *enc,
                                   uint16_t seq,
                                   uint8_t *out_buf);

/** MISSION_ITEM_REACHED (ID 46) --- fired once as each waypoint is accepted. */
uint8_t mav_encode_mission_item_reached(mav_encoder_t *enc,
                                        uint16_t seq,
                                        uint8_t *out_buf);

/**
 * PARAM_VALUE (ID 22) --- one tuning parameter. The GCS treats a parameter
 * list as mandatory before it considers a vehicle fully connected, so this is
 * not optional in practice.
 *   name  : up to 16 chars, zero-padded (not necessarily terminated)
 *   value : Q16.16, sent as REAL32
 */
uint8_t mav_encode_param_value(mav_encoder_t *enc,
                               const char *name,
                               fx_t        value,
                               uint8_t     param_type,
                               uint16_t    index,
                               uint16_t    count,
                               uint8_t    *out_buf);

#endif /* MAVLINK_ENCODE_H */
