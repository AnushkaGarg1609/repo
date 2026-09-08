#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

#include <stdbool.h>
#include <stdint.h>
#include "fixed_point.h"

/**
 * FLIGHT_MODES.H
 *
 * Section 9: Flight Mode and Manual Override Architecture
 * Implements the mode state machine from Figure 9.1
 */

/* ============================================================================
 * SECTION 9.1: Mode Taxonomy
 * ========================================================================= */

/**
 * Flight mode enumeration (Section 9.1).
 * Each mode has different setpoint sources for attitude/throttle/position.
 */
typedef enum {
    MODE_DISARMED,      /* Motors disabled, initialization state */
    MODE_STABILIZE,     /* Manual flight: pilot controls attitude directly */
    MODE_ALT_HOLD,      /* Altitude hold: pilot controls attitude, autopilot holds altitude */
    MODE_POS_HOLD,      /* Position hold: autopilot holds position, pilot can nudge */
    MODE_MISSION,       /* Autonomous: mission sequencer controls position setpoints */
    MODE_RTL            /* Return-to-launch: fly to home position automatically */
} flight_mode_t;

/* ============================================================================
 * SECTION 9.2: RC Input Structure
 * ========================================================================= */

/**
 * RC switch positions for mode selection.
 * Mapped from a physical switch channel on the RC transmitter.
 */
typedef enum {
    SWITCH_STABILIZE = 0,
    SWITCH_ALT_HOLD  = 1,
    SWITCH_POS_HOLD  = 2,
    SWITCH_AUTO      = 3,   /* Mission/Auto mode */
    SWITCH_UNDEFINED = 255
} rc_switch_position_t;

/**
 * RC input structure.
 * Contains decoded stick positions and switch states.
 */
typedef struct {
    fx_t roll;         /* Roll stick [-1.0, 1.0] in Q16.16 */
    fx_t pitch;        /* Pitch stick [-1.0, 1.0] in Q16.16 */
    fx_t throttle;     /* Throttle stick [0.0, 1.0] in Q16.16 */
    fx_t yaw;          /* Yaw stick [-1.0, 1.0] in Q16.16 */

    rc_switch_position_t mode_switch_position;  /* Flight mode selector */
    bool arm_switch;                            /* Arm/disarm switch */

    uint32_t last_valid_frame_time_ms;  /* Timestamp of last valid RC frame */
} rc_input_t;

/**
 * RC stick deadband (centered stick is not exactly zero).
 * If |stick| < deadband, treat as zero.
 */
#define RC_STICK_DEADBAND fx_from_float(0.05f)  /* 5% deadband */

/**
 * Check if any RC stick is deflected beyond deadband (for override detection).
 * @param rc RC input structure
 * @return true if any stick is actively deflected
 */
bool rc_stick_deflected(const rc_input_t *rc);

/* ============================================================================
 * SECTION 9.3: Navigation Health Structure
 * ========================================================================= */

/**
 * Navigation health status.
 * Tracks whether each sensor subsystem meets its quality requirements.
 */
typedef struct {
    bool baro_ok;       /* Barometer calibrated and reading valid */
    bool gps_fix_ok;    /* GPS has 3D fix with sufficient satellites */
    bool home_settled;  /* Home position latched and stable */
    bool imu_ok;        /* IMU passing self-test and calibrated */
    bool mag_ok;        /* Magnetometer calibrated (optional, improves yaw) */
} nav_health_t;

/* ============================================================================
 * SECTION 9.4: Mode State Machine (Listing 9.1)
 * ========================================================================= */

/**
 * Evaluate flight mode transitions based on current state, RC input, and sensor health.
 * Implements the state machine from Figure 9.1 with entry interlocks.
 *
 * Entry interlocks (Section 9.3):
 * - ALT_HOLD requires: baro_ok
 * - POS_HOLD requires: baro_ok + gps_fix_ok + home_settled
 * - MISSION requires: POS_HOLD working + mission uploaded + pilot selects AUTO
 *
 * Override Supremacy Rule (Section 9.2):
 * - Any deliberate stick deflection in MISSION/RTL immediately demotes to POS_HOLD
 *
 * @param current Current flight mode
 * @param rc RC input (sticks and switches)
 * @param health Navigation health status
 * @param mission_active_request True if mission is uploaded and ready to execute
 * @return Next flight mode (may be same as current if no transition)
 */
flight_mode_t evaluate_mode_transition(flight_mode_t current,
                                       const rc_input_t *rc,
                                       const nav_health_t *health,
                                       bool mission_active_request);

/**
 * Get human-readable name for a flight mode.
 * @param mode Flight mode
 * @return String name (statically allocated, do not free)
 */
const char* flight_mode_name(flight_mode_t mode);

#endif /* FLIGHT_MODES_H */
