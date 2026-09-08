#include "flight_modes.h"

/**
 * FLIGHT_MODES.C
 *
 * Section 9: Flight Mode State Machine Implementation
 */

/* ============================================================================
 * SECTION 9.2: RC Stick Override Detection
 * ========================================================================= */

bool rc_stick_deflected(const rc_input_t *rc) {
    /* Check if any stick exceeds the deadband threshold */
    return (fx_abs(rc->roll) > RC_STICK_DEADBAND) ||
           (fx_abs(rc->pitch) > RC_STICK_DEADBAND) ||
           (fx_abs(rc->yaw) > RC_STICK_DEADBAND);
    /* Note: throttle is not checked for override - it's always pilot-controlled
     * in most modes except full autonomous altitude control */
}

/* ============================================================================
 * SECTION 9.4: Mode Transition State Machine (Listing 9.1)
 * ========================================================================= */

flight_mode_t evaluate_mode_transition(flight_mode_t current,
                                       const rc_input_t *rc,
                                       const nav_health_t *health,
                                       bool mission_active_request) {
    /* Override Supremacy Rule (Section 9.2):
     * Any raw stick deflection in autonomous modes forces immediate demotion */
    if (current == MODE_MISSION || current == MODE_RTL) {
        if (rc_stick_deflected(rc)) {
            /* Pilot is taking control - demote to Position Hold if GPS ok,
             * otherwise Stabilize */
            return health->gps_fix_ok ? MODE_POS_HOLD : MODE_STABILIZE;
        }
    }

    /* Evaluate mode based on RC switch position and sensor health */
    switch (rc->mode_switch_position) {
        case SWITCH_STABILIZE:
            /* Stabilize mode has no preconditions - always safe to enter */
            return MODE_STABILIZE;

        case SWITCH_ALT_HOLD:
            /* Altitude Hold requires valid barometer */
            if (health->baro_ok) {
                return MODE_ALT_HOLD;
            } else {
                /* Barometer not ready - fall back to Stabilize */
                return MODE_STABILIZE;
            }

        case SWITCH_POS_HOLD:
            /* Position Hold requires GPS fix + home settled + barometer */
            if (health->gps_fix_ok && health->home_settled && health->baro_ok) {
                return MODE_POS_HOLD;
            } else if (health->baro_ok) {
                /* GPS not ready but baro ok - fall back to Altitude Hold */
                return MODE_ALT_HOLD;
            } else {
                /* Neither GPS nor baro ready - fall back to Stabilize */
                return MODE_STABILIZE;
            }

        case SWITCH_AUTO:
            /* Mission/Auto mode has strict entry requirements:
             * 1. Must currently be in Position Hold (or already in Mission)
             * 2. GPS fix must be good
             * 3. Mission must be uploaded and valid
             */
            if (current != MODE_POS_HOLD && current != MODE_MISSION) {
                /* Can't enter Mission directly from other modes - must transit
                 * through Position Hold first (Section 9.3) */
                if (health->gps_fix_ok && health->home_settled && health->baro_ok) {
                    return MODE_POS_HOLD;  /* Step 1: enter Position Hold */
                } else if (health->baro_ok) {
                    return MODE_ALT_HOLD;
                } else {
                    return MODE_STABILIZE;
                }
            }

            /* Already in Position Hold or Mission - check if we can enter/stay in Mission */
            if (health->gps_fix_ok && health->home_settled && mission_active_request) {
                return MODE_MISSION;
            } else {
                /* Mission not ready or GPS lost - fall back to Position Hold */
                return MODE_POS_HOLD;
            }

        case SWITCH_UNDEFINED:
        default:
            /* Undefined switch state - fail safe to Stabilize */
            return MODE_STABILIZE;
    }
}

/* ============================================================================
 * Helper: Mode Name String
 * ========================================================================= */

const char* flight_mode_name(flight_mode_t mode) {
    switch (mode) {
        case MODE_DISARMED:  return "DISARMED";
        case MODE_STABILIZE: return "STABILIZE";
        case MODE_ALT_HOLD:  return "ALT_HOLD";
        case MODE_POS_HOLD:  return "POS_HOLD";
        case MODE_MISSION:   return "MISSION";
        case MODE_RTL:       return "RTL";
        default:             return "UNKNOWN";
    }
}
