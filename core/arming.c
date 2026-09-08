#include "arming.h"

/**
 * ARMING.C
 *
 * Section 11.1: Arming State Machine Implementation
 */

/* ============================================================================
 * SECTION 11.1: Arming State Machine
 * ========================================================================= */

arming_result_t attempt_arm(const flight_state_t *state, arming_state_t *arm_state) {
    /* Check 0: Already armed? */
    if (*arm_state == ARMING_ARMED) {
        return ARM_DENIED_ALREADY_ARMED;
    }

    /* Check 1: IMU calibrated (Section 11.1) */
    if (!state->imu_calibrated) {
        return ARM_DENIED_IMU_FAULT;
    }

    /* Check 2: No critical sensor faults */
    if (state->critical_sensor_fault) {
        return ARM_DENIED_SENSOR_FAULT;
    }

    /* Check 3: Gyro stationary (all axes below threshold) */
    if (fx_abs(state->gyro_x) > GYRO_STATIONARY_THRESHOLD ||
        fx_abs(state->gyro_y) > GYRO_STATIONARY_THRESHOLD ||
        fx_abs(state->gyro_z) > GYRO_STATIONARY_THRESHOLD) {
        return ARM_DENIED_GYRO_MOVING;
    }

    /* Check 4: Throttle at minimum */
    if (state->throttle > THROTTLE_MIN_THRESHOLD) {
        return ARM_DENIED_THROTTLE_HIGH;
    }

    /* Check 5: Flight mode must be STABILIZE (Section 11.1)
     * "If the pilot's mode switch is in any position other than Manual/Stabilize
     * at the moment of an arm request, the arm transition is denied" */
    if (state->mode != MODE_STABILIZE) {
        return ARM_DENIED_MODE_INVALID;
    }

    /* All checks passed - ARM */
    *arm_state = ARMING_ARMED;
    return ARM_SUCCESS;
}

void disarm(arming_state_t *arm_state) {
    /* Disarming always succeeds - no preconditions */
    *arm_state = ARMING_DISARMED;
}

/* ============================================================================
 * Helper Functions
 * ========================================================================= */

const char* arming_result_name(arming_result_t result) {
    switch (result) {
        case ARM_SUCCESS:                return "SUCCESS";
        case ARM_DENIED_GYRO_MOVING:     return "DENIED: Gyro moving";
        case ARM_DENIED_THROTTLE_HIGH:   return "DENIED: Throttle high";
        case ARM_DENIED_MODE_INVALID:    return "DENIED: Invalid mode";
        case ARM_DENIED_ALREADY_ARMED:   return "DENIED: Already armed";
        case ARM_DENIED_IMU_FAULT:       return "DENIED: IMU fault";
        case ARM_DENIED_SENSOR_FAULT:    return "DENIED: Sensor fault";
        default:                         return "UNKNOWN";
    }
}

const char* arming_state_name(arming_state_t state) {
    switch (state) {
        case ARMING_DISARMED:   return "DISARMED";
        case ARMING_ARMED:      return "ARMED";
        case ARMING_DISARMING:  return "DISARMING";
        default:                return "UNKNOWN";
    }
}
