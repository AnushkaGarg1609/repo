#ifndef ARMING_H
#define ARMING_H

#include <stdbool.h>
#include <stdint.h>
#include "fixed_point.h"
#include "flight_modes.h"
#include "sensor_fusion.h"

/**
 * ARMING.H
 *
 * Section 11.1: The Arming State Machine
 * Ensures motors only spin when it's safe to do so.
 */

/* ============================================================================
 * SECTION 11.1: Arming States
 * ========================================================================= */

/**
 * Arming state enumeration.
 * Only in ARMED state are motor outputs permitted to reach the ESCs.
 */
typedef enum {
    ARMING_DISARMED,        /* Motors disabled, safe state */
    ARMING_ARMED,           /* Motors enabled, can spin */
    ARMING_DISARMING        /* Transition state (not used in simple implementation) */
} arming_state_t;

/**
 * Arming attempt result codes.
 * Indicates why an arm attempt succeeded or failed.
 */
typedef enum {
    ARM_SUCCESS,                /* Arming successful */
    ARM_DENIED_GYRO_MOVING,     /* Gyro detects motion (not stationary) */
    ARM_DENIED_THROTTLE_HIGH,   /* Throttle stick not at minimum */
    ARM_DENIED_MODE_INVALID,    /* Flight mode not safe for arming */
    ARM_DENIED_ALREADY_ARMED,   /* Already in armed state */
    ARM_DENIED_IMU_FAULT,       /* IMU not calibrated or failed self-test */
    ARM_DENIED_SENSOR_FAULT     /* Other critical sensor failure */
} arming_result_t;

/* ============================================================================
 * SECTION 11.1: Flight State Structure
 * ========================================================================= */

/**
 * Current flight state needed for arming checks.
 * Subset of the full vehicle state relevant to safety checks.
 */
typedef struct {
    /* Gyro rates (for stationary check) */
    fx_t gyro_x;  /* Roll rate (rad/s, Q16.16) */
    fx_t gyro_y;  /* Pitch rate (rad/s, Q16.16) */
    fx_t gyro_z;  /* Yaw rate (rad/s, Q16.16) */

    /* RC input */
    fx_t throttle;  /* Throttle stick [0.0, 1.0] in Q16.16 */

    /* Flight mode */
    flight_mode_t mode;

    /* Sensor health */
    bool imu_calibrated;
    bool critical_sensor_fault;  /* Any sensor critical to current mode failed */
} flight_state_t;

/* ============================================================================
 * SECTION 11.1: Arming Thresholds
 * ========================================================================= */

/**
 * Maximum gyro rate for stationary check (rad/s).
 * If any axis exceeds this, vehicle is considered moving.
 */
#define GYRO_STATIONARY_THRESHOLD fx_from_float(0.05f)  /* 0.05 rad/s ≈ 2.9°/s */

/**
 * Maximum throttle stick position for arming (0.0 = min, 1.0 = max).
 * Throttle must be below this to arm.
 */
#define THROTTLE_MIN_THRESHOLD fx_from_float(0.1f)  /* 10% stick position */

/* ============================================================================
 * SECTION 11.1: Arming Functions
 * ========================================================================= */

/**
 * Attempt to arm the vehicle.
 * Checks all preconditions and transitions to ARMED if safe.
 *
 * Preconditions (Section 11.1):
 * 1. Gyro stationary (all axes < threshold)
 * 2. Throttle at minimum
 * 3. Flight mode is STABILIZE (if mode check enabled)
 * 4. IMU calibrated
 * 5. No critical sensor faults
 *
 * @param state Current flight state
 * @param arm_state Current arming state (will be modified on success)
 * @return ARM_SUCCESS or specific denial reason
 */
arming_result_t attempt_arm(const flight_state_t *state, arming_state_t *arm_state);

/**
 * Disarm the vehicle.
 * Always succeeds - no preconditions for disarming.
 *
 * @param arm_state Current arming state (will be set to DISARMED)
 */
void disarm(arming_state_t *arm_state);

/**
 * Check if vehicle is currently armed.
 *
 * @param arm_state Current arming state
 * @return true if armed, false if disarmed
 */
static inline bool is_armed(arming_state_t arm_state) {
    return arm_state == ARMING_ARMED;
}

/**
 * Get human-readable name for arming result.
 *
 * @param result Arming result code
 * @return String description (statically allocated, do not free)
 */
const char* arming_result_name(arming_result_t result);

/**
 * Get human-readable name for arming state.
 *
 * @param state Arming state
 * @return String name (statically allocated, do not free)
 */
const char* arming_state_name(arming_state_t state);

#endif /* ARMING_H */
