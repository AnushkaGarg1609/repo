#ifndef ROTATION_H
#define ROTATION_H

#include "fixed_point.h"

/**
 * ROTATION.H
 *
 * Frame rotation utilities for flight control.
 * Converts between body frame (X=forward, Y=right, Z=down) and
 * world frame (N=north, E=east, D=down) using Euler angles.
 */

/* ============================================================================
 * QUATERNION AND EULER ANGLE TYPES
 * ========================================================================= */

/**
 * Quaternion representation (floating-point for simulation).
 * Represents rotation: q = w + xi + yj + zk
 */
typedef struct {
    float w, x, y, z;
} quaternion_t;

/**
 * Euler angles (floating-point, radians).
 * Roll-pitch-yaw convention.
 */
typedef struct {
    float roll;   /* Rotation about X axis (radians) */
    float pitch;  /* Rotation about Y axis (radians) */
    float yaw;    /* Rotation about Z axis (radians) */
} euler_t;

/* ============================================================================
 * SECTION 7.6 & 6.5: Frame Rotation Matrices
 * ========================================================================= */

/**
 * Rotate acceleration vector from world frame (NED) to body frame (XYZ).
 *
 * Uses the inverse rotation matrix (world → body):
 *   R_transpose = R_yaw^T * R_pitch^T * R_roll^T
 *
 * This is needed by the position controller (Section 7.6) to convert
 * desired world-frame acceleration commands into body-frame roll/pitch setpoints.
 *
 * @param accel_n North acceleration (world frame, Q16.16)
 * @param accel_e East acceleration (world frame, Q16.16)
 * @param accel_d Down acceleration (world frame, Q16.16)
 * @param roll Current roll angle in radians (Q16.16)
 * @param pitch Current pitch angle in radians (Q16.16)
 * @param yaw Current yaw angle in radians (Q16.16)
 * @param accel_x Output: forward acceleration (body frame, Q16.16)
 * @param accel_y Output: right acceleration (body frame, Q16.16)
 * @param accel_z Output: down acceleration (body frame, Q16.16)
 */
void rotate_accel_to_body(fx_t accel_n, fx_t accel_e, fx_t accel_d,
                          fx_t roll, fx_t pitch, fx_t yaw,
                          fx_t *accel_x, fx_t *accel_y, fx_t *accel_z);

/**
 * Rotate acceleration vector from body frame (XYZ) to world frame (NED).
 *
 * Uses the forward rotation matrix (body → world):
 *   R = R_roll * R_pitch * R_yaw
 *
 * This is needed by the navigation filter (Section 6.6) to integrate
 * IMU-measured body-frame acceleration into world-frame position/velocity estimates.
 *
 * Also used by altitude filter (Section 6.5) to extract vertical (Down) component
 * of body-frame acceleration for altitude estimation.
 *
 * @param accel_x Forward acceleration (body frame, Q16.16)
 * @param accel_y Right acceleration (body frame, Q16.16)
 * @param accel_z Down acceleration (body frame, Q16.16)
 * @param roll Current roll angle in radians (Q16.16)
 * @param pitch Current pitch angle in radians (Q16.16)
 * @param yaw Current yaw angle in radians (Q16.16)
 * @param accel_n Output: North acceleration (world frame, Q16.16)
 * @param accel_e Output: East acceleration (world frame, Q16.16)
 * @param accel_d Output: Down acceleration (world frame, Q16.16)
 */
void rotate_accel_to_world(fx_t accel_x, fx_t accel_y, fx_t accel_z,
                           fx_t roll, fx_t pitch, fx_t yaw,
                           fx_t *accel_n, fx_t *accel_e, fx_t *accel_d);

/**
 * Helper: Extract vertical (down) acceleration from body frame.
 * Simplified version of rotate_accel_to_world that only computes the Down component.
 * More efficient when only altitude estimation is needed (Section 6.5).
 *
 * @param accel_x Forward acceleration (body frame, Q16.16)
 * @param accel_y Right acceleration (body frame, Q16.16)
 * @param accel_z Down acceleration (body frame, Q16.16)
 * @param roll Current roll angle in radians (Q16.16)
 * @param pitch Current pitch angle in radians (Q16.16)
 * @return Down acceleration in world frame (Q16.16)
 */
fx_t extract_vertical_accel(fx_t accel_x, fx_t accel_y, fx_t accel_z,
                            fx_t roll, fx_t pitch);

/**
 * Convert quaternion to Euler angles.
 * Uses standard aerospace sequence: roll-pitch-yaw (ZYX).
 *
 * @param q Input quaternion
 * @param out_euler Output Euler angles (radians)
 */
void quat_to_euler(const quaternion_t *q, euler_t *out_euler);

/**
 * Convert Euler angles to quaternion.
 *
 * @param euler Input Euler angles (radians)
 * @param out_q Output quaternion
 */
void euler_to_quat(const euler_t *euler, quaternion_t *out_q);

#endif /* ROTATION_H */
