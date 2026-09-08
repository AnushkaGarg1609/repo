#include "rotation.h"

/**
 * ROTATION.C
 *
 * Frame rotation matrix implementations using CORDIC-computed sin/cos.
 * All rotations follow aerospace right-hand convention:
 *   - Roll (φ): rotation about X (forward) axis
 *   - Pitch (θ): rotation about Y (right) axis
 *   - Yaw (ψ): rotation about Z (down) axis
 */

/* ============================================================================
 * Body → World Rotation (Section 6.5, 6.6)
 * ========================================================================= */

void rotate_accel_to_world(fx_t accel_x, fx_t accel_y, fx_t accel_z,
                           fx_t roll, fx_t pitch, fx_t yaw,
                           fx_t *accel_n, fx_t *accel_e, fx_t *accel_d) {
    /* Compute sin/cos for all three Euler angles using CORDIC */
    fx_t sin_roll, cos_roll, sin_pitch, cos_pitch, sin_yaw, cos_yaw;
    cordic_sincos(roll, &sin_roll, &cos_roll);
    cordic_sincos(pitch, &sin_pitch, &cos_pitch);
    cordic_sincos(yaw, &sin_yaw, &cos_yaw);

    /* Full 3D rotation matrix: R = R_yaw * R_pitch * R_roll
     *
     * Expanded form (body XYZ → world NED):
     * [N]   [cos(ψ)cos(θ)    cos(ψ)sin(θ)sin(φ)-sin(ψ)cos(φ)    cos(ψ)sin(θ)cos(φ)+sin(ψ)sin(φ)] [X]
     * [E] = [sin(ψ)cos(θ)    sin(ψ)sin(θ)sin(φ)+cos(ψ)cos(φ)    sin(ψ)sin(θ)cos(φ)-cos(ψ)sin(φ)] [Y]
     * [D]   [-sin(θ)         cos(θ)sin(φ)                        cos(θ)cos(φ)                    ] [Z]
     */

    /* First row: North component */
    fx_t r11 = fx_mul(cos_yaw, cos_pitch);
    fx_t r12 = fx_sub(fx_mul(fx_mul(cos_yaw, sin_pitch), sin_roll),
                      fx_mul(sin_yaw, cos_roll));
    fx_t r13 = fx_add(fx_mul(fx_mul(cos_yaw, sin_pitch), cos_roll),
                      fx_mul(sin_yaw, sin_roll));

    /* Second row: East component */
    fx_t r21 = fx_mul(sin_yaw, cos_pitch);
    fx_t r22 = fx_add(fx_mul(fx_mul(sin_yaw, sin_pitch), sin_roll),
                      fx_mul(cos_yaw, cos_roll));
    fx_t r23 = fx_sub(fx_mul(fx_mul(sin_yaw, sin_pitch), cos_roll),
                      fx_mul(cos_yaw, sin_roll));

    /* Third row: Down component */
    fx_t r31 = fx_neg(sin_pitch);
    fx_t r32 = fx_mul(cos_pitch, sin_roll);
    fx_t r33 = fx_mul(cos_pitch, cos_roll);

    /* Matrix-vector multiply: [N, E, D]^T = R * [X, Y, Z]^T */
    *accel_n = fx_add(fx_add(fx_mul(r11, accel_x), fx_mul(r12, accel_y)),
                      fx_mul(r13, accel_z));
    *accel_e = fx_add(fx_add(fx_mul(r21, accel_x), fx_mul(r22, accel_y)),
                      fx_mul(r23, accel_z));
    *accel_d = fx_add(fx_add(fx_mul(r31, accel_x), fx_mul(r32, accel_y)),
                      fx_mul(r33, accel_z));
}

/* ============================================================================
 * World → Body Rotation (Section 7.6)
 * ========================================================================= */

void rotate_accel_to_body(fx_t accel_n, fx_t accel_e, fx_t accel_d,
                          fx_t roll, fx_t pitch, fx_t yaw,
                          fx_t *accel_x, fx_t *accel_y, fx_t *accel_z) {
    /* Compute sin/cos for all three Euler angles using CORDIC */
    fx_t sin_roll, cos_roll, sin_pitch, cos_pitch, sin_yaw, cos_yaw;
    cordic_sincos(roll, &sin_roll, &cos_roll);
    cordic_sincos(pitch, &sin_pitch, &cos_pitch);
    cordic_sincos(yaw, &sin_yaw, &cos_yaw);

    /* Inverse rotation matrix: R^T = R_roll^T * R_pitch^T * R_yaw^T
     * (Transpose of rotation matrix = inverse for orthogonal matrices)
     *
     * Expanded form (world NED → body XYZ):
     * [X]   [cos(ψ)cos(θ)                        sin(ψ)cos(θ)                       -sin(θ)        ] [N]
     * [Y] = [cos(ψ)sin(θ)sin(φ)-sin(ψ)cos(φ)    sin(ψ)sin(θ)sin(φ)+cos(ψ)cos(φ)    cos(θ)sin(φ)  ] [E]
     * [Z]   [cos(ψ)sin(θ)cos(φ)+sin(ψ)sin(φ)    sin(ψ)sin(θ)cos(φ)-cos(ψ)sin(φ)    cos(θ)cos(φ)  ] [D]
     */

    /* First row: X (forward) component */
    fx_t rt11 = fx_mul(cos_yaw, cos_pitch);
    fx_t rt12 = fx_mul(sin_yaw, cos_pitch);
    fx_t rt13 = fx_neg(sin_pitch);

    /* Second row: Y (right) component */
    fx_t rt21 = fx_sub(fx_mul(fx_mul(cos_yaw, sin_pitch), sin_roll),
                       fx_mul(sin_yaw, cos_roll));
    fx_t rt22 = fx_add(fx_mul(fx_mul(sin_yaw, sin_pitch), sin_roll),
                       fx_mul(cos_yaw, cos_roll));
    fx_t rt23 = fx_mul(cos_pitch, sin_roll);

    /* Third row: Z (down) component */
    fx_t rt31 = fx_add(fx_mul(fx_mul(cos_yaw, sin_pitch), cos_roll),
                       fx_mul(sin_yaw, sin_roll));
    fx_t rt32 = fx_sub(fx_mul(fx_mul(sin_yaw, sin_pitch), cos_roll),
                       fx_mul(cos_yaw, sin_roll));
    fx_t rt33 = fx_mul(cos_pitch, cos_roll);

    /* Matrix-vector multiply: [X, Y, Z]^T = R^T * [N, E, D]^T */
    *accel_x = fx_add(fx_add(fx_mul(rt11, accel_n), fx_mul(rt12, accel_e)),
                      fx_mul(rt13, accel_d));
    *accel_y = fx_add(fx_add(fx_mul(rt21, accel_n), fx_mul(rt22, accel_e)),
                      fx_mul(rt23, accel_d));
    *accel_z = fx_add(fx_add(fx_mul(rt31, accel_n), fx_mul(rt32, accel_e)),
                      fx_mul(rt33, accel_d));
}

/* ============================================================================
 * Optimized Vertical-Only Extraction (Section 6.5)
 * ========================================================================= */

fx_t extract_vertical_accel(fx_t accel_x, fx_t accel_y, fx_t accel_z,
                            fx_t roll, fx_t pitch) {
    /* Only compute the Down component (third row of body→world matrix)
     * Down = -sin(θ)*X + cos(θ)sin(φ)*Y + cos(θ)cos(φ)*Z
     *
     * This saves 6 CORDIC calls (no yaw needed) and ~70% of the multiplications
     * compared to full rotate_accel_to_world().
     */

    fx_t sin_roll, cos_roll, sin_pitch, cos_pitch;
    cordic_sincos(roll, &sin_roll, &cos_roll);
    cordic_sincos(pitch, &sin_pitch, &cos_pitch);

    fx_t term1 = fx_mul(fx_neg(sin_pitch), accel_x);
    fx_t term2 = fx_mul(fx_mul(cos_pitch, sin_roll), accel_y);
    fx_t term3 = fx_mul(fx_mul(cos_pitch, cos_roll), accel_z);

    return fx_add(fx_add(term1, term2), term3);
}

/* ============================================================================
 * QUATERNION ↔ EULER CONVERSIONS (for simulation)
 * ========================================================================= */

#include <math.h>

void quat_to_euler(const quaternion_t *q, euler_t *out_euler) {
    /* Convert quaternion to Euler angles using standard aerospace convention
     * ZYX (yaw-pitch-roll) intrinsic rotation sequence
     */

    float w = q->w, x = q->x, y = q->y, z = q->z;

    /* Roll (φ): rotation about X axis */
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    out_euler->roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (θ): rotation about Y axis */
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        /* Gimbal lock: pitch = ±90° */
        out_euler->pitch = copysignf(3.14159265f / 2.0f, sinp);
    } else {
        out_euler->pitch = asinf(sinp);
    }

    /* Yaw (ψ): rotation about Z axis */
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    out_euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

void euler_to_quat(const euler_t *euler, quaternion_t *out_q) {
    /* Convert Euler angles to quaternion */

    float roll = euler->roll, pitch = euler->pitch, yaw = euler->yaw;

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    out_q->w = cr * cp * cy + sr * sp * sy;
    out_q->x = sr * cp * cy - cr * sp * sy;
    out_q->y = cr * sp * cy + sr * cp * sy;
    out_q->z = cr * cp * sy - sr * sp * cy;
}
