#include <stddef.h>   /* NULL --- pulled in transitively on the host, but the
                       * bare-metal/Arduino RV32 target needs it explicitly. */
#include "sensor_fusion.h"

/**
 * SENSOR_FUSION.C
 *
 * Section 6: Complementary Filter Implementations
 * Roll/Pitch (6.2, 6.3), Yaw (6.4), Altitude (6.5), Position (6.6)
 */

/* ============================================================================
 * SECTION 6.3: Roll/Pitch Complementary Filter (Listing 6.1)
 * ========================================================================= */

void update_attitude_complementary(const imu_sample_t *s, attitude_t *att) {
    /* Step 1: Gyro-integrated estimate (predict step) */
    fx_t roll_gyro = fx_add(att->roll, fx_mul(s->gx, DT_FAST_Q16));
    fx_t pitch_gyro = fx_add(att->pitch, fx_mul(s->gy, DT_FAST_Q16));

    /* Step 2: Accelerometer-derived roll and pitch (correct step)
     * roll = atan2(ay, az)
     * pitch = atan2(-ax, sqrt(ay² + az²))
     */
    fx_t roll_acc;
    cordic_atan2(s->ay, s->az, &roll_acc);

    /* Compute horizontal magnitude for pitch calculation */
    fx_t ay_sq = fx_mul(s->ay, s->ay);
    fx_t az_sq = fx_mul(s->az, s->az);
    fx_t horiz = fx_sqrt(fx_add(ay_sq, az_sq));

    fx_t pitch_acc;
    cordic_atan2(fx_neg(s->ax), horiz, &pitch_acc);

    /* Step 3: Complementary filter blend
     * attitude = alpha * gyro_estimate + (1 - alpha) * accel_estimate
     */
    att->roll = fx_add(
        fx_mul(ALPHA_ATTITUDE_Q16, roll_gyro),
        fx_mul(ONE_MINUS_ALPHA_ATT_Q16, roll_acc)
    );

    att->pitch = fx_add(
        fx_mul(ALPHA_ATTITUDE_Q16, pitch_gyro),
        fx_mul(ONE_MINUS_ALPHA_ATT_Q16, pitch_acc)
    );

    /* Note: yaw is NOT updated here - it's handled by integrate_yaw()
     * and correct_yaw_with_mag() at different rates per Listing 6.2 */
}

/* ============================================================================
 * SECTION 6.4: Yaw Fusion (Listing 6.2)
 * ========================================================================= */

void integrate_yaw(fx_t gz_rad_per_sec, attitude_t *att) {
    /* Pure gyro integration at 100 Hz */
    att->yaw = fx_add(att->yaw, fx_mul(gz_rad_per_sec, DT_FAST_Q16));

    /* Wrap to [-π, π] */
    while (att->yaw > FX_PI) att->yaw = fx_sub(att->yaw, FX_TWO_PI);
    while (att->yaw < -FX_PI) att->yaw = fx_add(att->yaw, FX_TWO_PI);
}

void correct_yaw_with_mag(fx_t mag_heading, attitude_t *att) {
    /* Blend gyro-integrated yaw with magnetometer heading at 10 Hz */
    att->yaw = fx_add(
        fx_mul(ALPHA_YAW_Q16, att->yaw),
        fx_mul(ONE_MINUS_ALPHA_YAW_Q16, mag_heading)
    );

    /* Wrap to [-π, π] */
    while (att->yaw > FX_PI) att->yaw = fx_sub(att->yaw, FX_TWO_PI);
    while (att->yaw < -FX_PI) att->yaw = fx_add(att->yaw, FX_TWO_PI);
}

fx_t compute_mag_heading(const mag_sample_t *mag, const attitude_t *att) {
    /* Section 6.4: Tilt-compensated magnetometer heading
     * psi_mag = atan2(-My*cos(phi) + Mz*sin(phi),
     *                 Mx*cos(theta) + My*sin(theta)*sin(phi) + Mz*sin(theta)*cos(phi))
     * where phi = roll, theta = pitch
     */
    fx_t sin_phi, cos_phi;
    fx_t sin_theta, cos_theta;
    cordic_sincos(att->roll, &sin_phi, &cos_phi);
    cordic_sincos(att->pitch, &sin_theta, &cos_theta);

    /* Y component in horizontal plane: -My*cos(phi) + Mz*sin(phi) */
    fx_t num = fx_add(fx_neg(fx_mul(mag->my, cos_phi)), fx_mul(mag->mz, sin_phi));

    /* X component in horizontal plane: Mx*cos(theta) + My*sin(theta)*sin(phi) + Mz*sin(theta)*cos(phi) */
    fx_t den_term1 = fx_mul(mag->mx, cos_theta);
    fx_t den_term2 = fx_mul(fx_mul(mag->my, sin_theta), sin_phi);
    fx_t den_term3 = fx_mul(fx_mul(mag->mz, sin_theta), cos_phi);
    fx_t den = fx_add(den_term1, fx_add(den_term2, den_term3));

    fx_t heading;
    cordic_atan2(num, den, &heading);
    return heading;
}

/* ============================================================================
 * SECTION 6.5: Altitude Complementary Filter
 * ========================================================================= */

void update_altitude_filter(fx_t accel_z, fx_t baro_alt, fx_t dt, altitude_state_t *alt_state) {
    /* Step 1: Predict using integrated acceleration
     * velocity += accel * dt
     * altitude += velocity * dt
     */
    alt_state->velocity_z = fx_add(alt_state->velocity_z, fx_mul(accel_z, dt));
    fx_t predicted_alt = fx_add(alt_state->altitude, fx_mul(alt_state->velocity_z, dt));

    /* Step 2: Correct toward barometer reading (two-state filter)
     * Blend predicted altitude with barometer measurement
     */
    alt_state->altitude = fx_add(
        fx_mul(ALPHA_ALT_Q16, predicted_alt),
        fx_mul(ONE_MINUS_ALPHA_ALT_Q16, baro_alt)
    );

    /* Also correct velocity toward barometer's rate of change */
    if (alt_state->prev_baro_alt != 0) {  /* Skip first cycle */
        fx_t baro_velocity = fx_div(fx_sub(baro_alt, alt_state->prev_baro_alt), dt);
        alt_state->velocity_z = fx_add(
            fx_mul(ALPHA_ALT_Q16, alt_state->velocity_z),
            fx_mul(ONE_MINUS_ALPHA_ALT_Q16, baro_velocity)
        );
    }

    alt_state->prev_baro_alt = baro_alt;
}

/* ============================================================================
 * SECTION 6.6: Horizontal Position/Velocity Filter (Listing 6.3)
 * ========================================================================= */

void update_horizontal_nav(fx_t accel_n, fx_t accel_e, fx_t dt_nav,
                           const gps_fix_ned_t *fix_if_new, nav_state_t *nav) {
    /* Step 1: Predict - integrate world-frame acceleration into velocity and position */
    nav->vel_n = fx_add(nav->vel_n, fx_mul(accel_n, dt_nav));
    nav->vel_e = fx_add(nav->vel_e, fx_mul(accel_e, dt_nav));

    nav->pos_n = fx_add(nav->pos_n, fx_mul(nav->vel_n, dt_nav));
    nav->pos_e = fx_add(nav->pos_e, fx_mul(nav->vel_e, dt_nav));

    /* Step 2: Correct - blend toward GPS-derived position/velocity if new fix available */
    if (fix_if_new != NULL && fix_if_new->quality_ok) {
        /* Position correction */
        nav->pos_n = fx_add(
            fx_mul(ALPHA_POS_Q16, nav->pos_n),
            fx_mul(ONE_MINUS_ALPHA_POS_Q16, fix_if_new->pos_n)
        );
        nav->pos_e = fx_add(
            fx_mul(ALPHA_POS_Q16, nav->pos_e),
            fx_mul(ONE_MINUS_ALPHA_POS_Q16, fix_if_new->pos_e)
        );

        /* Velocity correction */
        nav->vel_n = fx_add(
            fx_mul(ALPHA_POS_Q16, nav->vel_n),
            fx_mul(ONE_MINUS_ALPHA_POS_Q16, fix_if_new->vel_n)
        );
        nav->vel_e = fx_add(
            fx_mul(ALPHA_POS_Q16, nav->vel_e),
            fx_mul(ONE_MINUS_ALPHA_POS_Q16, fix_if_new->vel_e)
        );
    }
    /* If no new fix (fix_if_new == NULL), estimator degrades gracefully
     * to pure inertial dead-reckoning per Listing 6.3 design */
}
