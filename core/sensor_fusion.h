#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "fixed_point.h"

/**
 * SENSOR_FUSION_H
 *
 * Section 6: Sensor Fusion Theory
 * Implements complementary filters for attitude, altitude, and position estimation.
 */

/* ============================================================================
 * SECTION 6.2 & 6.3: Roll/Pitch Attitude Estimation
 * ========================================================================= */

/**
 * Raw IMU sample structure.
 * All values are in Q16.16 fixed-point.
 */
typedef struct {
    fx_t ax, ay, az;  /* Accelerometer (body frame), arbitrary units (will be normalized) */
    fx_t gx, gy, gz;  /* Gyroscope (body frame), radians/sec */
} imu_sample_t;

/**
 * Attitude state: roll, pitch, yaw in radians (Q16.16).
 * Per Section 6.3, yaw is maintained separately by Section 6.4's magnetometer fusion.
 */
typedef struct {
    fx_t roll;   /* Roll angle (rotation about body X), radians */
    fx_t pitch;  /* Pitch angle (rotation about body Y), radians */
    fx_t yaw;    /* Yaw angle (rotation about body Z), radians */
} attitude_t;

/**
 * Complementary filter constants for roll/pitch fusion.
 * Per Listing 6.1: alpha = 0.98, (1 - alpha) = 0.02
 */
#define ALPHA_ATTITUDE_Q16       64225  /* 0.98 in Q16.16 */
#define ONE_MINUS_ALPHA_ATT_Q16  1311   /* 0.02 in Q16.16 */

/**
 * Time step for 100 Hz attitude loop (0.01 seconds)
 */
#define DT_FAST_Q16 655  /* 0.01 in Q16.16 */

/**
 * Update roll and pitch using complementary filter (Listing 6.1).
 * Blends gyro-integrated estimate with accelerometer-derived angles.
 *
 * @param s Raw IMU sample (accel + gyro)
 * @param att Attitude state to update (roll, pitch modified; yaw unchanged)
 */
void update_attitude_complementary(const imu_sample_t *s, attitude_t *att);

/* ============================================================================
 * SECTION 6.4: Yaw Fusion (Tilt-Compensated Magnetometer)
 * ========================================================================= */

/**
 * Raw magnetometer sample (body frame), arbitrary units.
 * Should be calibrated per Section 5.5 before passing to fusion.
 */
typedef struct {
    fx_t mx, my, mz;  /* Magnetometer (body frame), Q16.16 */
} mag_sample_t;

/**
 * Yaw fusion constants.
 * Per Listing 6.2: alpha_yaw = 0.90, (1 - alpha_yaw) = 0.10
 * Lower alpha than roll/pitch because yaw has no fast correction term.
 */
#define ALPHA_YAW_Q16           58982  /* 0.90 in Q16.16 */
#define ONE_MINUS_ALPHA_YAW_Q16  6554  /* 0.10 in Q16.16 */

/**
 * Time step for 10 Hz medium-rate loop (0.1 seconds)
 */
#define DT_MEDIUM_Q16 6554  /* 0.1 in Q16.16 */

/**
 * Integrate yaw with gyroscope only (called at 100 Hz, per Listing 6.2).
 * @param gz_dps Yaw rate in radians/sec (Q16.16)
 * @param att Attitude state (yaw modified)
 */
void integrate_yaw(fx_t gz_rad_per_sec, attitude_t *att);

/**
 * Correct yaw with magnetometer heading (called at 10 Hz, per Listing 6.2).
 * @param mag_heading Tilt-compensated heading from magnetometer (radians, Q16.16)
 * @param att Attitude state (yaw modified)
 */
void correct_yaw_with_mag(fx_t mag_heading, attitude_t *att);

/**
 * Compute tilt-compensated magnetometer heading (Section 6.4).
 * Requires current roll/pitch estimate.
 *
 * @param mag Raw (calibrated) magnetometer sample
 * @param att Current attitude (roll, pitch used for tilt compensation)
 * @return Heading in radians (Q16.16), range [-π, π]
 */
fx_t compute_mag_heading(const mag_sample_t *mag, const attitude_t *att);

/* ============================================================================
 * SECTION 6.5: Altitude Estimation (Barometer + Accelerometer)
 * ========================================================================= */

/**
 * Altitude state: estimated altitude and vertical velocity.
 * Two-state complementary filter per Section 6.5.
 */
typedef struct {
    fx_t altitude;      /* Estimated altitude (meters, Q16.16) */
    fx_t velocity_z;    /* Estimated vertical velocity (m/s, Q16.16) */
    fx_t prev_baro_alt; /* Previous barometer reading for derivative calculation */
} altitude_state_t;

/**
 * Altitude filter constants.
 * Per Section 6.5: alpha_h = 0.90 (same as yaw, slower correction than attitude)
 */
#define ALPHA_ALT_Q16           58982  /* 0.90 in Q16.16 */
#define ONE_MINUS_ALPHA_ALT_Q16  6554  /* 0.10 in Q16.16 */

/**
 * Update altitude and vertical velocity using complementary filter (Section 6.5).
 * Blends double-integrated vertical acceleration with barometer reading.
 *
 * @param accel_z World-frame vertical acceleration (m/s², Q16.16)
 * @param baro_alt Barometer-derived altitude (meters, Q16.16)
 * @param dt Time step (seconds, Q16.16) - typically DT_MEDIUM_Q16
 * @param alt_state Altitude state to update
 */
void update_altitude_filter(fx_t accel_z, fx_t baro_alt, fx_t dt, altitude_state_t *alt_state);

/* ============================================================================
 * SECTION 6.6: Horizontal Position/Velocity Estimation (GPS + Accelerometer)
 * ========================================================================= */

/**
 * GPS fix in local NED frame (Section 2.5).
 * Converted from lat/lon at GPS read time, not inside the filter.
 */
typedef struct {
    fx_t pos_n, pos_e;  /* North, East position (meters, Q16.16) */
    fx_t vel_n, vel_e;  /* North, East velocity (m/s, Q16.16) from Doppler */
    bool quality_ok;    /* True if fix meets quality gate (Section 5.7) */
} gps_fix_ned_t;

/**
 * Horizontal navigation state: position and velocity in NED frame.
 * Two-state complementary filter per Listing 6.3.
 */
typedef struct {
    fx_t pos_n, pos_e;  /* Estimated North, East position (meters, Q16.16) */
    fx_t vel_n, vel_e;  /* Estimated North, East velocity (m/s, Q16.16) */
} nav_state_t;

/**
 * Position filter constants.
 * Per Listing 6.3: alpha_pos = 0.90 (GPS is the only absolute horizontal reference)
 */
#define ALPHA_POS_Q16           58982  /* 0.90 in Q16.16 */
#define ONE_MINUS_ALPHA_POS_Q16  6554  /* 0.10 in Q16.16 */

/**
 * Time step for 5 Hz navigation loop (0.2 seconds)
 */
#define DT_NAV_Q16 13107  /* 0.2 in Q16.16 */

/**
 * Update horizontal position and velocity using complementary filter (Listing 6.3).
 * Blends integrated world-frame acceleration with GPS position/velocity.
 *
 * @param accel_n World-frame North acceleration (m/s², Q16.16)
 * @param accel_e World-frame East acceleration (m/s², Q16.16)
 * @param dt_nav Time step (seconds, Q16.16) - typically DT_NAV_Q16
 * @param fix_if_new Pointer to new GPS fix if available, NULL otherwise
 * @param nav Navigation state to update
 */
void update_horizontal_nav(fx_t accel_n, fx_t accel_e, fx_t dt_nav,
                           const gps_fix_ned_t *fix_if_new, nav_state_t *nav);

#endif /* SENSOR_FUSION_H */
