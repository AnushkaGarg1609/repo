#include "vehicle_sim.h"
#include <math.h>
#include <string.h>

/**
 * VEHICLE_SIM.C
 *
 * Simple quadcopter physics simulation.
 * Implements 6-DOF dynamics: 3D position + 3D orientation.
 */

/* ============================================================================
 * EARTH PARAMETERS (for NED <-> LLA conversion)
 * ========================================================================= */

#define EARTH_RADIUS_M 6371000.0  /* Mean Earth radius (meters) */
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define RAD_TO_DEG (180.0 / 3.14159265358979323846)

/* ============================================================================
 * INITIALIZATION
 * ========================================================================= */

void vehicle_sim_init(vehicle_state_t *state, double home_lat, double home_lon, float home_alt_msl) {
    /* Zero position and velocity */
    state->pos_n = 0.0f;
    state->pos_e = 0.0f;
    state->pos_d = 0.0f;  /* At home altitude */

    state->vel_n = 0.0f;
    state->vel_e = 0.0f;
    state->vel_d = 0.0f;

    /* Level attitude (identity quaternion) */
    state->attitude.w = 1.0f;
    state->attitude.x = 0.0f;
    state->attitude.y = 0.0f;
    state->attitude.z = 0.0f;

    /* Zero angular velocity */
    state->omega_x = 0.0f;
    state->omega_y = 0.0f;
    state->omega_z = 0.0f;

    /* Store home position */
    state->home_lat = home_lat;
    state->home_lon = home_lon;
    state->home_alt_msl = home_alt_msl;
}

/* ============================================================================
 * PHYSICS SIMULATION
 * ========================================================================= */

void vehicle_sim_step(vehicle_state_t *state, const motor_commands_t *motors, float dt) {
    /* ========================================================================
     * 1. Compute thrust and torques from motor commands
     * ===================================================================== */

    /* Each motor produces thrust proportional to PWM
     * Standard X-configuration quadcopter:
     *     0: Front-right (CW)
     *     1: Rear-right  (CCW)
     *     2: Rear-left   (CW)
     *     3: Front-left  (CCW)
     */

    float thrust[4];
    for (int i = 0; i < 4; i++) {
        /* Clamp to valid PWM range */
        uint16_t pwm = motors->motor[i];
        if (pwm < 1000) pwm = 1000;
        if (pwm > 2000) pwm = 2000;

        /* Thrust = k * (PWM - 1000) */
        thrust[i] = THRUST_COEFF * (pwm - 1000);
    }

    /* Total thrust (body Z-axis, upward in body frame = negative body-z) */
    float total_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];

    /* Torques (body frame) for X configuration matching control.h:
     * Motor 0: FL (Front-Left,  CW)   [+x, -y]
     * Motor 1: FR (Front-Right, CCW)  [+x, +y]
     * Motor 2: RR (Rear-Right,  CW)   [-x, +y]
     * Motor 3: RL (Rear-Left,   CCW)  [-x, -y]
     *
     * Roll:  Left motors (0,3) - Right motors (1,2) -> right wing down (positive roll)
     * Pitch: Front motors (0,1) - Rear motors (2,3)  -> nose up (positive pitch)
     * Yaw:   CCW motors (1,3) - CW motors (0,2)     -> clockwise rotation (positive yaw)
     */
    float arm_d = MOTOR_ARM_LENGTH_M * 0.7071068f;
    float torque_roll  = ((thrust[0] + thrust[3]) - (thrust[1] + thrust[2])) * arm_d;
    float torque_pitch = ((thrust[0] + thrust[1]) - (thrust[2] + thrust[3])) * arm_d;
    float torque_yaw   = ((thrust[1] + thrust[3]) - (thrust[0] + thrust[2])) * 0.02f;

    /* ========================================================================
     * 2. Compute linear acceleration (NED frame)
     * ===================================================================== */

    /* Thrust vector in body frame: [0, 0, -total_thrust] (up is negative Z) */
    float thrust_body[3] = {0.0f, 0.0f, -total_thrust};

    /* Rotate thrust to NED frame using current attitude */
    quaternion_t q = state->attitude;

    /* Quaternion rotation: v' = q * v * q^-1
     * For efficiency, use rotation matrix equivalent */
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;

    /* Thrust in NED frame (simplified quaternion rotation) */
    float thrust_ned_n = 2.0f * (qw*qy*thrust_body[2] + qx*qz*thrust_body[2]);
    float thrust_ned_e = 2.0f * (qy*qz*thrust_body[2] - qw*qx*thrust_body[2]);
    float thrust_ned_d = (qw*qw - qx*qx - qy*qy + qz*qz) * thrust_body[2];

    /* Add gravity (NED: down is positive, so +9.81 in D) */
    float accel_n = thrust_ned_n / VEHICLE_MASS_KG;
    float accel_e = thrust_ned_e / VEHICLE_MASS_KG;
    float accel_d = thrust_ned_d / VEHICLE_MASS_KG + GRAVITY;

    /* Drag force (opposing velocity) */
    float speed = sqrtf(state->vel_n * state->vel_n +
                       state->vel_e * state->vel_e +
                       state->vel_d * state->vel_d);
    if (speed > 0.01f) {
        float drag_force = DRAG_COEFFICIENT * speed * speed;
        accel_n -= drag_force * state->vel_n / speed / VEHICLE_MASS_KG;
        accel_e -= drag_force * state->vel_e / speed / VEHICLE_MASS_KG;
        accel_d -= drag_force * state->vel_d / speed / VEHICLE_MASS_KG;
    }

    /* ========================================================================
     * 3. Integrate linear motion
     * ===================================================================== */

    state->vel_n += accel_n * dt;
    state->vel_e += accel_e * dt;
    state->vel_d += accel_d * dt;

    state->pos_n += state->vel_n * dt;
    state->pos_e += state->vel_e * dt;
    state->pos_d += state->vel_d * dt;

    /* Ground collision (simple: can't go below home altitude) */
    if (state->pos_d > 0.0f) {
        state->pos_d = 0.0f;
        state->vel_d = 0.0f;
        /* Also zero horizontal velocity on ground impact */
        state->vel_n *= 0.5f;
        state->vel_e *= 0.5f;
    }

    /* ========================================================================
     * 4. Compute angular acceleration (body frame)
     * ===================================================================== */

    /* Angular drag */
    float omega_mag = sqrtf(state->omega_x * state->omega_x +
                           state->omega_y * state->omega_y +
                           state->omega_z * state->omega_z);
    float drag_torque = ANGULAR_DRAG_COEFFICIENT * omega_mag;

    float alpha_x = (torque_roll  - drag_torque * state->omega_x) / VEHICLE_INERTIA_KG_M2;
    float alpha_y = (torque_pitch - drag_torque * state->omega_y) / VEHICLE_INERTIA_KG_M2;
    float alpha_z = (torque_yaw   - drag_torque * state->omega_z) / VEHICLE_INERTIA_KG_M2;

    /* ========================================================================
     * 5. Integrate angular motion
     * ===================================================================== */

    state->omega_x += alpha_x * dt;
    state->omega_y += alpha_y * dt;
    state->omega_z += alpha_z * dt;

    /* Update attitude quaternion using angular velocity
     * q_dot = 0.5 * q ⊗ [0, ωx, ωy, ωz]
     */
    float half_dt = 0.5f * dt;
    quaternion_t q_dot;
    q_dot.w = half_dt * (-qx * state->omega_x - qy * state->omega_y - qz * state->omega_z);
    q_dot.x = half_dt * ( qw * state->omega_x + qy * state->omega_z - qz * state->omega_y);
    q_dot.y = half_dt * ( qw * state->omega_y + qz * state->omega_x - qx * state->omega_z);
    q_dot.z = half_dt * ( qw * state->omega_z + qx * state->omega_y - qy * state->omega_x);

    state->attitude.w += q_dot.w;
    state->attitude.x += q_dot.x;
    state->attitude.y += q_dot.y;
    state->attitude.z += q_dot.z;

    /* Normalize quaternion to prevent drift */
    float norm = sqrtf(state->attitude.w * state->attitude.w +
                      state->attitude.x * state->attitude.x +
                      state->attitude.y * state->attitude.y +
                      state->attitude.z * state->attitude.z);
    if (norm > 0.0001f) {
        state->attitude.w /= norm;
        state->attitude.x /= norm;
        state->attitude.y /= norm;
        state->attitude.z /= norm;
    }
}

/* ============================================================================
 * SENSOR SIMULATION
 * ========================================================================= */

void vehicle_sim_get_imu(const vehicle_state_t *state, imu_reading_t *out_imu) {
    /* Gyroscope: direct body-frame angular velocity (no integration needed) */
    out_imu->gyro_x = state->omega_x;
    out_imu->gyro_y = state->omega_y;
    out_imu->gyro_z = state->omega_z;

    /* Accelerometer: measures specific force (thrust - gravity) in body frame
     * Start with gravity vector in NED: [0, 0, 9.81]
     * Rotate to body frame using inverse quaternion (conjugate)
     */
    float qw = state->attitude.w;
    float qx = state->attitude.x;
    float qy = state->attitude.y;
    float qz = state->attitude.z;

    /* Gravity in body frame = q^-1 * [0, 0, g] * q */
    float grav_body_x = 2.0f * GRAVITY * (qx*qz - qw*qy);
    float grav_body_y = 2.0f * GRAVITY * (qy*qz + qw*qx);
    float grav_body_z = GRAVITY * (qw*qw - qx*qx - qy*qy + qz*qz);

    /* Accelerometer reads upward reaction force against gravity when level (+1g on z) */
    out_imu->accel_x = grav_body_x;
    out_imu->accel_y = grav_body_y;
    out_imu->accel_z = grav_body_z;

    /* TODO: Add noise model for realistic IMU (Gaussian noise) */
}

void vehicle_sim_get_baro(const vehicle_state_t *state, baro_reading_t *out_baro) {
    /* Altitude MSL = home_alt - pos_d (NED down is negative altitude) */
    out_baro->altitude_msl = state->home_alt_msl - state->pos_d;

    /* Pressure: use barometric formula (optional, not critical for simulation) */
    /* P = P0 * exp(-h / H) where H ≈ 8500m */
    out_baro->pressure_pa = 101325.0f * expf(-out_baro->altitude_msl / 8500.0f);
}

void vehicle_sim_get_gps(const vehicle_state_t *state, gps_reading_t *out_gps) {
    /* Convert NED position to lat/lon/alt */
    ned_to_lla(state->home_lat, state->home_lon, state->home_alt_msl,
               state->pos_n, state->pos_e, state->pos_d,
               &out_gps->latitude, &out_gps->longitude, &out_gps->altitude_msl);

    /* Velocity in NED (GPS provides velocity directly) */
    out_gps->vel_n = state->vel_n;
    out_gps->vel_e = state->vel_e;
    out_gps->vel_d = state->vel_d;

    /* GPS fix status (always good in simulation) */
    out_gps->fix_type = 3;  /* 3D fix */
    out_gps->num_sats = 12;

    /* TODO: Add GPS noise, update rate (5Hz), occasional dropouts */
}

/* ============================================================================
 * COORDINATE CONVERSION
 * ========================================================================= */

void ned_to_lla(double home_lat, double home_lon, float home_alt_msl,
                float pos_n, float pos_e, float pos_d,
                double *out_lat, double *out_lon, float *out_alt_msl) {
    /* Flat-earth approximation (valid for <10km distances)
     * Δlat = north / EARTH_RADIUS
     * Δlon = east / (EARTH_RADIUS * cos(lat))
     */

    double lat_rad = home_lat * DEG_TO_RAD;

    double delta_lat_rad = pos_n / EARTH_RADIUS_M;
    double delta_lon_rad = pos_e / (EARTH_RADIUS_M * cos(lat_rad));

    *out_lat = home_lat + delta_lat_rad * RAD_TO_DEG;
    *out_lon = home_lon + delta_lon_rad * RAD_TO_DEG;
    *out_alt_msl = home_alt_msl - pos_d;  /* NED down is negative altitude */
}
