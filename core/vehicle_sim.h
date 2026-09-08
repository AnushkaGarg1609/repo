#ifndef VEHICLE_SIM_H
#define VEHICLE_SIM_H

#include <stdint.h>
#include <stdbool.h>
#include "rotation.h"

/**
 * VEHICLE_SIM.H
 *
 * Simple quadcopter physics simulation for integration testing.
 * Models thrust, torque, drag, and gravity to produce realistic sensor readings.
 */

/* ============================================================================
 * VEHICLE PHYSICAL PARAMETERS
 * ========================================================================= */

/**
 * Quadcopter mass (kg).
 * Typical 250-450mm racing quad: 0.5-1.5 kg
 */
#define VEHICLE_MASS_KG 0.8f

/**
 * Moment of inertia (kg·m²).
 * Simplified: assume uniform disk approximation.
 * Typical small quad: 0.001-0.005 kg·m²
 */
#define VEHICLE_INERTIA_KG_M2 0.003f

/**
 * Motor arm length from center (meters).
 * Typical 250mm quad: ~0.125m (half of diagonal)
 */
#define MOTOR_ARM_LENGTH_M 0.125f

/**
 * Drag coefficient (N·s²/m²).
 * Linear drag model: F_drag = -k * v²
 */
#define DRAG_COEFFICIENT 0.1f

/**
 * Angular drag coefficient (N·m·s²/rad²).
 * Torque_drag = -k * ω²
 */
#define ANGULAR_DRAG_COEFFICIENT 0.001f

/**
 * Gravity (m/s²).
 */
#define GRAVITY 9.81f

/**
 * Motor thrust coefficient (N/µs PWM).
 * Maps PWM pulse width above 1000µs to thrust force.
 * At 1500µs (500µs delta): 4 motors × (0.003924 × 500) = 7.848 N (exact hover for 0.8kg mass).
 */
#define THRUST_COEFF 0.003924f

/**
 * Motor torque coefficient (N·m/µs PWM).
 * Reactive torque ratio from propeller drag.
 */
#define TORQUE_COEFF 0.0000785f

/* ============================================================================
 * VEHICLE STATE
 * ========================================================================= */

/**
 * Complete vehicle state for physics simulation.
 * All quantities in NED (North-East-Down) frame.
 */
typedef struct {
    /* Position (NED frame, meters) */
    float pos_n, pos_e, pos_d;

    /* Velocity (NED frame, m/s) */
    float vel_n, vel_e, vel_d;

    /* Attitude (quaternion: body frame relative to NED) */
    quaternion_t attitude;

    /* Angular velocity (body frame, rad/s) */
    float omega_x, omega_y, omega_z;

    /* Home position (for GPS simulation) */
    double home_lat, home_lon, home_alt_msl;

} vehicle_state_t;

/* ============================================================================
 * MOTOR COMMANDS
 * ========================================================================= */

/**
 * Motor PWM commands (microseconds).
 * Standard: 1000µs = min, 1500µs = mid, 2000µs = max
 */
typedef struct {
    uint16_t motor[4];  /* Front-right, rear-right, rear-left, front-left */
} motor_commands_t;

/* ============================================================================
 * SIMULATED SENSOR OUTPUTS
 * ========================================================================= */

/**
 * Simulated IMU reading (accelerometer + gyroscope).
 */
typedef struct {
    float accel_x, accel_y, accel_z;  /* Body frame, m/s² */
    float gyro_x, gyro_y, gyro_z;     /* Body frame, rad/s */
} imu_reading_t;

/**
 * Simulated barometer reading.
 */
typedef struct {
    float altitude_msl;  /* Meters above mean sea level */
    float pressure_pa;   /* Pressure in Pascals (optional) */
} baro_reading_t;

/**
 * Simulated GPS reading.
 */
typedef struct {
    double latitude;     /* Degrees */
    double longitude;    /* Degrees */
    float altitude_msl;  /* Meters above MSL */
    float vel_n, vel_e, vel_d;  /* NED velocity, m/s */
    uint8_t fix_type;    /* 0=no fix, 3=3D fix */
    uint8_t num_sats;    /* Number of satellites */
} gps_reading_t;

/* ============================================================================
 * SIMULATION FUNCTIONS
 * ========================================================================= */

/**
 * Initialize vehicle state at a given position.
 * Vehicle starts on ground, level attitude, zero velocity.
 *
 * @param state Vehicle state to initialize
 * @param home_lat Home latitude (degrees)
 * @param home_lon Home longitude (degrees)
 * @param home_alt_msl Home altitude MSL (meters)
 */
void vehicle_sim_init(vehicle_state_t *state, double home_lat, double home_lon, float home_alt_msl);

/**
 * Step the vehicle physics simulation forward by dt seconds.
 * Updates position, velocity, attitude, and angular rates based on motor commands.
 *
 * @param state Vehicle state (modified)
 * @param motors Motor PWM commands (1000-2000 µs)
 * @param dt Time step (seconds)
 */
void vehicle_sim_step(vehicle_state_t *state, const motor_commands_t *motors, float dt);

/**
 * Get simulated IMU reading from current vehicle state.
 * Includes gravity and vehicle acceleration in body frame.
 *
 * @param state Current vehicle state
 * @param out_imu Output IMU reading
 */
void vehicle_sim_get_imu(const vehicle_state_t *state, imu_reading_t *out_imu);

/**
 * Get simulated barometer reading.
 *
 * @param state Current vehicle state
 * @param out_baro Output barometer reading
 */
void vehicle_sim_get_baro(const vehicle_state_t *state, baro_reading_t *out_baro);

/**
 * Get simulated GPS reading.
 * Converts NED position relative to home into lat/lon/alt.
 *
 * @param state Current vehicle state
 * @param out_gps Output GPS reading
 */
void vehicle_sim_get_gps(const vehicle_state_t *state, gps_reading_t *out_gps);

/**
 * Convert NED position (relative to home) to lat/lon/alt.
 * Simple flat-earth approximation for local navigation.
 *
 * @param home_lat Home latitude (degrees)
 * @param home_lon Home longitude (degrees)
 * @param home_alt_msl Home altitude MSL (meters)
 * @param pos_n North position (meters)
 * @param pos_e East position (meters)
 * @param pos_d Down position (meters, negative = up)
 * @param out_lat Output latitude (degrees)
 * @param out_lon Output longitude (degrees)
 * @param out_alt_msl Output altitude MSL (meters)
 */
void ned_to_lla(double home_lat, double home_lon, float home_alt_msl,
                float pos_n, float pos_e, float pos_d,
                double *out_lat, double *out_lon, float *out_alt_msl);

#endif /* VEHICLE_SIM_H */
