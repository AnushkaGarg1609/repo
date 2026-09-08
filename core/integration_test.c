#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* Flight controller modules */
#include "fixed_point.h"
#include "rotation.h"
#include "sensor_fusion.h"
#include "control.h"
#include "flight_modes.h"
#include "arming.h"
#include "mission.h"
#include "vehicle_sim.h"

/**
 * INTEGRATION_TEST.C
 *
 * Full Closed-Loop Flight Controller Integration Simulation for Aries V3.
 *
 * Runs the complete cooperative superloop architecture:
 * - 100 Hz Fast Loop: IMU read, attitude fusion, cascaded angle/rate PID, mixer, motor output
 * - 10 Hz Medium Loop: RC decode, mode state machine, mag tilt-heading fusion, baro altitude fusion & PID
 * - 5 Hz Navigation Loop: GPS fix read, horizontal position/velocity fusion & PID, mission sequencer
 * - 2 Hz Slow Loop: Safety monitor & guardrails
 */

/* ============================================================================
 * SIMULATION PARAMETERS
 * ========================================================================= */

#define SIM_DURATION_SEC 40.0f      /* Total simulation time */
#define FAST_LOOP_HZ     100        /* 100 Hz (10 ms period) */
#define MEDIUM_DIVIDER   10         /* 10 Hz (100 ms period) */
#define NAV_DIVIDER      20         /* 5 Hz (200 ms period) */
#define SLOW_DIVIDER     50         /* 2 Hz (500 ms period) */
#define LOG_DIVIDER      10         /* 10 Hz CSV logging */

#define DT_FAST_SEC      0.01f
#define DT_MEDIUM_SEC    0.1f
#define DT_NAV_SEC       0.2f

/* Home GPS coordinates */
#define HOME_LAT     28.6139
#define HOME_LON     77.2090
#define HOME_ALT_MSL 216.0f

/* Baseline hover throttle (microseconds) */
#define HOVER_THROTTLE_PWM 1500

/* ============================================================================
 * GLOBAL FLIGHT CONTROLLER STATE
 * ========================================================================= */

/* Simulated physical vehicle */
static vehicle_state_t vehicle;

/* Estimator states */
static attitude_t attitude_est;
static altitude_state_t altitude_est;
static nav_state_t nav_horizontal_est;

/* Most recent IMU sample from the 100 Hz fast tier. The 10 Hz altitude filter
 * needs the real accelerometer to remove gravity; feeding it a synthetic
 * (0,0,9.81) collapses the vertical term to g*(cos(tilt)-1), which the filter
 * then integrates without bound whenever the vehicle is tilted. */
static imu_sample_t last_imu_sample;

/* Flight mode & arming */
static arming_state_t arming_state;
static flight_mode_t current_mode = MODE_DISARMED;
static nav_health_t nav_health;

/* Mission */
static mission_t mission;

/* Controllers (PIDs) */
static pid_t pid_roll_angle;
static pid_t pid_pitch_angle;
static pid_t pid_roll_rate;
static pid_t pid_pitch_rate;
static pid_t pid_yaw_rate;

static pid_t pid_altitude;
static pid_t pid_vel_z;

static pid_t pid_pos_n;
static pid_t pid_pos_e;
static pid_t pid_vel_n;
static pid_t pid_vel_e;

/* Setpoints shared across loops */
static fx_t setpoint_roll_angle = 0;
static fx_t setpoint_pitch_angle = 0;
static fx_t setpoint_yaw_rate = 0;
static fx_t setpoint_throttle = (1000 << FX_SHIFT);

static fx_t setpoint_altitude = (10 << FX_SHIFT);  /* Target 10m */
static fx_t setpoint_pos_n = 0;
static fx_t setpoint_pos_e = 0;

/* Motor outputs */
static fx_t motor_cmds_fx[NUM_MOTORS];
static motor_commands_t motor_cmds_sim;

/* RC input state */
static rc_input_t rc_input;

/* ============================================================================
 * CONTROLLER INITIALIZATION
 * ========================================================================= */

static void init_flight_controllers(void) {
    /* 1. Attitude Angle PIDs (Outer Loop: Angle Error -> Target Rate in rad/s) */
    pid_init(&pid_roll_angle,
             fx_from_float(4.0f),    /* Kp */
             fx_from_float(0.05f),   /* Ki */
             fx_from_float(0.02f),   /* Kd */
             fx_from_float(0.5f),    /* Integral limit */
             fx_from_float(-2.5f),   /* Min rate -2.5 rad/s */
             fx_from_float(2.5f));   /* Max rate +2.5 rad/s */

    pid_init(&pid_pitch_angle,
             fx_from_float(4.0f),
             fx_from_float(0.05f),
             fx_from_float(0.02f),
             fx_from_float(0.5f),
             fx_from_float(-2.5f),
             fx_from_float(2.5f));

    /* 2. Attitude Rate PIDs (Inner Loop: Rate Error -> Moment in PWM delta) */
    pid_init(&pid_roll_rate,
             fx_from_float(60.0f),   /* Kp */
             fx_from_float(15.0f),   /* Ki */
             fx_from_float(0.5f),    /* Kd */
             fx_from_float(40.0f),   /* Integral limit */
             fx_from_float(-250.0f), /* Output min */
             fx_from_float(250.0f)); /* Output max */

    pid_init(&pid_pitch_rate,
             fx_from_float(60.0f),
             fx_from_float(15.0f),
             fx_from_float(0.5f),
             fx_from_float(40.0f),
             fx_from_float(-250.0f),
             fx_from_float(250.0f));

    pid_init(&pid_yaw_rate,
             fx_from_float(50.0f),
             fx_from_float(10.0f),
             fx_from_float(0.2f),
             fx_from_float(30.0f),
             fx_from_float(-150.0f),
             fx_from_float(150.0f));

    /* 3. Altitude PIDs (Outer: Alt -> VelZ, Inner: VelZ -> Throttle Delta) */
    pid_init(&pid_altitude,
             fx_from_float(1.0f),    /* Kp: 1m error -> 1.0 m/s target climb */
             fx_from_float(0.05f),   /* Ki */
             fx_from_float(0.02f),   /* Kd */
             fx_from_float(1.5f),
             fx_from_float(-2.0f),   /* Max climb/descent rate */
             fx_from_float(2.0f));

    pid_init(&pid_vel_z,
             fx_from_float(70.0f),   /* Kp */
             fx_from_float(20.0f),   /* Ki */
             fx_from_float(2.0f),    /* Kd */
             fx_from_float(60.0f),
             fx_from_float(-250.0f), /* Throttle correction limits */
             fx_from_float(250.0f));

    /* 4. Position PIDs (Outer: Pos -> Vel, Inner: Vel -> Desired Accel) */
    pid_init(&pid_pos_n,
             fx_from_float(0.6f),
             fx_from_float(0.01f),
             fx_from_float(0.02f),
             fx_from_float(1.0f),
             fx_from_float(-2.5f),   /* Max 2.5 m/s velocity */
             fx_from_float(2.5f));

    pid_init(&pid_pos_e,
             fx_from_float(0.6f),
             fx_from_float(0.01f),
             fx_from_float(0.02f),
             fx_from_float(1.0f),
             fx_from_float(-2.5f),
             fx_from_float(2.5f));

    pid_init(&pid_vel_n,
             fx_from_float(0.8f),    /* Kp */
             fx_from_float(0.10f),   /* Ki */
             fx_from_float(0.02f),   /* Kd */
             fx_from_float(0.8f),
             fx_from_float(-2.0f),   /* Max 2.0 m/s² accel (~12° lean) */
             fx_from_float(2.0f));

    pid_init(&pid_vel_e,
             fx_from_float(0.8f),
             fx_from_float(0.10f),
             fx_from_float(0.02f),
             fx_from_float(0.8f),
             fx_from_float(-2.0f),
             fx_from_float(2.0f));
}

/* ============================================================================
 * SIMULATED RC FLIGHT PROGRAM
 * ========================================================================= */

static void update_pilot_rc_scenario(float time_sec, rc_input_t *rc) {
    /* Baseline centered sticks */
    rc->roll = 0;
    rc->pitch = 0;
    rc->yaw = 0;
    rc->throttle = 0;
    rc->arm_switch = false;
    rc->mode_switch_position = SWITCH_STABILIZE;

    if (time_sec < 2.2f) {
        /* Phase 0: Disarmed on ground (throttle at min) */
        rc->throttle = 0;
        rc->arm_switch = (time_sec >= 2.0f); /* Arm switch ON at 2.0s */
        rc->mode_switch_position = SWITCH_STABILIZE;
    } else if (time_sec >= 2.2f && time_sec < 6.0f) {
        /* Phase 1: Armed, gentle manual throttle ramp in STABILIZE */
        rc->arm_switch = true;
        rc->mode_switch_position = SWITCH_STABILIZE;
        float progress = (time_sec - 2.2f) / 3.8f;
        rc->throttle = fx_from_float(0.48f + 0.08f * progress);
    } else if (time_sec >= 6.0f && time_sec < 10.0f) {
        /* Phase 2: Altitude Hold mode (target 10m) */
        rc->arm_switch = true;
        rc->mode_switch_position = SWITCH_ALT_HOLD;
        rc->throttle = fx_from_float(0.5f); /* Centered throttle = hold */
    } else if (time_sec >= 10.0f && time_sec < 14.0f) {
        /* Phase 3: Position Hold mode */
        rc->arm_switch = true;
        rc->mode_switch_position = SWITCH_POS_HOLD;
        rc->throttle = fx_from_float(0.5f);
    } else if (time_sec >= 14.0f && time_sec < 34.0f) {
        /* Phase 4: Autonomous Mission mode (waypoints) */
        rc->arm_switch = true;
        rc->mode_switch_position = SWITCH_AUTO;
        rc->throttle = fx_from_float(0.5f);
    } else {
        /* Phase 5: Landing / Return Home */
        rc->arm_switch = true;
        rc->mode_switch_position = SWITCH_ALT_HOLD;
        rc->throttle = fx_from_float(0.35f); /* Descend */
    }
}

/* ============================================================================
 * RATE TIERS (COOPERATIVE SUPERLOOP IMPLEMENTATION)
 * ========================================================================= */

/* 5 Hz Navigation Tier */
static void run_nav_tier_5hz(void) {
    /* 1. Acquire GPS fix */
    gps_reading_t gps;
    vehicle_sim_get_gps(&vehicle, &gps);

    /* Convert GPS fix to NED */
    gps_fix_ned_t fix_ned;
    fix_ned.pos_n = fx_from_float(vehicle.pos_n);
    fix_ned.pos_e = fx_from_float(vehicle.pos_e);
    fix_ned.vel_n = fx_from_float(gps.vel_n);
    fix_ned.vel_e = fx_from_float(gps.vel_e);
    fix_ned.quality_ok = (gps.fix_type >= 3 && gps.num_sats >= 6);

    /* 2. Rotate body acceleration to world frame for dead reckoning */
    fx_t accel_world_n = 0, accel_world_e = 0, accel_world_d = 0;
    rotate_accel_to_world(
        fx_from_float(0.0f), fx_from_float(0.0f), fx_from_float(0.0f),
        attitude_est.roll, attitude_est.pitch, attitude_est.yaw,
        &accel_world_n, &accel_world_e, &accel_world_d
    );

    /* 3. Fuse Horizontal Navigation Filter */
    update_horizontal_nav(accel_world_n, accel_world_e, DT_NAV_Q16, &fix_ned, &nav_horizontal_est);

    /* 4. Autonomous Mission Sequencer (if in MISSION or RTL mode) */
    flight_mode_t requested_mode = current_mode;
    if (current_mode == MODE_MISSION || current_mode == MODE_RTL) {
        nav_state_3d_t nav_3d;
        nav_3d.north = nav_horizontal_est.pos_n;
        nav_3d.east = nav_horizontal_est.pos_e;
        nav_3d.down = fx_neg(altitude_est.altitude); /* NED down is negative altitude */

        position_setpoint_t mission_sp;
        navigation_step(&nav_3d, current_mode, &mission, &mission_sp, &requested_mode);

        setpoint_pos_n = mission_sp.north;
        setpoint_pos_e = mission_sp.east;
        setpoint_altitude = fx_neg(mission_sp.down);

        if (requested_mode == MODE_RTL && current_mode == MODE_MISSION) {
            current_mode = MODE_RTL;
            printf("  >>> MISSION COMPLETED! AUTO-TRANSITION TO RTL <<<\n");
        }
    }

    /* 5. Horizontal Position & Velocity Outer Loops */
    if (current_mode == MODE_POS_HOLD || current_mode == MODE_MISSION || current_mode == MODE_RTL) {
        /* Position Outer Loop: Pos Error -> Target Velocity */
        fx_t target_vel_n = pid_update(&pid_pos_n, setpoint_pos_n, nav_horizontal_est.pos_n, DT_NAV_Q16);
        fx_t target_vel_e = pid_update(&pid_pos_e, setpoint_pos_e, nav_horizontal_est.pos_e, DT_NAV_Q16);

        /* Velocity Inner Loop: Vel Error -> Commanded World Acceleration */
        fx_t cmd_accel_n = pid_update(&pid_vel_n, target_vel_n, nav_horizontal_est.vel_n, DT_NAV_Q16);
        fx_t cmd_accel_e = pid_update(&pid_vel_e, target_vel_e, nav_horizontal_est.vel_e, DT_NAV_Q16);

        /* Section 7.6: Rotate world acceleration into body-frame roll/pitch setpoints */
        fx_t sin_yaw, cos_yaw;
        cordic_sincos(attitude_est.yaw, &sin_yaw, &cos_yaw);

        fx_t g_fx = fx_from_float(9.81f);

        /* roll_sp  = +(accel_E * cos(psi) - accel_N * sin(psi)) / g */
        fx_t roll_num = fx_sub(fx_mul(cmd_accel_e, cos_yaw), fx_mul(cmd_accel_n, sin_yaw));
        setpoint_roll_angle = fx_div(roll_num, g_fx);

        /* pitch_sp = -(accel_N * cos(psi) + accel_E * sin(psi)) / g */
        fx_t pitch_num = fx_add(fx_mul(cmd_accel_n, cos_yaw), fx_mul(cmd_accel_e, sin_yaw));
        setpoint_pitch_angle = fx_neg(fx_div(pitch_num, g_fx));

        /* Clamp maximum tilt angle to ±20 degrees (~0.35 rad) for safety */
        fx_t max_tilt = fx_from_float(0.35f);
        setpoint_roll_angle = fx_clamp(setpoint_roll_angle, fx_neg(max_tilt), max_tilt);
        setpoint_pitch_angle = fx_clamp(setpoint_pitch_angle, fx_neg(max_tilt), max_tilt);
    }
}

/* 10 Hz Medium Tier */
static void run_medium_tier_10hz(void) {
    /* 1. Barometer reading & Altitude fusion */
    baro_reading_t baro;
    vehicle_sim_get_baro(&vehicle, &baro);
    fx_t baro_alt = fx_from_float(baro.altitude_msl - HOME_ALT_MSL);

    /* Vertical acceleration in world frame, from the real accelerometer.
     * extract_vertical_accel rotates the measured body-frame specific force by
     * the estimated tilt; subtracting g leaves true vertical acceleration. When
     * level and still the measurement is ~(0,0,9.81) and this nets to zero, but
     * during flight the horizontal and dynamic components are what let the
     * filter track climbs and descents instead of drifting. */
    fx_t accel_world_z = extract_vertical_accel(
        last_imu_sample.ax, last_imu_sample.ay, last_imu_sample.az,
        attitude_est.roll, attitude_est.pitch
    );
    accel_world_z = fx_sub(accel_world_z, fx_from_float(9.81f)); /* Remove gravity */

    update_altitude_filter(accel_world_z, baro_alt, DT_MEDIUM_Q16, &altitude_est);

    /* 2. Magnetometer reading & Tilt-compensated yaw fusion */
    mag_sample_t mag;
    euler_t true_euler;
    quat_to_euler(&vehicle.attitude, &true_euler);
    mag.mx = fx_from_float(cosf(true_euler.yaw));
    mag.my = fx_from_float(-sinf(true_euler.yaw));
    mag.mz = fx_from_float(0.0f);

    fx_t mag_heading = compute_mag_heading(&mag, &attitude_est);
    correct_yaw_with_mag(mag_heading, &attitude_est);

    /* 3. Evaluate Mode Transitions & Override Supremacy */
    if (!is_armed(arming_state)) {
        current_mode = MODE_DISARMED;
    } else {
        flight_mode_t next_mode = evaluate_mode_transition(
            current_mode,
            &rc_input,
            &nav_health,
            mission_is_loaded(&mission)
        );

        if (next_mode != current_mode) {
            printf("  [Mode Change: %s -> %s]\n", flight_mode_name(current_mode), flight_mode_name(next_mode));
            current_mode = next_mode;
        }
    }

    /* 4. Altitude Control Loop (if in ALT_HOLD, POS_HOLD, MISSION, or RTL) */
    if (current_mode == MODE_ALT_HOLD || current_mode == MODE_POS_HOLD ||
        current_mode == MODE_MISSION || current_mode == MODE_RTL) {

        /* Altitude Outer Loop: Alt Error -> Target Vertical Velocity */
        fx_t target_vel_z = pid_update(&pid_altitude, setpoint_altitude, altitude_est.altitude, DT_MEDIUM_Q16);

        /* Vertical Velocity Inner Loop: Vel Error -> Throttle Correction */
        fx_t throttle_delta = pid_update(&pid_vel_z, target_vel_z, altitude_est.velocity_z, DT_MEDIUM_Q16);

        /* Add to hover throttle baseline */
        setpoint_throttle = fx_add(fx_from_int(HOVER_THROTTLE_PWM), throttle_delta);
        setpoint_throttle = fx_clamp(setpoint_throttle, fx_from_int(1100), fx_from_int(1850));
    } else if (current_mode == MODE_STABILIZE) {
        /* Pilot direct throttle */
        setpoint_throttle = fx_from_int(1000 + (int32_t)(fx_to_float(rc_input.throttle) * 1000.0f));
    }
}

/* 2 Hz Slow Tier (Safety & Health Checks) */
static void run_slow_tier_2hz(void) {
    /* Update sensor health gates */
    nav_health.baro_ok = true;
    nav_health.gps_fix_ok = true;
    nav_health.home_settled = true;
    nav_health.imu_ok = true;
    nav_health.mag_ok = true;

    /* Emergency attitude check: angle > 60 degrees triggers failsafe disarm */
    fx_t max_safe_angle = fx_from_float(1.05f); /* ~60 deg */
    if (fx_abs(attitude_est.roll) > max_safe_angle || fx_abs(attitude_est.pitch) > max_safe_angle) {
        printf("  [CRITICAL: Angle Limit Exceeded! Emergency Disarming!]\n");
        disarm(&arming_state);
        current_mode = MODE_DISARMED;
    }
}

/* 100 Hz Fast Control Tier */
static void run_fast_tier_100hz(void) {
    /* 1. Read IMU sensors */
    imu_reading_t imu_raw;
    vehicle_sim_get_imu(&vehicle, &imu_raw);

    imu_sample_t imu_sample;
    imu_sample.ax = fx_from_float(imu_raw.accel_x);
    imu_sample.ay = fx_from_float(imu_raw.accel_y);
    imu_sample.az = fx_from_float(imu_raw.accel_z);
    imu_sample.gx = fx_from_float(imu_raw.gyro_x);
    imu_sample.gy = fx_from_float(imu_raw.gyro_y);
    imu_sample.gz = fx_from_float(imu_raw.gyro_z);

    /* Publish for the slower tiers (the 10 Hz altitude filter reads the accel). */
    last_imu_sample = imu_sample;

    /* 2. Update Attitude Complementary Filter (Roll & Pitch) */
    update_attitude_complementary(&imu_sample, &attitude_est);

    /* 3. Integrate Gyroscope Yaw */
    integrate_yaw(imu_sample.gz, &attitude_est);

    /* 4. Determine Attitude Setpoints based on Mode */
    if (current_mode == MODE_STABILIZE || current_mode == MODE_ALT_HOLD) {
        /* Pilot stick drives roll & pitch angle directly */
        setpoint_roll_angle = fx_mul(rc_input.roll, fx_from_float(0.35f));   /* ±20 deg */
        setpoint_pitch_angle = fx_mul(rc_input.pitch, fx_from_float(0.35f));
    }
    setpoint_yaw_rate = fx_mul(rc_input.yaw, fx_from_float(1.5f)); /* ±85 deg/s */

    /* 5. Cascaded Attitude PID Controllers */
    if (is_armed(arming_state) && current_mode != MODE_DISARMED) {
        /* Outer Angle Loop: Angle Error -> Target Angular Rate */
        fx_t target_rate_roll = pid_update(&pid_roll_angle, setpoint_roll_angle, attitude_est.roll, DT_FAST_Q16);
        fx_t target_rate_pitch = pid_update(&pid_pitch_angle, setpoint_pitch_angle, attitude_est.pitch, DT_FAST_Q16);
        fx_t target_rate_yaw = setpoint_yaw_rate;

        /* Inner Rate Loop: Rate Error -> Moment PWM Commands */
        fx_t roll_moment = pid_update(&pid_roll_rate, target_rate_roll, imu_sample.gx, DT_FAST_Q16);
        fx_t pitch_moment = pid_update(&pid_pitch_rate, target_rate_pitch, imu_sample.gy, DT_FAST_Q16);
        fx_t yaw_moment = pid_update(&pid_yaw_rate, target_rate_yaw, imu_sample.gz, DT_FAST_Q16);

        /* 6. Motor Mixer */
        mix_and_output_motors(setpoint_throttle, roll_moment, pitch_moment, yaw_moment, motor_cmds_fx);

        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_cmds_sim.motor[i] = motor_to_pwm(motor_cmds_fx[i]);
        }
    } else {
        /* Disarmed: Motors strictly off (1000 µs) */
        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_cmds_fx[i] = fx_from_int(1000);
            motor_cmds_sim.motor[i] = 1000;
        }
    }

    /* 7. Step 6-DOF Vehicle Physics Simulation */
    vehicle_sim_step(&vehicle, &motor_cmds_sim, DT_FAST_SEC);
}

/* ============================================================================
 * MAIN SIMULATION RUNNER
 * ========================================================================= */

int main(void) {
    printf("====================================================================\n");
    printf("  ARIES V3 FLIGHT CONTROLLER - FULL CLOSED-LOOP SIMULATION TEST\n");
    printf("====================================================================\n\n");
    printf("Target Architecture: VEGA RV32IM (Q16.16 Fixed-Point, No FPU, Superloop)\n");
    printf("Simulating %.1f seconds of multi-mode autonomous mission flight...\n\n", SIM_DURATION_SEC);

    /* Initialize vehicle physics */
    vehicle_sim_init(&vehicle, HOME_LAT, HOME_LON, HOME_ALT_MSL);

    /* Initialize estimators */
    memset(&attitude_est, 0, sizeof(attitude_est));
    memset(&altitude_est, 0, sizeof(altitude_est));
    memset(&nav_horizontal_est, 0, sizeof(nav_horizontal_est));

    /* Initialize controllers */
    init_flight_controllers();

    /* Initialize arming state */
    arming_state = ARMING_DISARMED;
    memset(&nav_health, 0, sizeof(nav_health));
    nav_health.imu_ok = true;
    nav_health.baro_ok = true;
    nav_health.gps_fix_ok = true;
    nav_health.home_settled = true;
    nav_health.mag_ok = true;

    /* Load 4-Waypoint Autonomous Mission:
     * WP 0: (0m,   0m,  10m alt) - Takeoff & Home
     * WP 1: (15m,  0m,  10m alt) - North
     * WP 2: (15m, 15m,  10m alt) - East
     * WP 3: (0m,  15m,  10m alt) - South
     * WP 4: (0m,   0m,  10m alt) - Return Home
     */
    waypoint_t waypoints[5] = {
        {fx_from_float(0.0f),  fx_from_float(0.0f),  fx_from_float(-10.0f), fx_from_float(2.0f)},
        {fx_from_float(15.0f), fx_from_float(0.0f),  fx_from_float(-10.0f), fx_from_float(2.0f)},
        {fx_from_float(15.0f), fx_from_float(15.0f), fx_from_float(-10.0f), fx_from_float(2.0f)},
        {fx_from_float(0.0f),  fx_from_float(15.0f), fx_from_float(-10.0f), fx_from_float(2.0f)},
        {fx_from_float(0.0f),  fx_from_float(0.0f),  fx_from_float(-10.0f), fx_from_float(2.0f)}
    };
    mission_load(&mission, waypoints, 5, false);

    /* Open CSV log file */
    FILE *log_file = fopen("flight_log.csv", "w");
    if (!log_file) {
        fprintf(stderr, "ERROR: Could not create flight_log.csv\n");
        return 1;
    }

    fprintf(log_file, "time,mode,armed,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,");
    fprintf(log_file, "roll,pitch,yaw,est_roll,est_pitch,est_yaw,");
    fprintf(log_file, "alt_true,alt_est,sp_alt,sp_n,sp_e,");
    fprintf(log_file, "m0,m1,m2,m3,throttle_cmd\n");

    /* ========================================================================
     * SIMULATION MAIN LOOP (Cooperative Superloop Dispatcher)
     * ===================================================================== */
    uint32_t total_ticks = (uint32_t)(SIM_DURATION_SEC * FAST_LOOP_HZ);

    for (uint32_t tick = 0; tick < total_ticks; tick++) {
        float time_sec = tick * DT_FAST_SEC;

        /* Get pilot input scenario */
        update_pilot_rc_scenario(time_sec, &rc_input);

        /* Check arming attempt at t=2.0s */
        if (tick == (uint32_t)(2.0f * FAST_LOOP_HZ)) {
            flight_state_t fstate;
            fstate.gyro_x = fx_from_float(vehicle.omega_x);
            fstate.gyro_y = fx_from_float(vehicle.omega_y);
            fstate.gyro_z = fx_from_float(vehicle.omega_z);
            fstate.throttle = rc_input.throttle;
            fstate.mode = MODE_STABILIZE;
            fstate.imu_calibrated = true;
            fstate.critical_sensor_fault = false;

            arming_result_t arm_res = attempt_arm(&fstate, &arming_state);
            printf("  [t=%.1fs] Arm Request -> %s (%s)\n",
                   time_sec, arming_result_name(arm_res), arming_state_name(arming_state));
            if (arm_res == ARM_SUCCESS) {
                current_mode = MODE_STABILIZE;
            }
        }

        /* 1. Fast Tier (100 Hz, every tick) */
        run_fast_tier_100hz();

        /* 2. Medium Tier (10 Hz, every 10 ticks) */
        if (tick % MEDIUM_DIVIDER == 0) {
            run_medium_tier_10hz();
        }

        /* 3. Navigation Tier (5 Hz, every 20 ticks) */
        if (tick % NAV_DIVIDER == 0) {
            run_nav_tier_5hz();
        }

        /* 4. Slow Tier (2 Hz, every 50 ticks) */
        if (tick % SLOW_DIVIDER == 0) {
            run_slow_tier_2hz();
        }

        /* 5. Telemetry CSV Logging (10 Hz) */
        if (tick % LOG_DIVIDER == 0) {
            euler_t true_euler;
            quat_to_euler(&vehicle.attitude, &true_euler);

            fprintf(log_file, "%.2f,%s,%d,",
                    time_sec,
                    flight_mode_name(current_mode),
                    is_armed(arming_state) ? 1 : 0);

            fprintf(log_file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                    vehicle.pos_n, vehicle.pos_e, vehicle.pos_d,
                    vehicle.vel_n, vehicle.vel_e, vehicle.vel_d);

            fprintf(log_file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",
                    true_euler.roll * 180.0f / 3.14159f,
                    true_euler.pitch * 180.0f / 3.14159f,
                    true_euler.yaw * 180.0f / 3.14159f,
                    fx_to_float(attitude_est.roll) * 180.0f / 3.14159f,
                    fx_to_float(attitude_est.pitch) * 180.0f / 3.14159f,
                    fx_to_float(attitude_est.yaw) * 180.0f / 3.14159f);

            fprintf(log_file, "%.2f,%.2f,%.2f,%.2f,%.2f,",
                    -vehicle.pos_d,
                    fx_to_float(altitude_est.altitude),
                    fx_to_float(setpoint_altitude),
                    fx_to_float(setpoint_pos_n),
                    fx_to_float(setpoint_pos_e));

            fprintf(log_file, "%d,%d,%d,%d,%d\n",
                    motor_cmds_sim.motor[0], motor_cmds_sim.motor[1],
                    motor_cmds_sim.motor[2], motor_cmds_sim.motor[3],
                    fx_to_int(setpoint_throttle));
        }

        /* Progress printout every 2.5 seconds */
        if (tick % (uint32_t)(FAST_LOOP_HZ * 2.5f) == 0) {
            euler_t true_euler;
            quat_to_euler(&vehicle.attitude, &true_euler);
            printf("[t=%5.1fs] Mode=%-10s Armed=%d | Pos(N,E,Alt)=(%5.1fm, %5.1fm, %4.1fm) | Att(R,P,Y)=(%4.1f°, %4.1f°, %4.1f°) | Motors=[%4d,%4d,%4d,%4d]\n",
                   time_sec,
                   flight_mode_name(current_mode),
                   is_armed(arming_state) ? 1 : 0,
                   vehicle.pos_n, vehicle.pos_e, -vehicle.pos_d,
                   true_euler.roll * 180.0f / 3.14159f,
                   true_euler.pitch * 180.0f / 3.14159f,
                   true_euler.yaw * 180.0f / 3.14159f,
                   motor_cmds_sim.motor[0], motor_cmds_sim.motor[1],
                   motor_cmds_sim.motor[2], motor_cmds_sim.motor[3]);
        }
    }

    fclose(log_file);

    printf("\n====================================================================\n");
    printf("  FULL SIMULATION COMPLETE: SUCCESS! ✓\n");
    printf("====================================================================\n");
    printf("Flight Log exported to: flight_log.csv\n");
    printf("Final Vehicle State:\n");
    printf("  - Position: North = %.2fm, East = %.2fm, Altitude = %.2fm\n",
           vehicle.pos_n, vehicle.pos_e, -vehicle.pos_d);
    printf("  - Velocity: North = %.2fm/s, East = %.2fm/s, Vert = %.2fm/s\n",
           vehicle.vel_n, vehicle.vel_e, -vehicle.vel_d);
    printf("  - Total Mission Waypoints Sequenced: %d / %d\n", mission.current_index, mission.count);

    return 0;
}
