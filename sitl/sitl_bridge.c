/**
 * sitl_bridge.c
 *
 * Software-In-The-Loop harness --- flies the real firmware against a simulated
 * vehicle and talks to Mission Planner or QGroundControl over UDP.
 *
 * ---------------------------------------------------------------------------
 * WHAT IS AND IS NOT SIMULATED
 * ---------------------------------------------------------------------------
 *
 *      +-------------------------------------------------------------+
 *      |  REAL FIRMWARE --- identical code to the Aries V3 build      |
 *      |                                                             |
 *      |    core/    Q16.16 math, CORDIC, complementary filters,      |
 *      |             cascaded PID, mixer, mode machine, arming,       |
 *      |             mission sequencer                               |
 *      |    mavlink/ encoder, parser, ground-control bridge           |
 *      +-------------------------------------------------------------+
 *                 ^                              |
 *      sensors    |                              |  motor PWM
 *                 |                              v
 *      +-------------------------------------------------------------+
 *      |  SIMULATED --- exists only on the PC                         |
 *      |    core/vehicle_sim.c   6-DOF rigid-body physics             |
 *      |    sitl_link.c          UDP standing in for the SiK radio    |
 *      |    this file's RC block a transmitter that is not here       |
 *      +-------------------------------------------------------------+
 *
 * Nothing in core/ or mavlink/ is stubbed, mocked, or recompiled differently.
 * The fixed-point arithmetic is bit-exact with what the RV32IM core will
 * execute, so a control response seen here is the control response the board
 * will produce given the same sensor input.
 *
 * ---------------------------------------------------------------------------
 * USING IT
 * ---------------------------------------------------------------------------
 *   1. Run this program.
 *   2. Mission Planner: connection type UDP, port 14550, press Connect.
 *      QGroundControl connects automatically.
 *   3. Draw a mission on the map and press Write/Upload.
 *   4. Actions -> Arm, then Takeoff, then set mode AUTO to fly it.
 *
 * Ctrl+C stops the simulation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>

/* Flight core --- the code that will run on the board */
#include "fixed_point.h"
#include "rotation.h"
#include "sensor_fusion.h"
#include "control.h"
#include "flight_modes.h"
#include "arming.h"
#include "mission.h"

/* MAVLink stack --- also runs on the board */
#include "mavlink_bridge.h"

/* Simulation-only */
#include "vehicle_sim.h"
#include "sitl_link.h"

/* ============================================================================
 * Configuration
 * ========================================================================= */

#define FAST_LOOP_HZ        100
#define TICK_PERIOD_MS      10          /* 100 Hz */
#define MEDIUM_DIVIDER      10          /* 10 Hz */
#define NAV_DIVIDER         20          /*  5 Hz */
#define SLOW_DIVIDER        50          /*  2 Hz */
#define HEARTBEAT_DIVIDER   100         /*  1 Hz */
#define STATUS_DIVIDER      100         /*  1 Hz console line */

#define DT_FAST_SEC         0.01f

/* Where the vehicle sits on the ground. Anywhere works; this is IISc,
 * Bangalore, so the map opens somewhere recognisable. */
#define HOME_LAT            12.9716
#define HOME_LON            77.5946
#define HOME_ALT_MSL        920.0f

#define HOVER_THROTTLE_PWM  1500

/* ============================================================================
 * Flight controller state --- the same set integration_test.c declares
 * ========================================================================= */

static vehicle_state_t   vehicle;

static attitude_t        attitude_est;
static altitude_state_t  altitude_est;
static nav_state_t       nav_horizontal_est;

static arming_state_t    arming_state;
static flight_mode_t     current_mode = MODE_DISARMED;
static nav_health_t      nav_health;

static mission_t         mission;

static pid_t pid_roll_angle, pid_pitch_angle;
static pid_t pid_roll_rate,  pid_pitch_rate, pid_yaw_rate;
static pid_t pid_altitude,   pid_vel_z;
static pid_t pid_pos_n, pid_pos_e, pid_vel_n, pid_vel_e;

static fx_t setpoint_roll_angle  = 0;
static fx_t setpoint_pitch_angle = 0;
static fx_t setpoint_yaw_rate    = 0;
static fx_t setpoint_throttle    = (1000 << FX_SHIFT);
static fx_t setpoint_altitude    = 0;
static fx_t setpoint_pos_n       = 0;
static fx_t setpoint_pos_e       = 0;

static fx_t              motor_cmds_fx[NUM_MOTORS];
static motor_commands_t  motor_cmds_sim;
static rc_input_t        rc_input;

/* Most recent accelerometer/gyro sample, published by the 100 Hz tier for the
 * 10 Hz altitude filter. Section 6.5 fuses barometric altitude against the
 * measured specific force, so the filter needs the real accelerometer reading
 * --- not an assumed 1 g, which would make the estimate diverge whenever the
 * vehicle is tilted or accelerating. */
static imu_sample_t      last_imu_sample;

/* ============================================================================
 * Ground-control state
 * ========================================================================= */

static mav_bridge_t  bridge;
static sitl_link_t  *link;

/* The Section 10.3 sequencer reports waypoint arrival by advancing
 * current_index; this mirrors it so MISSION_ITEM_REACHED fires exactly once
 * per waypoint rather than every navigation tick. */
static uint8_t last_reported_wp = 0xFF;

/* Set once the vehicle has a usable fix, so the GCS can convert waypoints. */
static bool home_latched = false;

static volatile sig_atomic_t keep_running = 1;

static void on_sigint(int sig) { (void)sig; keep_running = 0; }

/* ============================================================================
 * Controller initialisation --- gains identical to integration_test.c
 * ========================================================================= */

static void init_flight_controllers(void) {
    /* Attitude angle loop: angle error -> target angular rate (rad/s) */
    pid_init(&pid_roll_angle,  fx_from_float(4.0f), fx_from_float(0.05f), fx_from_float(0.02f),
             fx_from_float(0.5f), fx_from_float(-2.5f), fx_from_float(2.5f));
    pid_init(&pid_pitch_angle, fx_from_float(4.0f), fx_from_float(0.05f), fx_from_float(0.02f),
             fx_from_float(0.5f), fx_from_float(-2.5f), fx_from_float(2.5f));

    /* Attitude rate loop: rate error -> moment (PWM delta) */
    pid_init(&pid_roll_rate,  fx_from_float(60.0f), fx_from_float(15.0f), fx_from_float(0.5f),
             fx_from_float(40.0f), fx_from_float(-250.0f), fx_from_float(250.0f));
    pid_init(&pid_pitch_rate, fx_from_float(60.0f), fx_from_float(15.0f), fx_from_float(0.5f),
             fx_from_float(40.0f), fx_from_float(-250.0f), fx_from_float(250.0f));
    pid_init(&pid_yaw_rate,   fx_from_float(50.0f), fx_from_float(10.0f), fx_from_float(0.2f),
             fx_from_float(30.0f), fx_from_float(-150.0f), fx_from_float(150.0f));

    /* Altitude: alt -> climb rate -> throttle delta */
    pid_init(&pid_altitude, fx_from_float(1.0f), fx_from_float(0.05f), fx_from_float(0.02f),
             fx_from_float(1.5f), fx_from_float(-2.0f), fx_from_float(2.0f));
    pid_init(&pid_vel_z,    fx_from_float(70.0f), fx_from_float(20.0f), fx_from_float(2.0f),
             fx_from_float(60.0f), fx_from_float(-250.0f), fx_from_float(250.0f));

    /* Position: pos -> velocity -> commanded world acceleration */
    pid_init(&pid_pos_n, fx_from_float(0.6f), fx_from_float(0.01f), fx_from_float(0.02f),
             fx_from_float(1.0f), fx_from_float(-2.5f), fx_from_float(2.5f));
    pid_init(&pid_pos_e, fx_from_float(0.6f), fx_from_float(0.01f), fx_from_float(0.02f),
             fx_from_float(1.0f), fx_from_float(-2.5f), fx_from_float(2.5f));
    pid_init(&pid_vel_n, fx_from_float(0.8f), fx_from_float(0.10f), fx_from_float(0.02f),
             fx_from_float(0.8f), fx_from_float(-2.0f), fx_from_float(2.0f));
    pid_init(&pid_vel_e, fx_from_float(0.8f), fx_from_float(0.10f), fx_from_float(0.02f),
             fx_from_float(0.8f), fx_from_float(-2.0f), fx_from_float(2.0f));
}

/* ============================================================================
 * GCS-tunable parameters
 *
 * Exposing the live gain storage means Mission Planner's parameter tree can
 * retune the controller while the simulation is flying --- the same mechanism
 * that will work over the telemetry radio on the bench.
 * ========================================================================= */

static const mav_param_entry_t param_table[] = {
    { "ANG_RLL_P",  &pid_roll_angle.Kp  },
    { "ANG_PIT_P",  &pid_pitch_angle.Kp },
    { "RATE_RLL_P", &pid_roll_rate.Kp   },
    { "RATE_RLL_I", &pid_roll_rate.Ki   },
    { "RATE_RLL_D", &pid_roll_rate.Kd   },
    { "RATE_PIT_P", &pid_pitch_rate.Kp  },
    { "RATE_PIT_I", &pid_pitch_rate.Ki  },
    { "RATE_PIT_D", &pid_pitch_rate.Kd  },
    { "RATE_YAW_P", &pid_yaw_rate.Kp    },
    { "ALT_HOLD_P", &pid_altitude.Kp    },
    { "VELZ_P",     &pid_vel_z.Kp       },
    { "VELZ_I",     &pid_vel_z.Ki       },
    { "POS_XY_P",   &pid_pos_n.Kp       },
    { "VEL_XY_P",   &pid_vel_n.Kp       }
};
#define PARAM_COUNT ((uint16_t)(sizeof(param_table) / sizeof(param_table[0])))

/* ============================================================================
 * Simulated RC transmitter
 *
 * There is no radio on the bench, so this fabricates the stick and switch
 * inputs the Section 9 mode machine expects. Deliberately minimal: sticks
 * stay centred, so the Override Supremacy Rule never fires and the ground
 * station retains control. The mode switch is driven by whatever mode the GCS
 * last asked for, which routes the GCS request through the real state machine
 * and its entry interlocks rather than around them.
 * ========================================================================= */

static rc_switch_position_t gcs_switch_request = SWITCH_STABILIZE;

static void update_simulated_rc(rc_input_t *rc, uint32_t now_ms) {
    rc->roll  = 0;
    rc->pitch = 0;
    rc->yaw   = 0;

    /* Throttle must sit at minimum for the Section 11.1 arming check to pass,
     * and at mid-stick afterwards so ALT_HOLD reads "hold current altitude". */
    rc->throttle = is_armed(arming_state) ? fx_from_float(0.5f) : 0;

    rc->arm_switch                = is_armed(arming_state);
    rc->mode_switch_position      = gcs_switch_request;
    rc->last_valid_frame_time_ms  = now_ms;
}

static rc_switch_position_t mode_to_switch(flight_mode_t m) {
    switch (m) {
    case MODE_ALT_HOLD: return SWITCH_ALT_HOLD;
    case MODE_POS_HOLD: return SWITCH_POS_HOLD;
    case MODE_MISSION:  return SWITCH_AUTO;
    default:            return SWITCH_STABILIZE;
    }
}

/* ============================================================================
 * Superloop tiers --- structurally identical to integration_test.c, but driven
 * by the ground station instead of a scripted flight programme
 * ========================================================================= */

static void run_nav_tier_5hz(void) {
    gps_reading_t gps;
    gps_fix_ned_t fix_ned;
    fx_t accel_world_n = 0, accel_world_e = 0, accel_world_d = 0;

    vehicle_sim_get_gps(&vehicle, &gps);

    fix_ned.pos_n      = fx_from_float(vehicle.pos_n);
    fix_ned.pos_e      = fx_from_float(vehicle.pos_e);
    fix_ned.vel_n      = fx_from_float(gps.vel_n);
    fix_ned.vel_e      = fx_from_float(gps.vel_e);
    fix_ned.quality_ok = (gps.fix_type >= 3 && gps.num_sats >= 6);

    rotate_accel_to_world(0, 0, 0,
                          attitude_est.roll, attitude_est.pitch, attitude_est.yaw,
                          &accel_world_n, &accel_world_e, &accel_world_d);

    update_horizontal_nav(accel_world_n, accel_world_e, DT_NAV_Q16,
                          &fix_ned, &nav_horizontal_est);

    /* Section 5.7 / 2.5: latch a provisional NED origin from the first fix
     * that passes the quality gate.
     *
     * The architecture latches home at arming, but a ground station uploads
     * its mission before arming and the waypoints have nothing to convert
     * against until an origin exists --- so one is established here and the
     * real latch happens at arming (see service_gcs_events). That re-latch is
     * safe because the bridge keeps the coordinates the ground station sent
     * and re-projects the mission when the origin moves. */
    if (!home_latched && fix_ned.quality_ok) {
        mavlink_bridge_set_home(&bridge,
                                (int32_t)(gps.latitude  * 1e7),
                                (int32_t)(gps.longitude * 1e7),
                                (int32_t)(gps.altitude_msl * 1000.0f));
        home_latched = true;
        printf("[home] latched at %.7f, %.7f  (%u sats, fix type %u)\n",
               gps.latitude, gps.longitude, gps.num_sats, gps.fix_type);
        mavlink_bridge_send_statustext(&bridge, MAV_SEVERITY_INFO, "Home position set");
    }

    /* Mission sequencer (Section 10.3) */
    if (current_mode == MODE_MISSION || current_mode == MODE_RTL) {
        nav_state_3d_t      nav_3d;
        position_setpoint_t mission_sp;
        flight_mode_t       requested_mode = current_mode;

        nav_3d.north = nav_horizontal_est.pos_n;
        nav_3d.east  = nav_horizontal_est.pos_e;
        nav_3d.down  = fx_neg(altitude_est.altitude);

        navigation_step(&nav_3d, current_mode, &mission, &mission_sp, &requested_mode);

        setpoint_pos_n    = mission_sp.north;
        setpoint_pos_e    = mission_sp.east;
        setpoint_altitude = fx_neg(mission_sp.down);

        /* Tell the ground station each time a waypoint is accepted. */
        if (mission.current_index != last_reported_wp) {
            if (last_reported_wp != 0xFF && mission.current_index > 0) {
                mavlink_bridge_notify_waypoint_reached(&bridge,
                                                       (uint16_t)(mission.current_index - 1));
                printf("[mission] waypoint %u reached\n", mission.current_index - 1);
            }
            last_reported_wp = mission.current_index;
        }

        if (requested_mode == MODE_RTL && current_mode == MODE_MISSION) {
            current_mode       = MODE_RTL;
            gcs_switch_request = SWITCH_POS_HOLD;   /* keep the machine consistent */
            printf("[mission] complete -> RTL\n");
            mavlink_bridge_send_statustext(&bridge, MAV_SEVERITY_INFO,
                                           "Mission complete: RTL");
        }
    }

    /* Horizontal position and velocity outer loops (Section 7.6) */
    if (current_mode == MODE_POS_HOLD || current_mode == MODE_MISSION ||
        current_mode == MODE_RTL) {
        fx_t target_vel_n = pid_update(&pid_pos_n, setpoint_pos_n, nav_horizontal_est.pos_n, DT_NAV_Q16);
        fx_t target_vel_e = pid_update(&pid_pos_e, setpoint_pos_e, nav_horizontal_est.pos_e, DT_NAV_Q16);

        fx_t cmd_accel_n = pid_update(&pid_vel_n, target_vel_n, nav_horizontal_est.vel_n, DT_NAV_Q16);
        fx_t cmd_accel_e = pid_update(&pid_vel_e, target_vel_e, nav_horizontal_est.vel_e, DT_NAV_Q16);

        fx_t sin_yaw, cos_yaw;
        fx_t g_fx = fx_from_float(9.81f);
        fx_t max_tilt = fx_from_float(0.35f);
        fx_t roll_num, pitch_num;

        cordic_sincos(attitude_est.yaw, &sin_yaw, &cos_yaw);

        roll_num  = fx_sub(fx_mul(cmd_accel_e, cos_yaw), fx_mul(cmd_accel_n, sin_yaw));
        setpoint_roll_angle = fx_div(roll_num, g_fx);

        pitch_num = fx_add(fx_mul(cmd_accel_n, cos_yaw), fx_mul(cmd_accel_e, sin_yaw));
        setpoint_pitch_angle = fx_neg(fx_div(pitch_num, g_fx));

        setpoint_roll_angle  = fx_clamp(setpoint_roll_angle,  fx_neg(max_tilt), max_tilt);
        setpoint_pitch_angle = fx_clamp(setpoint_pitch_angle, fx_neg(max_tilt), max_tilt);
    }
}

static void run_medium_tier_10hz(void) {
    baro_reading_t baro;
    mag_sample_t   mag;
    euler_t        true_euler;
    fx_t           baro_alt, accel_world_z, mag_heading;

    vehicle_sim_get_baro(&vehicle, &baro);
    baro_alt = fx_from_float(baro.altitude_msl - HOME_ALT_MSL);

    /* Rotate the measured specific force into the world Down axis and remove
     * gravity, leaving the vehicle's own vertical acceleration. Feeding a
     * constant 1 g here instead of the live sample (as the short scripted
     * integration test does) makes this term collapse to -g*(1 - cos(tilt)),
     * which the filter then integrates without bound the moment the vehicle
     * tilts far --- the altitude estimate runs away and never recovers. */
    accel_world_z = extract_vertical_accel(last_imu_sample.ax,
                                           last_imu_sample.ay,
                                           last_imu_sample.az,
                                           attitude_est.roll, attitude_est.pitch);
    accel_world_z = fx_sub(accel_world_z, fx_from_float(9.81f));

    update_altitude_filter(accel_world_z, baro_alt, DT_MEDIUM_Q16, &altitude_est);

    quat_to_euler(&vehicle.attitude, &true_euler);
    mag.mx = fx_from_float(cosf(true_euler.yaw));
    mag.my = fx_from_float(-sinf(true_euler.yaw));
    mag.mz = 0;
    mag_heading = compute_mag_heading(&mag, &attitude_est);
    correct_yaw_with_mag(mag_heading, &attitude_est);

    /* Mode state machine (Section 9), with the GCS request entering through
     * the same switch input a pilot would use. */
    if (!is_armed(arming_state)) {
        current_mode = MODE_DISARMED;
    } else if (current_mode != MODE_RTL) {
        flight_mode_t next_mode = evaluate_mode_transition(current_mode, &rc_input,
                                                           &nav_health,
                                                           mission_is_loaded(&mission));
        if (next_mode != current_mode) {
            printf("[mode] %s -> %s\n", flight_mode_name(current_mode),
                                        flight_mode_name(next_mode));
            current_mode = next_mode;
        }
    }

    /* Altitude control loop (Section 7.5) */
    if (current_mode == MODE_ALT_HOLD || current_mode == MODE_POS_HOLD ||
        current_mode == MODE_MISSION  || current_mode == MODE_RTL) {
        fx_t target_vel_z   = pid_update(&pid_altitude, setpoint_altitude,
                                         altitude_est.altitude, DT_MEDIUM_Q16);
        fx_t throttle_delta = pid_update(&pid_vel_z, target_vel_z,
                                         altitude_est.velocity_z, DT_MEDIUM_Q16);

        setpoint_throttle = fx_add(fx_from_int(HOVER_THROTTLE_PWM), throttle_delta);
        setpoint_throttle = fx_clamp(setpoint_throttle, fx_from_int(1100), fx_from_int(1850));
    } else if (current_mode == MODE_STABILIZE) {
        setpoint_throttle = fx_from_int(1000 + (int32_t)(fx_to_float(rc_input.throttle) * 1000.0f));
    }
}

static void run_slow_tier_2hz(void) {
    fx_t max_safe_angle = fx_from_float(1.05f);   /* ~60 degrees */

    nav_health.baro_ok      = true;
    nav_health.gps_fix_ok   = true;
    nav_health.home_settled = home_latched;
    nav_health.imu_ok       = true;
    nav_health.mag_ok       = true;

    /* Only meaningful while armed. A disarmed vehicle lying on its side after
     * a heavy landing is legitimately past the limit, and re-disarming it
     * every 500 ms would flood the telemetry link with STATUSTEXT for as long
     * as it sat there. */
    if (is_armed(arming_state) &&
        (fx_abs(attitude_est.roll)  > max_safe_angle ||
         fx_abs(attitude_est.pitch) > max_safe_angle)) {
        printf("[failsafe] attitude limit exceeded --- emergency disarm\n");
        mavlink_bridge_send_statustext(&bridge, MAV_SEVERITY_EMERGENCY,
                                       "CRASH DETECTED: DISARMED");
        disarm(&arming_state);
        current_mode       = MODE_DISARMED;
        gcs_switch_request = SWITCH_STABILIZE;
    }
}

/**
 * Landing-gear model.
 *
 * vehicle_sim.c stops the vehicle sinking below home altitude, but nothing
 * stops it lying on its side spinning: with the motors off there is no torque
 * to right it, and angular drag alone takes a very long time to bleed off. A
 * real quadcopter resting on its legs settles level within a second or so.
 *
 * Without this, one hard landing ends the session --- the vehicle can never
 * satisfy the Section 11.1 "gyro stationary" arming check again, so every
 * subsequent arm request is refused and the harness is stuck until restarted.
 * That is a property of the missing simulation, not of the firmware, so the
 * fix belongs here rather than in the arming logic.
 */
static void apply_ground_contact(void) {
    const float GROUND_EPS = 0.05f;    /* within 5 cm of the pad */
    euler_t e;
    quaternion_t level;

    if (vehicle.pos_d < -GROUND_EPS) return;      /* airborne */
    if (is_armed(arming_state)) return;           /* may be about to take off */

    /* Resting on the pad: bleed off residual motion and settle upright,
     * keeping the heading the vehicle came to rest on. */
    vehicle.vel_n *= 0.80f;
    vehicle.vel_e *= 0.80f;
    vehicle.vel_d  = 0.0f;
    vehicle.pos_d  = 0.0f;

    vehicle.omega_x *= 0.80f;
    vehicle.omega_y *= 0.80f;
    vehicle.omega_z *= 0.80f;

    quat_to_euler(&vehicle.attitude, &e);
    e.roll  *= 0.80f;
    e.pitch *= 0.80f;
    euler_to_quat(&e, &level);
    vehicle.attitude = level;
}

static void run_fast_tier_100hz(void) {
    imu_reading_t imu_raw;
    imu_sample_t  imu_sample;
    int i;

    vehicle_sim_get_imu(&vehicle, &imu_raw);

    imu_sample.ax = fx_from_float(imu_raw.accel_x);
    imu_sample.ay = fx_from_float(imu_raw.accel_y);
    imu_sample.az = fx_from_float(imu_raw.accel_z);
    imu_sample.gx = fx_from_float(imu_raw.gyro_x);
    imu_sample.gy = fx_from_float(imu_raw.gyro_y);
    imu_sample.gz = fx_from_float(imu_raw.gyro_z);

    update_attitude_complementary(&imu_sample, &attitude_est);
    integrate_yaw(imu_sample.gz, &attitude_est);

    if (current_mode == MODE_STABILIZE || current_mode == MODE_ALT_HOLD) {
        setpoint_roll_angle  = fx_mul(rc_input.roll,  fx_from_float(0.35f));
        setpoint_pitch_angle = fx_mul(rc_input.pitch, fx_from_float(0.35f));
    }
    setpoint_yaw_rate = fx_mul(rc_input.yaw, fx_from_float(1.5f));

    if (is_armed(arming_state) && current_mode != MODE_DISARMED) {
        fx_t target_rate_roll  = pid_update(&pid_roll_angle,  setpoint_roll_angle,
                                            attitude_est.roll,  DT_FAST_Q16);
        fx_t target_rate_pitch = pid_update(&pid_pitch_angle, setpoint_pitch_angle,
                                            attitude_est.pitch, DT_FAST_Q16);

        fx_t roll_moment  = pid_update(&pid_roll_rate,  target_rate_roll,  imu_sample.gx, DT_FAST_Q16);
        fx_t pitch_moment = pid_update(&pid_pitch_rate, target_rate_pitch, imu_sample.gy, DT_FAST_Q16);
        fx_t yaw_moment   = pid_update(&pid_yaw_rate,   setpoint_yaw_rate, imu_sample.gz, DT_FAST_Q16);

        mix_and_output_motors(setpoint_throttle, roll_moment, pitch_moment,
                              yaw_moment, motor_cmds_fx);

        for (i = 0; i < NUM_MOTORS; i++) {
            motor_cmds_sim.motor[i] = motor_to_pwm(motor_cmds_fx[i]);
        }
    } else {
        for (i = 0; i < NUM_MOTORS; i++) {
            motor_cmds_fx[i]        = fx_from_int(1000);
            motor_cmds_sim.motor[i] = 1000;
        }
    }

    last_imu_sample = imu_sample;

    vehicle_sim_step(&vehicle, &motor_cmds_sim, DT_FAST_SEC);
    apply_ground_contact();
}

/* ============================================================================
 * Ground-station plumbing
 * ========================================================================= */

/** Fill the snapshot the MAVLink bridge reads. Pure state copying. */
static void build_snapshot(mav_flight_snapshot_t *s, uint32_t now_ms) {
    gps_reading_t gps;
    int32_t throttle_pwm;

    vehicle_sim_get_gps(&vehicle, &gps);
    memset(s, 0, sizeof(*s));

    s->time_boot_ms = now_ms;

    s->roll       = attitude_est.roll;
    s->pitch      = attitude_est.pitch;
    s->yaw        = attitude_est.yaw;
    s->rollspeed  = fx_from_float(vehicle.omega_x);
    s->pitchspeed = fx_from_float(vehicle.omega_y);
    s->yawspeed   = fx_from_float(vehicle.omega_z);

    s->altitude_rel = altitude_est.altitude;
    s->climb_rate   = altitude_est.velocity_z;

    s->pos_n = nav_horizontal_est.pos_n;
    s->pos_e = nav_horizontal_est.pos_e;
    s->vel_n = nav_horizontal_est.vel_n;
    s->vel_e = nav_horizontal_est.vel_e;
    s->vel_d = fx_from_float(vehicle.vel_d);

    s->gps_lat_e7     = (int32_t)(gps.latitude  * 1e7);
    s->gps_lon_e7     = (int32_t)(gps.longitude * 1e7);
    s->gps_alt_mm_msl = (int32_t)(gps.altitude_msl * 1000.0f);
    s->gps_fix_type   = gps.fix_type;
    s->gps_num_sats   = gps.num_sats;
    s->gps_eph        = 120;                 /* HDOP 1.2, a plausible open-sky value */

    s->mode                = current_mode;
    s->armed               = is_armed(arming_state);
    s->sys_state           = is_armed(arming_state) ? (uint8_t)MAV_STATE_ACTIVE
                                                    : (uint8_t)MAV_STATE_STANDBY;
    s->mission_current_seq = mission.current_index;

    /* No battery monitor in the simulation; report a plausible, steady pack so
     * the ground station's voltage display is populated rather than blank. */
    s->battery_mv  = 11400;
    s->battery_pct = 85;
    s->cpu_load    = 250;                    /* 25 % --- a stand-in for Section 12 */

    throttle_pwm     = fx_to_int(setpoint_throttle);
    if (throttle_pwm < 1000) throttle_pwm = 1000;
    if (throttle_pwm > 2000) throttle_pwm = 2000;
    s->throttle_pct  = (uint16_t)((throttle_pwm - 1000) / 10);

    s->sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO
                       | MAV_SYS_STATUS_SENSOR_3D_ACCEL
                       | MAV_SYS_STATUS_SENSOR_3D_MAG
                       | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
                       | MAV_SYS_STATUS_SENSOR_GPS
                       | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL
                       | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION
                       | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL
                       | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL
                       | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    s->sensors_enabled = s->sensors_present;
    s->sensors_health  = s->sensors_present;
}

/**
 * Act on whatever the ground station asked for.
 *
 * Every branch here answers the bridge with the real outcome, so a refused
 * arm shows the actual Section 11.1 denial reason in the ground station's
 * message pane rather than a generic failure.
 */
static void service_gcs_events(void) {
    mav_events_t ev;
    mavlink_bridge_poll_events(&bridge, &ev);

    if (ev.arm_request) {
        if (ev.arm_value) {
            flight_state_t fstate;
            arming_result_t res;

            fstate.gyro_x   = fx_from_float(vehicle.omega_x);
            fstate.gyro_y   = fx_from_float(vehicle.omega_y);
            fstate.gyro_z   = fx_from_float(vehicle.omega_z);
            fstate.throttle = rc_input.throttle;
            fstate.mode     = MODE_STABILIZE;
            fstate.imu_calibrated       = true;
            fstate.critical_sensor_fault= false;

            res = attempt_arm(&fstate, &arming_state);
            printf("[gcs] arm request -> %s\n", arming_result_name(res));

            if (res == ARM_SUCCESS) {
                gps_reading_t fix;

                /* Section 2.5: the local NED origin is latched at arming. It
                 * was latched provisionally at the first fix so the ground
                 * station had something to convert waypoints against; moving
                 * it here re-projects any loaded mission from the coordinates
                 * that were uploaded, so the waypoints stay on the ground
                 * where they were drawn while the origin follows the vehicle
                 * to wherever it actually took off from. */
                vehicle_sim_get_gps(&vehicle, &fix);
                if (fix.fix_type >= 3 && fix.num_sats >= 6) {
                    mavlink_bridge_set_home(&bridge,
                                            (int32_t)(fix.latitude  * 1e7),
                                            (int32_t)(fix.longitude * 1e7),
                                            (int32_t)(fix.altitude_msl * 1000.0f));
                    printf("[home] re-latched at arming: %.7f, %.7f\n",
                           fix.latitude, fix.longitude);
                }

                current_mode       = MODE_STABILIZE;
                gcs_switch_request = SWITCH_STABILIZE;
                last_reported_wp   = 0xFF;
                mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, "Armed");
            } else {
                mavlink_bridge_command_result(&bridge, MAV_RESULT_DENIED,
                                              arming_result_name(res));
            }
        } else {
            disarm(&arming_state);
            current_mode       = MODE_DISARMED;
            gcs_switch_request = SWITCH_STABILIZE;
            printf("[gcs] disarm\n");
            mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, "Disarmed");
        }
    }

    if (ev.mode_request) {
        /* The request is routed through the mode switch, so the Section 9.3
         * entry interlocks decide whether it is honoured. Refusing here would
         * duplicate that logic; letting the state machine refuse keeps one
         * authority over mode changes. */
        if (ev.mode_value == MODE_RTL) {
            if (is_armed(arming_state)) {
                current_mode       = MODE_RTL;
                setpoint_pos_n     = 0;
                setpoint_pos_e     = 0;
                gcs_switch_request = SWITCH_POS_HOLD;
                printf("[gcs] mode -> RTL\n");
                mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, "RTL");
            } else {
                mavlink_bridge_command_result(&bridge, MAV_RESULT_DENIED, "RTL needs arming");
            }
        } else {
            gcs_switch_request = mode_to_switch(ev.mode_value);
            printf("[gcs] mode -> %s (pending interlocks)\n", flight_mode_name(ev.mode_value));
            mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, NULL);
        }
    }

    if (ev.rtl_request) {
        if (is_armed(arming_state)) {
            current_mode       = MODE_RTL;
            setpoint_pos_n     = 0;
            setpoint_pos_e     = 0;
            gcs_switch_request = SWITCH_POS_HOLD;
            printf("[gcs] return to launch\n");
            mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, "Returning to launch");
        } else {
            mavlink_bridge_command_result(&bridge, MAV_RESULT_DENIED, "Not armed");
        }
    }

    if (ev.takeoff_request) {
        if (!is_armed(arming_state)) {
            mavlink_bridge_command_result(&bridge, MAV_RESULT_DENIED, "Not armed");
        } else {
            fx_t alt = ev.takeoff_alt_m;
            if (alt < fx_from_float(1.0f)) alt = fx_from_float(10.0f);  /* GCS default */
            setpoint_altitude  = alt;
            gcs_switch_request = SWITCH_ALT_HOLD;
            printf("[gcs] takeoff to %.1f m\n", (double)fx_to_float(alt));
            mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, "Taking off");
        }
    }

    if (ev.mission_start_request) {
        if (!mission_is_loaded(&mission)) {
            mavlink_bridge_command_result(&bridge, MAV_RESULT_DENIED, "No mission loaded");
        } else if (!is_armed(arming_state)) {
            mavlink_bridge_command_result(&bridge, MAV_RESULT_DENIED, "Not armed");
        } else {
            gcs_switch_request     = SWITCH_AUTO;
            mission.current_index  = 0;
            last_reported_wp       = 0xFF;
            printf("[gcs] mission start (%u waypoints)\n", mission.count);
            mavlink_bridge_command_result(&bridge, MAV_RESULT_ACCEPTED, "Mission started");
        }
    }

    if (ev.mission_uploaded) {
        printf("[gcs] mission uploaded: %u waypoints\n", mission.count);
        for (uint8_t i = 0; i < mission.count; i++) {
            printf("        WP %u   N %+8.1f m   E %+8.1f m   alt %6.1f m   r %.1f m\n",
                   i,
                   (double)fx_to_float(mission.waypoints[i].north),
                   (double)fx_to_float(mission.waypoints[i].east),
                   (double)-fx_to_float(mission.waypoints[i].down),
                   (double)fx_to_float(mission.waypoints[i].acceptance_radius));
        }
        last_reported_wp = 0xFF;
    }

    if (ev.mission_cleared) {
        printf("[gcs] mission cleared\n");
        last_reported_wp = 0xFF;
    }
}

/** Move bytes between the UDP socket and the MAVLink bridge. */
static void service_link(uint32_t now_ms) {
    uint8_t rx[2048];
    uint8_t tx[1024];
    int     n;
    uint16_t pending;

    /* Inbound: drain every waiting datagram, byte at a time, exactly as the
     * UART receive interrupt of Section 4.2 will do on the board. */
    while ((n = sitl_link_recv(link, rx, (int)sizeof(rx))) > 0) {
        for (int i = 0; i < n; i++) {
            mavlink_bridge_handle_byte(&bridge, rx[i], now_ms);
        }
    }

    /* Outbound: one datagram per drain. */
    pending = mavlink_bridge_tx_read(&bridge, tx, (uint16_t)sizeof(tx));
    if (pending > 0) {
        sitl_link_send(link, tx, (int)pending);
    }
}

/* ============================================================================
 * main
 * ========================================================================= */

int main(int argc, char **argv) {
    const char *gcs_host   = "127.0.0.1";
    uint16_t    gcs_port   = SITL_DEFAULT_GCS_PORT;
    uint16_t    local_port = SITL_DEFAULT_LOCAL_PORT;
    uint32_t    tick       = 0;
    uint32_t    next_tick_ms;
    mav_flight_snapshot_t snapshot;

    if (argc > 1) gcs_host   = argv[1];
    if (argc > 2) gcs_port   = (uint16_t)atoi(argv[2]);
    if (argc > 3) local_port = (uint16_t)atoi(argv[3]);

    printf("====================================================================\n");
    printf("  ARIES V3 FLIGHT CONTROLLER --- SOFTWARE IN THE LOOP\n");
    printf("====================================================================\n");
    printf("  Firmware  : core/ + mavlink/, Q16.16 fixed-point, unmodified\n");
    printf("  Simulated : 6-DOF vehicle physics, UDP in place of a SiK radio\n");
    printf("  Telemetry : -> %s:%u    (listening on %u)\n", gcs_host, gcs_port, local_port);
    printf("--------------------------------------------------------------------\n");
    printf("  Mission Planner : connection type UDP, port %u, Connect\n", gcs_port);
    printf("  QGroundControl  : connects automatically\n");
    printf("  Then: upload a mission, Arm, Takeoff, set mode AUTO.\n");
    printf("  Ctrl+C to stop.\n");
    printf("====================================================================\n\n");

    signal(SIGINT, on_sigint);

    link = sitl_link_open(gcs_host, gcs_port, local_port);
    if (!link) return 1;

    vehicle_sim_init(&vehicle, HOME_LAT, HOME_LON, HOME_ALT_MSL);

    memset(&attitude_est,       0, sizeof(attitude_est));
    memset(&altitude_est,       0, sizeof(altitude_est));
    memset(&nav_horizontal_est, 0, sizeof(nav_horizontal_est));
    memset(&nav_health,         0, sizeof(nav_health));

    init_flight_controllers();
    arming_state = ARMING_DISARMED;

    /* No mission is preloaded: the ground station uploads one. */
    mission_clear(&mission);

    mavlink_bridge_init(&bridge, &mission);
    mavlink_bridge_set_param_table(&bridge, param_table, PARAM_COUNT);

    next_tick_ms = sitl_millis();

    while (keep_running) {
        uint32_t now_ms = sitl_millis();

        /* --- Superloop dispatch, exactly the tier structure of Section 3.4 -- */
        update_simulated_rc(&rc_input, now_ms);

        run_fast_tier_100hz();                                   /* 100 Hz */
        if (tick % MEDIUM_DIVIDER == 0) run_medium_tier_10hz();  /*  10 Hz */
        if (tick % NAV_DIVIDER    == 0) run_nav_tier_5hz();      /*   5 Hz */
        if (tick % SLOW_DIVIDER   == 0) run_slow_tier_2hz();     /*   2 Hz */

        /* --- Ground-control link ---------------------------------------- */
        service_link(now_ms);
        service_gcs_events();
        mavlink_bridge_update(&bridge, now_ms);

        if (tick % HEARTBEAT_DIVIDER == 0) {
            build_snapshot(&snapshot, now_ms);
            mavlink_bridge_tick_1hz(&bridge, &snapshot);
        }
        if (tick % MEDIUM_DIVIDER == 0) {
            build_snapshot(&snapshot, now_ms);
            mavlink_bridge_tick_10hz(&bridge, &snapshot);
        }
        if (tick % NAV_DIVIDER == 0) {
            build_snapshot(&snapshot, now_ms);
            mavlink_bridge_tick_5hz(&bridge, &snapshot);
        }

        /* Flush again so replies generated this tick leave without a delay. */
        service_link(now_ms);

        /* --- Console status --------------------------------------------- */
        if (tick % STATUS_DIVIDER == 0) {
            const mav_link_stats_t *st = mavlink_bridge_stats(&bridge);
            printf("\r t=%5.1fs  %-9s %-8s  alt %6.2f m  N %+7.1f  E %+7.1f  "
                   "wp %u/%u  gcs:%s rx:%u crc:%u   ",
                   now_ms / 1000.0,
                   flight_mode_name(current_mode),
                   is_armed(arming_state) ? "ARMED" : "disarmed",
                   (double)fx_to_float(altitude_est.altitude),
                   (double)fx_to_float(nav_horizontal_est.pos_n),
                   (double)fx_to_float(nav_horizontal_est.pos_e),
                   mission.count ? mission.current_index : 0, mission.count,
                   sitl_link_peer_known(link) ? "up" : "--",
                   st->packets_received, st->crc_errors);
            fflush(stdout);
        }

        /* --- Pace to wall clock ------------------------------------------ */
        tick++;
        next_tick_ms += TICK_PERIOD_MS;
        {
            uint32_t t = sitl_millis();
            if ((int32_t)(next_tick_ms - t) > 0) {
                sitl_sleep_ms(next_tick_ms - t);
            } else {
                /* Fell behind (a debugger pause, a busy machine). Resynchronise
                 * rather than sprinting to catch up, which would make the
                 * physics step non-real-time and the telemetry rates wrong. */
                next_tick_ms = t;
            }
        }
    }

    printf("\n\nStopping.\n");
    printf("  MAVLink frames received : %u\n", mavlink_bridge_stats(&bridge)->packets_received);
    printf("  CRC errors              : %u\n", mavlink_bridge_stats(&bridge)->crc_errors);
    printf("  Frames dropped (TX full): %u\n", bridge.tx_dropped_frames);

    sitl_link_close(link);
    return 0;
}
