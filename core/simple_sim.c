#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* Include flight controller modules */
#include "fixed_point.h"
#include "rotation.h"
#include "sensor_fusion.h"
#include "flight_modes.h"
#include "arming.h"
#include "mission.h"
#include "vehicle_sim.h"

/**
 * SIMPLE_SIM.C
 *
 * Simplified integration test - demonstrates vehicle physics
 * with minimal flight controller integration.
 */

#define SIM_DURATION_SEC 20.0f
#define SIM_DT 0.01f  /* 100Hz simulation */
#define HOME_LAT 28.6139
#define HOME_LON 77.2090
#define HOME_ALT_MSL 216.0f

/* Simple RC control scenario */
void get_motor_commands(float time_sec, motor_commands_t *cmds) {
    /* Timeline:
     * 0-2s:   Motors off (1000µs)
     * 2-5s:   Gradual takeoff (1000 → 1550µs)
     * 5-15s:  Hover (1550µs)
     * 15-20s: Land (1550 → 1100µs)
     */

    uint16_t pwm = 1000;  /* Start with motors off */

    if (time_sec >= 2.0f && time_sec < 5.0f) {
        /* Takeoff ramp */
        float progress = (time_sec - 2.0f) / 3.0f;
        pwm = 1000 + (uint16_t)(550.0f * progress);
    } else if (time_sec >= 5.0f && time_sec < 15.0f) {
        /* Hover */
        pwm = 1550;
    } else if (time_sec >= 15.0f) {
        /* Landing */
        float progress = (time_sec - 15.0f) / 5.0f;
        if (progress > 1.0f) progress = 1.0f;
        pwm = 1550 - (uint16_t)(450.0f * progress);
    }

    /* Equal thrust on all motors (no attitude control yet) */
    for (int i = 0; i < 4; i++) {
        cmds->motor[i] = pwm;
    }
}

int main(void) {
    printf("==============================================================\n");
    printf(" VEHICLE PHYSICS SIMULATION TEST\n");
    printf("==============================================================\n\n");

    /* Initialize vehicle */
    vehicle_state_t vehicle;
    vehicle_sim_init(&vehicle, HOME_LAT, HOME_LON, HOME_ALT_MSL);

    /* Open CSV log */
    FILE *log = fopen("flight_log.csv", "w");
    if (!log) {
        fprintf(stderr, "ERROR: Could not create flight_log.csv\n");
        return 1;
    }

    /* CSV header */
    fprintf(log, "time,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,");
    fprintf(log, "roll,pitch,yaw,gyro_x,gyro_y,gyro_z,");
    fprintf(log, "accel_x,accel_y,accel_z,alt_baro,alt_fused,");
    fprintf(log, "m0,m1,m2,m3,throttle_rc,mode,armed\n");

    /* Simulation loop */
    uint32_t steps = (uint32_t)(SIM_DURATION_SEC / SIM_DT);

    for (uint32_t step = 0; step < steps; step++) {
        float time = step * SIM_DT;

        /* Get motor commands */
        motor_commands_t motors;
        get_motor_commands(time, &motors);

        /* Step physics */
        vehicle_sim_step(&vehicle, &motors, SIM_DT);

        /* Get sensor readings */
        imu_reading_t imu;
        baro_reading_t baro;
        vehicle_sim_get_imu(&vehicle, &imu);
        vehicle_sim_get_baro(&vehicle, &baro);

        /* Convert attitude to Euler angles */
        euler_t euler;
        quat_to_euler(&vehicle.attitude, &euler);

        /* Log every 0.1 seconds (10Hz) */
        if (step % 10 == 0) {
            fprintf(log, "%.2f,", time);
            fprintf(log, "%.3f,%.3f,%.3f,", vehicle.pos_n, vehicle.pos_e, vehicle.pos_d);
            fprintf(log, "%.3f,%.3f,%.3f,", vehicle.vel_n, vehicle.vel_e, vehicle.vel_d);
            fprintf(log, "%.3f,%.3f,%.3f,",
                    euler.roll * 180.0f / 3.14159f,
                    euler.pitch * 180.0f / 3.14159f,
                    euler.yaw * 180.0f / 3.14159f);
            fprintf(log, "%.3f,%.3f,%.3f,", imu.gyro_x, imu.gyro_y, imu.gyro_z);
            fprintf(log, "%.3f,%.3f,%.3f,", imu.accel_x, imu.accel_y, imu.accel_z);
            fprintf(log, "%.3f,%.3f,", baro.altitude_msl - HOME_ALT_MSL, baro.altitude_msl - HOME_ALT_MSL);
            fprintf(log, "%d,%d,%d,%d,", motors.motor[0], motors.motor[1],
                    motors.motor[2], motors.motor[3]);
            fprintf(log, "%d,STABILIZE,%d\n", motors.motor[0], (motors.motor[0] > 1050) ? 1 : 0);
        }

        /* Print progress every second */
        if (step % 100 == 0) {
            printf("[%.1fs] Alt=%.2fm  Vel=%.2fm/s  Motors=%dµs  Attitude=(%.1f°, %.1f°, %.1f°)\n",
                   time, -vehicle.pos_d,
                   sqrtf(vehicle.vel_n*vehicle.vel_n + vehicle.vel_e*vehicle.vel_e + vehicle.vel_d*vehicle.vel_d),
                   motors.motor[0],
                   euler.roll * 180.0f / 3.14159f,
                   euler.pitch * 180.0f / 3.14159f,
                   euler.yaw * 180.0f / 3.14159f);
        }
    }

    fclose(log);

    printf("\n==============================================================\n");
    printf(" SIMULATION COMPLETE\n");
    printf("==============================================================\n");
    printf("Final state:\n");
    printf("  Position: N=%.2fm E=%.2fm Alt=%.2fm\n",
           vehicle.pos_n, vehicle.pos_e, -vehicle.pos_d);
    printf("  Velocity: %.2f m/s\n",
           sqrtf(vehicle.vel_n*vehicle.vel_n + vehicle.vel_e*vehicle.vel_e + vehicle.vel_d*vehicle.vel_d));
    printf("\nLog saved to: flight_log.csv\n");
    printf("To visualize: python plot_flight.py\n");

    return 0;
}
