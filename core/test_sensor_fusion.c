#include <stdio.h>
#include <math.h>
#include "fixed_point.h"
#include "sensor_fusion.h"

#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

void test_attitude_filter(void) {
    printf("=== Test 1: Roll/Pitch Complementary Filter ===\n");
    attitude_t att = {0, 0, 0};
    float roll_target = DEG_TO_RAD(30.0f);
    float g = 9.81f;
    printf("\nSimulating 30 deg roll tilt convergence (200 cycles at 100 Hz):\n");
    for (int i = 0; i < 200; i++) {
        imu_sample_t imu;
        imu.ax = fx_from_float(0.0f);
        imu.ay = fx_from_float(g * sinf(roll_target));
        imu.az = fx_from_float(g * cosf(roll_target));
        imu.gx = 0; imu.gy = 0; imu.gz = 0;
        update_attitude_complementary(&imu, &att);
        if (i % 40 == 0 || i == 199) {
            printf("  Cycle %3d: Roll = %6.2f deg (Target: 30.00), Pitch = %6.2f deg\n",
                   i, RAD_TO_DEG(fx_to_float(att.roll)), RAD_TO_DEG(fx_to_float(att.pitch)));
        }
    }
    float final_roll_error = fabsf(fx_to_float(att.roll) - roll_target);
    printf("\n  Final roll error: %.4f rad (%.2f deg)\n", final_roll_error, RAD_TO_DEG(final_roll_error));
    printf("  Status: %s\n\n", final_roll_error < 0.05f ? "PASS" : "FAIL");
}

void test_yaw_fusion(void) {
    printf("=== Test 2: Yaw Fusion (Magnetometer + Gyro) ===\n");
    attitude_t att = {0, 0, 0};
    float heading_target = DEG_TO_RAD(45.0f);
    printf("\nSimulating 45 deg heading, 30 magnetometer corrections:\n");
    for (int i = 0; i < 300; i++) {
        integrate_yaw(0, &att);
        if (i % 10 == 0) {
            mag_sample_t mag;
            mag.mx = fx_from_float(cosf(heading_target));
            mag.my = fx_from_float(-sinf(heading_target)); /* NED: north=mx, east=my */
            mag.mz = fx_from_float(0.0f);
            fx_t mag_heading = compute_mag_heading(&mag, &att);
            correct_yaw_with_mag(mag_heading, &att);
            if ((i / 10) % 5 == 0 || i == 290) {
                printf("  Correction %2d: Yaw = %6.2f deg (Target: 45.00)\n", i / 10, RAD_TO_DEG(fx_to_float(att.yaw)));
            }
        }
    }
    float final_yaw_error = fabsf(fx_to_float(att.yaw) - heading_target);
    printf("\n  Final yaw error: %.4f rad (%.2f deg)\n", final_yaw_error, RAD_TO_DEG(final_yaw_error));
    printf("  Status: %s\n\n", final_yaw_error < 0.05f ? "PASS" : "FAIL");
}

void test_altitude_filter(void) {
    printf("=== Test 3: Altitude Filter ===\n");
    altitude_state_t alt = {0, 0, 0};
    float climb_rate = 2.0f;
    float target_altitude = 10.0f;
    fx_t dt = DT_MEDIUM_Q16;
    printf("\nSimulating 2 m/s climb for 5 seconds (50 cycles at 10 Hz):\n");
    for (int i = 0; i < 50; i++) {
        float time = i * 0.1f;
        float true_alt = climb_rate * time;
        float baro_noise = 0.1f * sinf(i * 0.5f);
        fx_t baro_alt = fx_from_float(true_alt + baro_noise);
        fx_t accel_z = fx_from_float(0.0f);
        update_altitude_filter(accel_z, baro_alt, dt, &alt);
        if (i % 10 == 0 || i == 49) {
            printf("  Time %4.1fs: Alt = %6.2f m (True: %5.2f m), VelZ = %5.2f m/s\n",
                   time, fx_to_float(alt.altitude), true_alt, fx_to_float(alt.velocity_z));
        }
    }
    float final_alt_error = fabsf(fx_to_float(alt.altitude) - target_altitude);
    printf("\n  Final altitude error: %.2f m\n", final_alt_error);
    printf("  Status: %s\n\n", final_alt_error < 0.5f ? "PASS" : "FAIL");
}

void test_position_filter(void) {
    printf("=== Test 4: Horizontal Position Filter ===\n");
    nav_state_t nav = {0, 0, 0, 0};
    fx_t dt = DT_NAV_Q16;
    printf("\nSimulating movement: 0->50m North tracking GPS fixes:\n");
    for (int i = 0; i < 30; i++) {
        float time = i * 0.2f;
        float true_vel = 10.0f;
        float true_pos = 10.0f * time;
        /* Initial acceleration pulse to bring velocity to 10 m/s (50 m/s² for 0.2s) */
        fx_t accel_n = (i == 0) ? fx_from_float(50.0f) : fx_from_float(0.0f);
        fx_t accel_e = fx_from_float(0.0f);
        gps_fix_ned_t *fix_ptr = NULL;
        gps_fix_ned_t gps_fix;
        if (i < 25) {
            gps_fix.pos_n = fx_from_float(true_pos + 0.3f * sinf(i * 0.5f));
            gps_fix.pos_e = fx_from_float(0.0f);
            gps_fix.vel_n = fx_from_float(true_vel);
            gps_fix.vel_e = fx_from_float(0.0f);
            gps_fix.quality_ok = true;
            fix_ptr = &gps_fix;
        }
        update_horizontal_nav(accel_n, accel_e, dt, fix_ptr, &nav);
        if (i % 5 == 0 || i == 29) {
            printf("  Time %4.1fs: PosN = %6.2f m (True: %5.2f m), VelN = %5.2f m/s %s\n",
                   time, fx_to_float(nav.pos_n), (time < 5.0f ? true_pos : 50.0f), fx_to_float(nav.vel_n),
                   (fix_ptr == NULL) ? "[GPS LOSS - DEAD RECKONING]" : "");
        }
    }
    /* At t=5.8s (i=29), with dead reckoning coasting at 10 m/s, position reaches ~58.0m */
    float final_pos_error = fabsf(fx_to_float(nav.pos_n) - 58.0f);
    printf("\n  Dead reckoning pos at t=5.8s: %.2f m (Error vs 58.0m projected: %.2f m)\n",
           fx_to_float(nav.pos_n), final_pos_error);
    printf("  Status: %s\n\n", final_pos_error < 2.0f ? "PASS" : "FAIL");
}

int main(void) {
    printf("==============================================================\n");
    printf(" SENSOR FUSION TEST SUITE - Section 6\n");
    printf("==============================================================\n\n");
    test_attitude_filter();
    test_yaw_fusion();
    test_altitude_filter();
    test_position_filter();
    printf("==============================================================\n");
    printf(" ALL TESTS COMPLETE\n");
    printf("==============================================================\n");
    return 0;
}
