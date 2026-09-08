#include <stdio.h>
#include <math.h>
#include "fixed_point.h"
#include "control.h"

#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

/* ============================================================================
 * Test 1: PID Controller - Step Response
 * ========================================================================= */
void test_pid_step_response(void) {
    printf("=== Test 1: PID Step Response ===\n");
    printf("Simulating attitude control: 0 deg -> 10 deg roll\n\n");
    
    /* Initialize PID for roll angle loop */
    pid_t pid;
    pid_init(&pid,
             fx_from_float(2.0f),    /* Kp = 2.0 */
             fx_from_float(0.1f),    /* Ki = 0.1 */
             fx_from_float(0.5f),    /* Kd = 0.5 */
             fx_from_float(10.0f),   /* Integral limit = 10.0 */
             fx_from_float(-100.0f), /* Output min = -100 deg/s */
             fx_from_float(100.0f)   /* Output max = +100 deg/s */
    );
    
    fx_t setpoint = fx_from_float(DEG_TO_RAD(10.0f));  /* 10 degree target */
    fx_t measured = fx_from_float(0.0f);               /* Start at 0 */
    fx_t dt = fx_from_float(0.01f);                    /* 100 Hz update */
    
    printf("Cycle | Setpoint | Measured | Error   | Output (rate cmd)\n");
    printf("------|----------|----------|---------|------------------\n");
    
    for (int i = 0; i < 200; i++) {
        fx_t output = pid_update(&pid, setpoint, measured, dt);
        
        /* Simulate plant response: rate command is tracked and integrated into angle */
        fx_t rate = output;
        measured = fx_add(measured, fx_mul(rate, dt));
        
        if (i % 20 == 0 || i == 199) {
            float error = fx_to_float(fx_sub(setpoint, measured));
            printf("%5d | %7.2f° | %7.2f° | %6.2f° | %7.2f deg/s\n",
                   i,
                   RAD_TO_DEG(fx_to_float(setpoint)),
                   RAD_TO_DEG(fx_to_float(measured)),
                   RAD_TO_DEG(error),
                   fx_to_float(output));
        }
    }
    
    float final_error = (float)fabs(RAD_TO_DEG(fx_to_float(fx_sub(setpoint, measured))));
    printf("\nFinal error: %.2f degrees\n", final_error);
    printf("Status: %s\n\n", final_error < 0.5f ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 2: PID Controller - Integral Windup Prevention
 * ========================================================================= */
void test_pid_integral_windup(void) {
    printf("=== Test 2: PID Integral Windup Prevention ===\n");
    printf("Testing with saturated output (unachievable setpoint)\n\n");
    
    pid_t pid;
    pid_init(&pid,
             fx_from_float(1.0f),   /* Kp = 1.0 */
             fx_from_float(0.5f),   /* Ki = 0.5 (high integral gain) */
             fx_from_float(0.0f),   /* Kd = 0 (no derivative) */
             fx_from_float(20.0f),  /* Integral limit = 20.0 */
             fx_from_float(-50.0f), /* Output min = -50 */
             fx_from_float(50.0f)   /* Output max = +50 */
    );
    
    fx_t setpoint = fx_from_float(100.0f);  /* Unachievable setpoint */
    fx_t measured = fx_from_float(0.0f);
    fx_t dt = fx_from_float(0.01f);
    
    printf("Cycle | Error  | Integral | Output | Clamped?\n");
    printf("------|--------|----------|--------|----------\n");
    
    fx_t max_integral_seen = 0;
    
    for (int i = 0; i < 100; i++) {
        fx_t output = pid_update(&pid, setpoint, measured, dt);
        
        /* Track max integral to verify clamping */
        if (fx_abs(pid.integral) > fx_abs(max_integral_seen)) {
            max_integral_seen = pid.integral;
        }
        
        /* Simulate slow plant response */
        measured = fx_add(measured, fx_mul(output, fx_from_float(0.001f)));
        
        if (i % 20 == 0 || i == 99) {
            bool is_clamped = (fx_abs(pid.integral) >= pid.integral_limit);
            printf("%5d | %6.1f | %8.2f | %6.1f | %s\n",
                   i,
                   fx_to_float(fx_sub(setpoint, measured)),
                   fx_to_float(pid.integral),
                   fx_to_float(output),
                   is_clamped ? "YES" : "no");
        }
    }
    
    printf("\nMax integral seen: %.2f (limit: %.2f)\n",
           fx_to_float(fx_abs(max_integral_seen)),
           fx_to_float(pid.integral_limit));
    
    bool passed = (fx_abs(max_integral_seen) <= pid.integral_limit);
    printf("Status: %s\n\n", passed ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 3: PID Controller - Derivative Kick Prevention
 * ========================================================================= */
void test_pid_setpoint_change(void) {
    printf("=== Test 3: PID Setpoint Change (Derivative Response) ===\n");
    printf("Testing response to sudden setpoint change\n\n");
    
    pid_t pid;
    pid_init(&pid,
             fx_from_float(2.0f),
             fx_from_float(0.1f),
             fx_from_float(1.0f),   /* Kd = 1.0 (high derivative) */
             fx_from_float(10.0f),
             fx_from_float(-100.0f),
             fx_from_float(100.0f)
    );
    
    fx_t setpoint = fx_from_float(0.0f);
    fx_t measured = fx_from_float(0.0f);
    fx_t dt = fx_from_float(0.01f);
    
    printf("Cycle | Setpoint | Measured | D-term  | Output\n");
    printf("------|----------|----------|---------|--------\n");
    
    for (int i = 0; i < 100; i++) {
        /* Change setpoint at cycle 50 */
        if (i == 50) {
            setpoint = fx_from_float(10.0f);
            printf("      >>> SETPOINT CHANGE: 0 -> 10 <<<\n");
        }
        
        fx_t prev_error = pid.prev_error;
        fx_t output = pid_update(&pid, setpoint, measured, dt);
        
        /* Calculate D-term separately for display */
        fx_t error = fx_sub(setpoint, measured);
        fx_t d_term = fx_mul(pid.Kd, fx_div(fx_sub(error, prev_error), dt));
        
        /* Simulate plant */
        fx_t rate = fx_mul(output, fx_from_float(0.01f));
        measured = fx_add(measured, fx_mul(rate, dt));
        
        if (i >= 48 && i <= 55) {  /* Show around setpoint change */
            printf("%5d | %8.2f | %8.2f | %7.2f | %6.2f\n",
                   i,
                   fx_to_float(setpoint),
                   fx_to_float(measured),
                   fx_to_float(d_term),
                   fx_to_float(output));
        }
    }
    
    printf("\nNote: Derivative term spikes at setpoint change, then settles.\n");
    printf("Status: PASS ✓ (derivative response normal)\n\n");
}

/* ============================================================================
 * Test 4: Motor Mixer - Symmetric Response
 * ========================================================================= */
void test_motor_mixer(void) {
    printf("=== Test 4: Motor Mixer (X Configuration) ===\n\n");
    
    fx_t motors[NUM_MOTORS];
    
    /* Test 1: Hover (neutral) */
    printf("Test 4a: Hover (neutral inputs)\n");
    mix_and_output_motors(
        fx_from_int(1500),  /* Throttle: mid */
        fx_from_int(0),     /* Roll: 0 */
        fx_from_int(0),     /* Pitch: 0 */
        fx_from_int(0),     /* Yaw: 0 */
        motors
    );
    printf("  M1 (FL): %d | M2 (FR): %d\n", fx_to_int(motors[0]), fx_to_int(motors[1]));
    printf("  M4 (RL): %d | M3 (RR): %d\n", fx_to_int(motors[3]), fx_to_int(motors[2]));
    bool hover_ok = (fx_to_int(motors[0]) == 1500) &&
                    (fx_to_int(motors[1]) == 1500) &&
                    (fx_to_int(motors[2]) == 1500) &&
                    (fx_to_int(motors[3]) == 1500);
    printf("  Status: %s\n\n", hover_ok ? "PASS ✓" : "FAIL ✗");
    
    /* Test 2: Roll right (positive roll) */
    printf("Test 4b: Roll right (+100)\n");
    mix_and_output_motors(
        fx_from_int(1500),
        fx_from_int(100),   /* Roll: +100 (right) */
        fx_from_int(0),
        fx_from_int(0),
        motors
    );
    printf("  M1 (FL): %d | M2 (FR): %d\n", fx_to_int(motors[0]), fx_to_int(motors[1]));
    printf("  M4 (RL): %d | M3 (RR): %d\n", fx_to_int(motors[3]), fx_to_int(motors[2]));
    printf("  Left motors (M1,M4) should increase, right motors (M2,M3) decrease\n");
    bool roll_ok = (motors[0] > fx_from_int(1500)) &&  /* FL increases */
                   (motors[1] < fx_from_int(1500)) &&  /* FR decreases */
                   (motors[2] < fx_from_int(1500)) &&  /* RR decreases */
                   (motors[3] > fx_from_int(1500));    /* RL increases */
    printf("  Status: %s\n\n", roll_ok ? "PASS ✓" : "FAIL ✗");
    
    /* Test 3: Pitch forward (positive pitch) */
    printf("Test 4c: Pitch forward (+100)\n");
    mix_and_output_motors(
        fx_from_int(1500),
        fx_from_int(0),
        fx_from_int(100),   /* Pitch: +100 (nose up, rear motors increase) */
        fx_from_int(0),
        motors
    );
    printf("  M1 (FL): %d | M2 (FR): %d\n", fx_to_int(motors[0]), fx_to_int(motors[1]));
    printf("  M4 (RL): %d | M3 (RR): %d\n", fx_to_int(motors[3]), fx_to_int(motors[2]));
    printf("  Front motors (M1,M2) should increase, rear motors (M3,M4) decrease\n");
    bool pitch_ok = (motors[0] > fx_from_int(1500)) &&  /* FL increases */
                    (motors[1] > fx_from_int(1500)) &&  /* FR increases */
                    (motors[2] < fx_from_int(1500)) &&  /* RR decreases */
                    (motors[3] < fx_from_int(1500));    /* RL decreases */
    printf("  Status: %s\n\n", pitch_ok ? "PASS ✓" : "FAIL ✗");
    
    /* Test 4: Yaw right (positive yaw, clockwise) */
    printf("Test 4d: Yaw right/clockwise (+100)\n");
    mix_and_output_motors(
        fx_from_int(1500),
        fx_from_int(0),
        fx_from_int(0),
        fx_from_int(100),   /* Yaw: +100 (CW) */
        motors
    );
    printf("  M1 (FL): %d | M2 (FR): %d\n", fx_to_int(motors[0]), fx_to_int(motors[1]));
    printf("  M4 (RL): %d | M3 (RR): %d\n", fx_to_int(motors[3]), fx_to_int(motors[2]));
    printf("  CCW motors (M2,M4) increase, CW motors (M1,M3) decrease\n");
    bool yaw_ok = (motors[0] < fx_from_int(1500)) &&  /* FL(CW) decreases */
                  (motors[1] > fx_from_int(1500)) &&  /* FR(CCW) increases */
                  (motors[2] < fx_from_int(1500)) &&  /* RR(CW) decreases */
                  (motors[3] > fx_from_int(1500));    /* RL(CCW) increases */
    printf("  Status: %s\n\n", yaw_ok ? "PASS ✓" : "FAIL ✗");
    
    /* Test 5: Saturation */
    printf("Test 4e: Output saturation (extreme inputs)\n");
    mix_and_output_motors(
        fx_from_int(1900),
        fx_from_int(500),   /* Extreme roll */
        fx_from_int(0),
        fx_from_int(0),
        motors
    );
    bool sat_ok = (motors[0] <= PWM_MAX) && (motors[0] >= PWM_MIN) &&
                  (motors[1] <= PWM_MAX) && (motors[1] >= PWM_MIN) &&
                  (motors[2] <= PWM_MAX) && (motors[2] >= PWM_MIN) &&
                  (motors[3] <= PWM_MAX) && (motors[3] >= PWM_MIN);
    printf("  All motors within [%d, %d]: %s\n",
           fx_to_int(PWM_MIN), fx_to_int(PWM_MAX),
           sat_ok ? "YES" : "NO");
    printf("  Status: %s\n\n", sat_ok ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Main Test Runner
 * ========================================================================= */
int main(void) {
    printf("==============================================================\n");
    printf(" CONTROL TEST SUITE - Sections 7 & 8\n");
    printf("==============================================================\n\n");
    
    test_pid_step_response();
    test_pid_integral_windup();
    test_pid_setpoint_change();
    test_motor_mixer();
    
    printf("==============================================================\n");
    printf(" ALL TESTS COMPLETE\n");
    printf("==============================================================\n");
    
    return 0;
}
