#include <stdio.h>
#include <string.h>
#include "fixed_point.h"
#include "arming.h"

/* Helper to create flight state with specific conditions */
flight_state_t make_flight_state(float gx, float gy, float gz, 
                                 float throttle, 
                                 flight_mode_t mode,
                                 bool imu_ok, bool sensor_ok) {
    flight_state_t state;
    state.gyro_x = fx_from_float(gx);
    state.gyro_y = fx_from_float(gy);
    state.gyro_z = fx_from_float(gz);
    state.throttle = fx_from_float(throttle);
    state.mode = mode;
    state.imu_calibrated = imu_ok;
    state.critical_sensor_fault = !sensor_ok;
    return state;
}

/* ============================================================================
 * Test 1: Successful Arming
 * ========================================================================= */
void test_successful_arming(void) {
    printf("=== Test 1: Successful Arming ===\n\n");
    
    arming_state_t arm_state = ARMING_DISARMED;
    
    /* Perfect conditions: gyro stationary, throttle min, STABILIZE mode */
    flight_state_t state = make_flight_state(
        0.0f, 0.0f, 0.0f,      /* Gyro stationary */
        0.0f,                  /* Throttle at zero */
        MODE_STABILIZE,        /* Correct mode */
        true, true             /* IMU ok, no sensor faults */
    );
    
    arming_result_t result = attempt_arm(&state, &arm_state);
    
    printf("Conditions: Gyro still, throttle min, mode=STABILIZE\n");
    printf("Result: %s\n", arming_result_name(result));
    printf("State: %s\n", arming_state_name(arm_state));
    printf("Status: %s\n\n", (result == ARM_SUCCESS && arm_state == ARMING_ARMED) ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 2: Gyro Moving
 * ========================================================================= */
void test_gyro_moving(void) {
    printf("=== Test 2: Arming Denied - Gyro Moving ===\n\n");
    
    arming_state_t arm_state = ARMING_DISARMED;
    arming_result_t result;
    
    /* Test 2a: Roll axis moving */
    flight_state_t state_roll = make_flight_state(
        0.1f, 0.0f, 0.0f,      /* Roll gyro > threshold (0.05) */
        0.0f, MODE_STABILIZE, true, true
    );
    result = attempt_arm(&state_roll, &arm_state);
    printf("Test 2a: Roll gyro = 0.1 rad/s (threshold 0.05)\n");
    printf("  Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_GYRO_MOVING) ? "✓" : "✗");
    
    /* Test 2b: Pitch axis moving */
    flight_state_t state_pitch = make_flight_state(
        0.0f, 0.08f, 0.0f,     /* Pitch gyro > threshold */
        0.0f, MODE_STABILIZE, true, true
    );
    result = attempt_arm(&state_pitch, &arm_state);
    printf("Test 2b: Pitch gyro = 0.08 rad/s\n");
    printf("  Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_GYRO_MOVING) ? "✓" : "✗");
    
    /* Test 2c: Yaw axis moving */
    flight_state_t state_yaw = make_flight_state(
        0.0f, 0.0f, -0.06f,    /* Yaw gyro > threshold */
        0.0f, MODE_STABILIZE, true, true
    );
    result = attempt_arm(&state_yaw, &arm_state);
    printf("Test 2c: Yaw gyro = -0.06 rad/s\n");
    printf("  Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_GYRO_MOVING) ? "✓" : "✗");
    
    /* Test 2d: Just below threshold (should succeed) */
    flight_state_t state_still = make_flight_state(
        0.04f, 0.03f, 0.02f,   /* All below 0.05 threshold */
        0.0f, MODE_STABILIZE, true, true
    );
    arm_state = ARMING_DISARMED;
    result = attempt_arm(&state_still, &arm_state);
    printf("Test 2d: All gyros < 0.05 rad/s (should ARM)\n");
    printf("  Result: %s %s\n\n", arming_result_name(result),
           (result == ARM_SUCCESS) ? "✓" : "✗");
}

/* ============================================================================
 * Test 3: Throttle High
 * ========================================================================= */
void test_throttle_high(void) {
    printf("=== Test 3: Arming Denied - Throttle High ===\n\n");
    
    arming_state_t arm_state = ARMING_DISARMED;
    arming_result_t result;
    
    /* Test 3a: Throttle at 50% */
    flight_state_t state_high = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.5f,                  /* Throttle at 50% */
        MODE_STABILIZE, true, true
    );
    result = attempt_arm(&state_high, &arm_state);
    printf("Test 3a: Throttle = 50%% (threshold 10%%)\n");
    printf("  Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_THROTTLE_HIGH) ? "✓" : "✗");
    
    /* Test 3b: Throttle just above threshold */
    flight_state_t state_above = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.11f,                 /* Just above 10% threshold */
        MODE_STABILIZE, true, true
    );
    result = attempt_arm(&state_above, &arm_state);
    printf("Test 3b: Throttle = 11%% (just above threshold)\n");
    printf("  Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_THROTTLE_HIGH) ? "✓" : "✗");
    
    /* Test 3c: Throttle just below threshold (should succeed) */
    flight_state_t state_below = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.09f,                 /* Just below 10% threshold */
        MODE_STABILIZE, true, true
    );
    arm_state = ARMING_DISARMED;
    result = attempt_arm(&state_below, &arm_state);
    printf("Test 3c: Throttle = 9%% (should ARM)\n");
    printf("  Result: %s %s\n\n", arming_result_name(result),
           (result == ARM_SUCCESS) ? "✓" : "✗");
}

/* ============================================================================
 * Test 4: Invalid Flight Mode
 * ========================================================================= */
void test_invalid_mode(void) {
    printf("=== Test 4: Arming Denied - Invalid Flight Mode ===\n\n");
    
    arming_state_t arm_state = ARMING_DISARMED;
    arming_result_t result;
    
    flight_mode_t invalid_modes[] = {
        MODE_ALT_HOLD,
        MODE_POS_HOLD,
        MODE_MISSION,
        MODE_RTL
    };
    
    for (int i = 0; i < 4; i++) {
        flight_state_t state = make_flight_state(
            0.0f, 0.0f, 0.0f,
            0.0f,
            invalid_modes[i],
            true, true
        );
        
        result = attempt_arm(&state, &arm_state);
        printf("Mode = %s: %s %s\n",
               flight_mode_name(invalid_modes[i]),
               arming_result_name(result),
               (result == ARM_DENIED_MODE_INVALID) ? "✓" : "✗");
    }
    
    printf("\n");
}

/* ============================================================================
 * Test 5: Sensor Faults
 * ========================================================================= */
void test_sensor_faults(void) {
    printf("=== Test 5: Arming Denied - Sensor Faults ===\n\n");
    
    arming_state_t arm_state = ARMING_DISARMED;
    arming_result_t result;
    
    /* Test 5a: IMU not calibrated */
    flight_state_t state_imu = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.0f, MODE_STABILIZE,
        false, true            /* IMU NOT calibrated */
    );
    result = attempt_arm(&state_imu, &arm_state);
    printf("Test 5a: IMU not calibrated\n");
    printf("  Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_IMU_FAULT) ? "✓" : "✗");
    
    /* Test 5b: Critical sensor fault */
    flight_state_t state_sensor = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.0f, MODE_STABILIZE,
        true, false            /* Sensor fault */
    );
    result = attempt_arm(&state_sensor, &arm_state);
    printf("Test 5b: Critical sensor fault\n");
    printf("  Result: %s %s\n\n", arming_result_name(result),
           (result == ARM_DENIED_SENSOR_FAULT) ? "✓" : "✗");
}

/* ============================================================================
 * Test 6: Already Armed
 * ========================================================================= */
void test_already_armed(void) {
    printf("=== Test 6: Arming Denied - Already Armed ===\n\n");
    
    arming_state_t arm_state = ARMING_ARMED;  /* Start armed */
    
    flight_state_t state = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.0f, MODE_STABILIZE, true, true
    );
    
    arming_result_t result = attempt_arm(&state, &arm_state);
    
    printf("Attempt to arm while already armed\n");
    printf("Result: %s %s\n", arming_result_name(result),
           (result == ARM_DENIED_ALREADY_ARMED) ? "✓" : "✗");
    printf("State remains: %s %s\n\n", arming_state_name(arm_state),
           (arm_state == ARMING_ARMED) ? "✓" : "✗");
}

/* ============================================================================
 * Test 7: Disarming
 * ========================================================================= */
void test_disarming(void) {
    printf("=== Test 7: Disarming ===\n\n");
    
    arming_state_t arm_state = ARMING_ARMED;
    
    printf("Initial state: %s\n", arming_state_name(arm_state));
    
    disarm(&arm_state);
    
    printf("After disarm(): %s\n", arming_state_name(arm_state));
    printf("Status: %s\n\n", (arm_state == ARMING_DISARMED) ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 8: Complete Arming Sequence
 * ========================================================================= */
void test_complete_sequence(void) {
    printf("=== Test 8: Complete Arming Sequence ===\n\n");
    
    arming_state_t arm_state = ARMING_DISARMED;
    arming_result_t result;
    
    printf("Step 1: Start disarmed\n");
    printf("  State: %s\n", arming_state_name(arm_state));
    
    /* Perfect conditions */
    flight_state_t state = make_flight_state(
        0.0f, 0.0f, 0.0f,
        0.0f, MODE_STABILIZE, true, true
    );
    
    printf("Step 2: Attempt arm with perfect conditions\n");
    result = attempt_arm(&state, &arm_state);
    printf("  Result: %s\n", arming_result_name(result));
    printf("  State: %s %s\n", arming_state_name(arm_state),
           (arm_state == ARMING_ARMED) ? "✓" : "✗");
    
    printf("Step 3: Verify is_armed() function\n");
    printf("  is_armed() = %s %s\n", is_armed(arm_state) ? "true" : "false",
           is_armed(arm_state) ? "✓" : "✗");
    
    printf("Step 4: Disarm\n");
    disarm(&arm_state);
    printf("  State: %s %s\n", arming_state_name(arm_state),
           (arm_state == ARMING_DISARMED) ? "✓" : "✗");
    
    printf("Step 5: Verify is_armed() returns false\n");
    printf("  is_armed() = %s %s\n\n", is_armed(arm_state) ? "true" : "false",
           !is_armed(arm_state) ? "✓" : "✗");
}

/* ============================================================================
 * Main Test Runner
 * ========================================================================= */
int main(void) {
    printf("==============================================================\n");
    printf(" ARMING STATE MACHINE TEST SUITE - Section 11.1\n");
    printf("==============================================================\n\n");
    
    test_successful_arming();
    test_gyro_moving();
    test_throttle_high();
    test_invalid_mode();
    test_sensor_faults();
    test_already_armed();
    test_disarming();
    test_complete_sequence();
    
    printf("==============================================================\n");
    printf(" ALL TESTS COMPLETE\n");
    printf("==============================================================\n");
    
    return 0;
}
