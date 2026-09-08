#include <stdio.h>
#include <math.h>
#include "fixed_point.h"
#include "rotation.h"

#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

/* Test tolerance for rotation matrices (CORDIC + rounding errors) */
#define ROTATION_TOLERANCE 0.01f  /* 1% or ~0.01 m/s² for 1G */

/* ============================================================================
 * Test 1: Identity Rotation (Zero Attitude)
 * ========================================================================= */
void test_identity_rotation(void) {
    printf("=== Test 1: Identity Rotation (Level Flight) ===\n");
    printf("Roll=Pitch=Yaw=0, vector should pass through unchanged\n\n");
    
    fx_t accel_x = fx_from_float(1.0f);  /* 1 m/s² forward */
    fx_t accel_y = fx_from_float(2.0f);  /* 2 m/s² right */
    fx_t accel_z = fx_from_float(3.0f);  /* 3 m/s² down */
    
    fx_t roll = 0, pitch = 0, yaw = 0;
    
    /* Body → World */
    fx_t accel_n, accel_e, accel_d;
    rotate_accel_to_world(accel_x, accel_y, accel_z, roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d);
    
    printf("Body→World: [%.2f, %.2f, %.2f] → [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_x), fx_to_float(accel_y), fx_to_float(accel_z),
           fx_to_float(accel_n), fx_to_float(accel_e), fx_to_float(accel_d));
    
    /* World → Body (should be inverse) */
    fx_t accel_x2, accel_y2, accel_z2;
    rotate_accel_to_body(accel_n, accel_e, accel_d, roll, pitch, yaw,
                         &accel_x2, &accel_y2, &accel_z2);
    
    printf("World→Body: [%.2f, %.2f, %.2f] → [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_n), fx_to_float(accel_e), fx_to_float(accel_d),
           fx_to_float(accel_x2), fx_to_float(accel_y2), fx_to_float(accel_z2));
    
    bool pass = (fabsf(fx_to_float(accel_n) - 1.0f) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_e) - 2.0f) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_d) - 3.0f) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_x2) - 1.0f) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_y2) - 2.0f) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_z2) - 3.0f) < ROTATION_TOLERANCE);
    
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 2: Pure Roll (90° Right Roll)
 * ========================================================================= */
void test_pure_roll(void) {
    printf("=== Test 2: Pure Roll (90° Right) ===\n");
    printf("Body +Y (right) should rotate to world -D (up)\n\n");
    
    fx_t accel_x = fx_from_float(0.0f);
    fx_t accel_y = fx_from_float(1.0f);  /* 1 m/s² right in body frame */
    fx_t accel_z = fx_from_float(0.0f);
    
    fx_t roll = fx_from_float(DEG_TO_RAD(90.0f));
    fx_t pitch = 0, yaw = 0;
    
    fx_t accel_n, accel_e, accel_d;
    rotate_accel_to_world(accel_x, accel_y, accel_z, roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d);
    
    printf("Body [0, 1, 0] at roll=90° → World [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_n), fx_to_float(accel_e), fx_to_float(accel_d));
    printf("Expected: [0, 0, +1] (body right rotates to world down in NED frame)\n");

    bool pass = (fabsf(fx_to_float(accel_n)) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_e)) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_d) - 1.0f) < ROTATION_TOLERANCE);
    
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 3: Pure Pitch (45° Nose Up)
 * ========================================================================= */
void test_pure_pitch(void) {
    printf("=== Test 3: Pure Pitch (45° Nose Up) ===\n");
    printf("Body +X (forward) should rotate partially up and forward\n\n");
    
    fx_t accel_x = fx_from_float(1.0f);  /* 1 m/s² forward in body frame */
    fx_t accel_y = fx_from_float(0.0f);
    fx_t accel_z = fx_from_float(0.0f);
    
    fx_t roll = 0;
    fx_t pitch = fx_from_float(DEG_TO_RAD(45.0f));
    fx_t yaw = 0;
    
    fx_t accel_n, accel_e, accel_d;
    rotate_accel_to_world(accel_x, accel_y, accel_z, roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d);
    
    printf("Body [1, 0, 0] at pitch=45° → World [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_n), fx_to_float(accel_e), fx_to_float(accel_d));
    
    /* At 45° pitch: N ≈ cos(45°) = 0.707, D ≈ -sin(45°) = -0.707 */
    float expected_n = cosf(DEG_TO_RAD(45.0f));
    float expected_d = -sinf(DEG_TO_RAD(45.0f));
    printf("Expected: [%.2f, 0, %.2f]\n", expected_n, expected_d);
    
    bool pass = (fabsf(fx_to_float(accel_n) - expected_n) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_e)) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_d) - expected_d) < ROTATION_TOLERANCE);
    
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 4: Pure Yaw (90° Right Turn)
 * ========================================================================= */
void test_pure_yaw(void) {
    printf("=== Test 4: Pure Yaw (90° Right/East) ===\n");
    printf("Body +X (forward) should rotate to world +E (east)\n\n");
    
    fx_t accel_x = fx_from_float(1.0f);  /* 1 m/s² forward in body frame */
    fx_t accel_y = fx_from_float(0.0f);
    fx_t accel_z = fx_from_float(0.0f);
    
    fx_t roll = 0, pitch = 0;
    fx_t yaw = fx_from_float(DEG_TO_RAD(90.0f));
    
    fx_t accel_n, accel_e, accel_d;
    rotate_accel_to_world(accel_x, accel_y, accel_z, roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d);
    
    printf("Body [1, 0, 0] at yaw=90° → World [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_n), fx_to_float(accel_e), fx_to_float(accel_d));
    printf("Expected: [0, 1, 0] (body forward becomes world east)\n");
    
    bool pass = (fabsf(fx_to_float(accel_n)) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_e) - 1.0f) < ROTATION_TOLERANCE) &&
                (fabsf(fx_to_float(accel_d)) < ROTATION_TOLERANCE);
    
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 5: Combined Attitude (Roll + Pitch + Yaw)
 * ========================================================================= */
void test_combined_attitude(void) {
    printf("=== Test 5: Combined Attitude (30° Roll, 20° Pitch, 45° Yaw) ===\n");
    printf("Testing full 3D rotation with all axes\n\n");
    
    fx_t accel_x = fx_from_float(0.0f);
    fx_t accel_y = fx_from_float(0.0f);
    fx_t accel_z = fx_from_float(9.81f);  /* Gravity (down) in body frame */
    
    fx_t roll = fx_from_float(DEG_TO_RAD(30.0f));
    fx_t pitch = fx_from_float(DEG_TO_RAD(20.0f));
    fx_t yaw = fx_from_float(DEG_TO_RAD(45.0f));
    
    fx_t accel_n, accel_e, accel_d;
    rotate_accel_to_world(accel_x, accel_y, accel_z, roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d);
    
    printf("Body [0, 0, 9.81] at (R=30°, P=20°, Y=45°) → World [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_n), fx_to_float(accel_e), fx_to_float(accel_d));
    printf("Down component should still be close to 9.81 (gravity always points down)\n");
    
    /* Gravity magnitude should be preserved (rotation is orthogonal) */
    float mag_in = 9.81f;
    float mag_out = sqrtf(fx_to_float(accel_n) * fx_to_float(accel_n) +
                          fx_to_float(accel_e) * fx_to_float(accel_e) +
                          fx_to_float(accel_d) * fx_to_float(accel_d));
    printf("Magnitude: in=%.2f, out=%.2f (should be equal)\n", mag_in, mag_out);
    
    bool pass = (fabsf(mag_out - mag_in) < 0.1f);  /* 0.1 m/s² tolerance for magnitude */
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 6: Roundtrip (Body→World→Body)
 * ========================================================================= */
void test_roundtrip(void) {
    printf("=== Test 6: Roundtrip Consistency (Body→World→Body) ===\n");
    printf("Vector should return to original after forward + inverse rotation\n\n");
    
    fx_t accel_x_orig = fx_from_float(1.5f);
    fx_t accel_y_orig = fx_from_float(-2.3f);
    fx_t accel_z_orig = fx_from_float(3.7f);
    
    fx_t roll = fx_from_float(DEG_TO_RAD(25.0f));
    fx_t pitch = fx_from_float(DEG_TO_RAD(-15.0f));
    fx_t yaw = fx_from_float(DEG_TO_RAD(60.0f));
    
    /* Body → World */
    fx_t accel_n, accel_e, accel_d;
    rotate_accel_to_world(accel_x_orig, accel_y_orig, accel_z_orig,
                          roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d);
    
    /* World → Body */
    fx_t accel_x_final, accel_y_final, accel_z_final;
    rotate_accel_to_body(accel_n, accel_e, accel_d,
                         roll, pitch, yaw,
                         &accel_x_final, &accel_y_final, &accel_z_final);
    
    printf("Original: [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_x_orig), fx_to_float(accel_y_orig), fx_to_float(accel_z_orig));
    printf("After roundtrip: [%.2f, %.2f, %.2f]\n",
           fx_to_float(accel_x_final), fx_to_float(accel_y_final), fx_to_float(accel_z_final));
    
    float err_x = fabsf(fx_to_float(accel_x_final) - fx_to_float(accel_x_orig));
    float err_y = fabsf(fx_to_float(accel_y_final) - fx_to_float(accel_y_orig));
    float err_z = fabsf(fx_to_float(accel_z_final) - fx_to_float(accel_z_orig));
    printf("Errors: X=%.4f, Y=%.4f, Z=%.4f\n", err_x, err_y, err_z);
    
    bool pass = (err_x < ROTATION_TOLERANCE) &&
                (err_y < ROTATION_TOLERANCE) &&
                (err_z < ROTATION_TOLERANCE);
    
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Test 7: Vertical Extraction Optimization
 * ========================================================================= */
void test_vertical_extraction(void) {
    printf("=== Test 7: Vertical Extraction (Optimized vs Full) ===\n");
    printf("extract_vertical_accel() should match full rotation Down component\n\n");
    
    fx_t accel_x = fx_from_float(1.0f);
    fx_t accel_y = fx_from_float(0.5f);
    fx_t accel_z = fx_from_float(9.81f);
    
    fx_t roll = fx_from_float(DEG_TO_RAD(10.0f));
    fx_t pitch = fx_from_float(DEG_TO_RAD(15.0f));
    fx_t yaw = fx_from_float(DEG_TO_RAD(45.0f));  /* Not used by extract_vertical */
    
    /* Full rotation */
    fx_t accel_n, accel_e, accel_d_full;
    rotate_accel_to_world(accel_x, accel_y, accel_z, roll, pitch, yaw,
                          &accel_n, &accel_e, &accel_d_full);
    
    /* Optimized vertical-only */
    fx_t accel_d_opt = extract_vertical_accel(accel_x, accel_y, accel_z, roll, pitch);
    
    printf("Full rotation Down: %.4f\n", fx_to_float(accel_d_full));
    printf("Optimized extract:  %.4f\n", fx_to_float(accel_d_opt));
    printf("Difference: %.4f\n", fabsf(fx_to_float(accel_d_full) - fx_to_float(accel_d_opt)));
    
    bool pass = (fabsf(fx_to_float(accel_d_full) - fx_to_float(accel_d_opt)) < 0.001f);
    printf("Status: %s\n\n", pass ? "PASS ✓" : "FAIL ✗");
}

/* ============================================================================
 * Main Test Runner
 * ========================================================================= */
int main(void) {
    printf("==============================================================\n");
    printf(" ROTATION MATRIX TEST SUITE\n");
    printf("==============================================================\n\n");
    
    test_identity_rotation();
    test_pure_roll();
    test_pure_pitch();
    test_pure_yaw();
    test_combined_attitude();
    test_roundtrip();
    test_vertical_extraction();
    
    printf("==============================================================\n");
    printf(" ALL TESTS COMPLETE\n");
    printf("==============================================================\n");
    
    return 0;
}
