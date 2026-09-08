#include <stdio.h>
#include <string.h>
#include "fixed_point.h"
#include "flight_modes.h"

/* Helper to create RC input with specific switch position */
rc_input_t make_rc_input(rc_switch_position_t switch_pos, bool sticks_centered) {
    rc_input_t rc;
    rc.mode_switch_position = switch_pos;
    rc.arm_switch = false;
    rc.last_valid_frame_time_ms = 0;
    
    if (sticks_centered) {
        rc.roll = 0;
        rc.pitch = 0;
        rc.yaw = 0;
        rc.throttle = fx_from_float(0.5f);
    } else {
        /* Sticks deflected (for override test) */
        rc.roll = fx_from_float(0.3f);
        rc.pitch = fx_from_float(0.2f);
        rc.yaw = fx_from_float(0.1f);
        rc.throttle = fx_from_float(0.6f);
    }
    
    return rc;
}

/* Helper to create nav health with specific sensor states */
nav_health_t make_nav_health(bool baro, bool gps, bool home) {
    nav_health_t health;
    health.baro_ok = baro;
    health.gps_fix_ok = gps;
    health.home_settled = home;
    health.imu_ok = true;  /* Assume IMU always ok for these tests */
    health.mag_ok = true;
    return health;
}

/* ============================================================================
 * Test 1: Basic Mode Transitions (Healthy Sensors)
 * ========================================================================= */
void test_basic_transitions(void) {
    printf("=== Test 1: Basic Mode Transitions (All Sensors Healthy) ===\n\n");
    
    nav_health_t health = make_nav_health(true, true, true);  /* All sensors ok */
    flight_mode_t mode;
    
    /* Test 1a: Stabilize mode */
    rc_input_t rc = make_rc_input(SWITCH_STABILIZE, true);
    mode = evaluate_mode_transition(MODE_STABILIZE, &rc, &health, false);
    printf("Switch=STABILIZE → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    /* Test 1b: Altitude Hold */
    rc.mode_switch_position = SWITCH_ALT_HOLD;
    mode = evaluate_mode_transition(MODE_STABILIZE, &rc, &health, false);
    printf("Switch=ALT_HOLD → Mode=%s (Expected: ALT_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_ALT_HOLD) ? "✓" : "✗");
    
    /* Test 1c: Position Hold */
    rc.mode_switch_position = SWITCH_POS_HOLD;
    mode = evaluate_mode_transition(MODE_ALT_HOLD, &rc, &health, false);
    printf("Switch=POS_HOLD → Mode=%s (Expected: POS_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_POS_HOLD) ? "✓" : "✗");
    
    /* Test 1d: Mission (requires being in POS_HOLD first) */
    rc.mode_switch_position = SWITCH_AUTO;
    mode = evaluate_mode_transition(MODE_POS_HOLD, &rc, &health, true);
    printf("Switch=AUTO from POS_HOLD → Mode=%s (Expected: MISSION) %s\n",
           flight_mode_name(mode),
           (mode == MODE_MISSION) ? "✓" : "✗");
    
    printf("\n");
}

/* ============================================================================
 * Test 2: Entry Interlocks (Missing Sensors)
 * ========================================================================= */
void test_entry_interlocks(void) {
    printf("=== Test 2: Entry Interlocks (Sensor Failures) ===\n\n");
    
    rc_input_t rc = make_rc_input(SWITCH_ALT_HOLD, true);
    flight_mode_t mode;
    
    /* Test 2a: ALT_HOLD without barometer */
    nav_health_t no_baro = make_nav_health(false, true, true);
    mode = evaluate_mode_transition(MODE_STABILIZE, &rc, &no_baro, false);
    printf("ALT_HOLD request, baro=NO → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    /* Test 2b: POS_HOLD without GPS */
    rc.mode_switch_position = SWITCH_POS_HOLD;
    nav_health_t no_gps = make_nav_health(true, false, false);
    mode = evaluate_mode_transition(MODE_STABILIZE, &rc, &no_gps, false);
    printf("POS_HOLD request, GPS=NO → Mode=%s (Expected: ALT_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_ALT_HOLD) ? "✓" : "✗");
    
    /* Test 2c: POS_HOLD without barometer OR GPS */
    nav_health_t no_sensors = make_nav_health(false, false, false);
    mode = evaluate_mode_transition(MODE_STABILIZE, &rc, &no_sensors, false);
    printf("POS_HOLD request, baro=NO, GPS=NO → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    /* Test 2d: MISSION without mission uploaded */
    rc.mode_switch_position = SWITCH_AUTO;
    nav_health_t all_ok = make_nav_health(true, true, true);
    mode = evaluate_mode_transition(MODE_POS_HOLD, &rc, &all_ok, false);  /* mission_active=false */
    printf("AUTO request, mission=NO → Mode=%s (Expected: POS_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_POS_HOLD) ? "✓" : "✗");
    
    printf("\n");
}

/* ============================================================================
 * Test 3: Mission Entry Path (Must Transit Through POS_HOLD)
 * ========================================================================= */
void test_mission_entry_path(void) {
    printf("=== Test 3: Mission Entry Path (Must Go Through POS_HOLD) ===\n\n");
    
    nav_health_t health = make_nav_health(true, true, true);
    rc_input_t rc = make_rc_input(SWITCH_AUTO, true);
    flight_mode_t mode;
    
    /* Test 3a: Try to enter MISSION directly from STABILIZE (should fail) */
    mode = evaluate_mode_transition(MODE_STABILIZE, &rc, &health, true);
    printf("AUTO from STABILIZE → Mode=%s (Expected: POS_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_POS_HOLD) ? "✓" : "✗");
    
    /* Test 3b: Now that we are in POS_HOLD, AUTO should work */
    mode = evaluate_mode_transition(MODE_POS_HOLD, &rc, &health, true);
    printf("AUTO from POS_HOLD → Mode=%s (Expected: MISSION) %s\n",
           flight_mode_name(mode),
           (mode == MODE_MISSION) ? "✓" : "✗");
    
    /* Test 3c: Try from ALT_HOLD (should step to POS_HOLD first) */
    mode = evaluate_mode_transition(MODE_ALT_HOLD, &rc, &health, true);
    printf("AUTO from ALT_HOLD → Mode=%s (Expected: POS_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_POS_HOLD) ? "✓" : "✗");
    
    printf("\n");
}

/* ============================================================================
 * Test 4: Override Supremacy Rule
 * ========================================================================= */
void test_override_supremacy(void) {
    printf("=== Test 4: Override Supremacy Rule ===\n\n");
    
    nav_health_t health = make_nav_health(true, true, true);
    flight_mode_t mode;
    
    /* Test 4a: Stick deflection in MISSION mode */
    rc_input_t rc_deflected = make_rc_input(SWITCH_AUTO, false);  /* sticks NOT centered */
    mode = evaluate_mode_transition(MODE_MISSION, &rc_deflected, &health, true);
    printf("MISSION + stick deflection → Mode=%s (Expected: POS_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_POS_HOLD) ? "✓" : "✗");
    
    /* Test 4b: Stick deflection in RTL mode */
    mode = evaluate_mode_transition(MODE_RTL, &rc_deflected, &health, true);
    printf("RTL + stick deflection → Mode=%s (Expected: POS_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_POS_HOLD) ? "✓" : "✗");
    
    /* Test 4c: Centered sticks in MISSION (should stay in MISSION) */
    rc_input_t rc_centered = make_rc_input(SWITCH_AUTO, true);  /* sticks centered */
    mode = evaluate_mode_transition(MODE_MISSION, &rc_centered, &health, true);
    printf("MISSION + sticks centered → Mode=%s (Expected: MISSION) %s\n",
           flight_mode_name(mode),
           (mode == MODE_MISSION) ? "✓" : "✗");
    
    /* Test 4d: Override when GPS is lost - should fall back to STABILIZE */
    nav_health_t no_gps = make_nav_health(true, false, false);
    mode = evaluate_mode_transition(MODE_MISSION, &rc_deflected, &no_gps, true);
    printf("MISSION + stick deflection + GPS lost → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    printf("\n");
}

/* ============================================================================
 * Test 5: Failsafe Degradation (Sensor Loss During Flight)
 * ========================================================================= */
void test_failsafe_degradation(void) {
    printf("=== Test 5: Failsafe Degradation (Sensor Loss) ===\n\n");
    
    rc_input_t rc = make_rc_input(SWITCH_POS_HOLD, true);  /* Switch stays on POS_HOLD */
    flight_mode_t mode;
    
    /* Test 5a: GPS loss during POS_HOLD */
    nav_health_t gps_lost = make_nav_health(true, false, false);
    mode = evaluate_mode_transition(MODE_POS_HOLD, &rc, &gps_lost, false);
    printf("POS_HOLD, GPS lost → Mode=%s (Expected: ALT_HOLD) %s\n",
           flight_mode_name(mode),
           (mode == MODE_ALT_HOLD) ? "✓" : "✗");
    
    /* Test 5b: Both GPS and baro lost */
    nav_health_t all_lost = make_nav_health(false, false, false);
    mode = evaluate_mode_transition(MODE_POS_HOLD, &rc, &all_lost, false);
    printf("POS_HOLD, GPS+baro lost → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    /* Test 5c: Baro loss during ALT_HOLD */
    rc.mode_switch_position = SWITCH_ALT_HOLD;
    nav_health_t baro_lost = make_nav_health(false, true, true);
    mode = evaluate_mode_transition(MODE_ALT_HOLD, &rc, &baro_lost, false);
    printf("ALT_HOLD, baro lost → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    printf("\n");
}

/* ============================================================================
 * Test 6: Undefined Switch State (Failsafe)
 * ========================================================================= */
void test_undefined_switch(void) {
    printf("=== Test 6: Undefined Switch State ===\n\n");
    
    nav_health_t health = make_nav_health(true, true, true);
    rc_input_t rc = make_rc_input(SWITCH_UNDEFINED, true);
    
    flight_mode_t mode = evaluate_mode_transition(MODE_POS_HOLD, &rc, &health, false);
    printf("UNDEFINED switch → Mode=%s (Expected: STABILIZE) %s\n",
           flight_mode_name(mode),
           (mode == MODE_STABILIZE) ? "✓" : "✗");
    
    printf("\n");
}

/* ============================================================================
 * Main Test Runner
 * ========================================================================= */
int main(void) {
    printf("==============================================================\n");
    printf(" FLIGHT MODE STATE MACHINE TEST SUITE - Section 9\n");
    printf("==============================================================\n\n");
    
    test_basic_transitions();
    test_entry_interlocks();
    test_mission_entry_path();
    test_override_supremacy();
    test_failsafe_degradation();
    test_undefined_switch();
    
    printf("==============================================================\n");
    printf(" ALL TESTS COMPLETE\n");
    printf("==============================================================\n");
    
    return 0;
}
