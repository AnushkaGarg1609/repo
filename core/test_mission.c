#include <stdio.h>
#include <string.h>
#include "fixed_point.h"
#include "mission.h"

/* Helper to create navigation state */
nav_state_3d_t make_nav_state(float n, float e, float d) {
    nav_state_3d_t state;
    state.north = fx_from_float(n);
    state.east = fx_from_float(e);
    state.down = fx_from_float(d);
    return state;
}

/* Helper to create waypoint */
waypoint_t make_waypoint(float n, float e, float d, float radius) {
    waypoint_t wp;
    wp.north = fx_from_float(n);
    wp.east = fx_from_float(e);
    wp.down = fx_from_float(d);
    wp.acceptance_radius = fx_from_float(radius);
    return wp;
}

/* Helper to print position */
void print_position(const char* label, fx_t n, fx_t e, fx_t d) {
    printf("%s: N=%.2f E=%.2f D=%.2f\n", label,
           fx_to_float(n), fx_to_float(e), fx_to_float(d));
}

/* ============================================================================
 * Test 1: Mission Loading and Basic State
 * ========================================================================= */
void test_mission_loading(void) {
    printf("=== Test 1: Mission Loading ===\n\n");
    
    mission_t mission;
    
    /* Test 1a: Empty mission */
    mission_clear(&mission);
    printf("Test 1a: Empty mission\n");
    printf("  is_loaded() = %s %s\n", mission_is_loaded(&mission) ? "true" : "false",
           !mission_is_loaded(&mission) ? "✓" : "✗");
    printf("  is_complete() = %s %s\n", mission_is_complete(&mission) ? "true" : "false",
           mission_is_complete(&mission) ? "✓" : "✗");
    
    /* Test 1b: Load valid mission */
    waypoint_t waypoints[4] = {
        make_waypoint(0.0f, 0.0f, -10.0f, 2.0f),   /* Home/start */
        make_waypoint(10.0f, 0.0f, -10.0f, 2.0f),  /* North 10m */
        make_waypoint(10.0f, 10.0f, -10.0f, 2.0f), /* East 10m */
        make_waypoint(0.0f, 0.0f, -10.0f, 2.0f)    /* Return home */
    };
    
    bool loaded = mission_load(&mission, waypoints, 4, false);
    printf("\nTest 1b: Load 4-waypoint mission\n");
    printf("  mission_load() = %s %s\n", loaded ? "true" : "false",
           loaded ? "✓" : "✗");
    printf("  count = %d %s\n", mission.count, (mission.count == 4) ? "✓" : "✗");
    printf("  current_index = %d %s\n", mission.current_index,
           (mission.current_index == 0) ? "✓" : "✗");
    printf("  is_loaded() = %s %s\n", mission_is_loaded(&mission) ? "true" : "false",
           mission_is_loaded(&mission) ? "✓" : "✗");
    printf("  is_complete() = %s %s\n\n", mission_is_complete(&mission) ? "true" : "false",
           !mission_is_complete(&mission) ? "✓" : "✗");
}

/* ============================================================================
 * Test 2: Distance Calculations
 * ========================================================================= */
void test_distance_calculations(void) {
    printf("=== Test 2: Distance Calculations ===\n\n");

    nav_state_3d_t pos = make_nav_state(0.0f, 0.0f, -10.0f);
    
    /* Test 2a: Zero distance (at waypoint) */
    waypoint_t wp_zero = make_waypoint(0.0f, 0.0f, -10.0f, 2.0f);
    fx_t dist_zero = waypoint_distance(&pos, &wp_zero);
    printf("Test 2a: Distance to same position\n");
    printf("  Distance = %.2f m %s\n", fx_to_float(dist_zero),
           (fx_to_float(dist_zero) < 0.01f) ? "✓" : "✗");
    
    /* Test 2b: Pure north */
    waypoint_t wp_north = make_waypoint(10.0f, 0.0f, -10.0f, 2.0f);
    fx_t dist_north = waypoint_distance(&pos, &wp_north);
    printf("\nTest 2b: 10m north\n");
    printf("  Distance = %.2f m %s\n", fx_to_float(dist_north),
           (fx_to_float(dist_north) > 9.9f && fx_to_float(dist_north) < 10.1f) ? "✓" : "✗");
    
    /* Test 2c: Diagonal (3-4-5 triangle) */
    waypoint_t wp_diag = make_waypoint(3.0f, 4.0f, -10.0f, 2.0f);
    fx_t dist_diag = waypoint_distance(&pos, &wp_diag);
    printf("\nTest 2c: 3-4-5 triangle\n");
    printf("  Distance = %.2f m (expected 5.0) %s\n", fx_to_float(dist_diag),
           (fx_to_float(dist_diag) > 4.9f && fx_to_float(dist_diag) < 5.1f) ? "✓" : "✗");
    
    /* Test 2d: 3D distance (with altitude change) */
    waypoint_t wp_3d = make_waypoint(3.0f, 4.0f, -22.0f, 2.0f);  /* 12m down */
    fx_t dist_3d = waypoint_distance(&pos, &wp_3d);
    /* sqrt(3² + 4² + 12²) = sqrt(9 + 16 + 144) = sqrt(169) = 13 */
    printf("\nTest 2d: 3D distance (3,4,12)\n");
    printf("  Distance = %.2f m (expected 13.0) %s\n", fx_to_float(dist_3d),
           (fx_to_float(dist_3d) > 12.9f && fx_to_float(dist_3d) < 13.1f) ? "✓" : "✗");
    
    /* Test 2e: Horizontal distance only */
    fx_t dist_horiz = waypoint_distance_horizontal(&pos, &wp_3d);
    printf("\nTest 2e: Horizontal distance (ignoring altitude)\n");
    printf("  Distance = %.2f m (expected 5.0) %s\n\n", fx_to_float(dist_horiz),
           (fx_to_float(dist_horiz) > 4.9f && fx_to_float(dist_horiz) < 5.1f) ? "✓" : "✗");
}

/* ============================================================================
 * Test 3: Single Waypoint Sequencing
 * ========================================================================= */
void test_single_waypoint(void) {
    printf("=== Test 3: Single Waypoint Mission ===\n\n");
    
    mission_t mission;
    waypoint_t waypoints[1] = {
        make_waypoint(10.0f, 0.0f, -10.0f, 2.0f)
    };
    mission_load(&mission, waypoints, 1, false);
    
    position_setpoint_t setpoint = {0};
    flight_mode_t mode_request;
    
    /* Start far from waypoint */
    nav_state_3d_t pos = make_nav_state(0.0f, 0.0f, -10.0f);
    
    printf("Step 1: Start at origin, target at N=10m\n");
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    print_position("  Setpoint", setpoint.north, setpoint.east, setpoint.down);
    printf("  current_index = %d %s\n", mission.current_index,
           (mission.current_index == 0) ? "✓" : "✗");
    printf("  mode_request = %s %s\n", flight_mode_name(mode_request),
           (mode_request == MODE_MISSION) ? "✓" : "✗");
    
    /* Move closer but not within acceptance */
    pos = make_nav_state(7.0f, 0.0f, -10.0f);
    printf("\nStep 2: Move to N=7m (3m from target, outside 2m radius)\n");
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("  current_index = %d %s\n", mission.current_index,
           (mission.current_index == 0) ? "✓" : "✗");
    
    /* Reach waypoint */
    pos = make_nav_state(10.0f, 0.0f, -10.0f);
    printf("\nStep 3: Reach waypoint (N=10m)\n");
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("  current_index = %d (advanced) %s\n", mission.current_index,
           (mission.current_index == 1) ? "✓" : "✗");
    printf("  is_complete() = %s %s\n", mission_is_complete(&mission) ? "true" : "false",
           mission_is_complete(&mission) ? "✓" : "✗");
    printf("  mode_request = %s (should request RTL) %s\n\n",
           flight_mode_name(mode_request),
           (mode_request == MODE_RTL) ? "✓" : "✗");
}

/* ============================================================================
 * Test 4: Multi-Waypoint Square Pattern
 * ========================================================================= */
void test_square_pattern(void) {
    printf("=== Test 4: Square Pattern Mission ===\n\n");
    
    mission_t mission;
    waypoint_t waypoints[4] = {
        make_waypoint(0.0f, 0.0f, -10.0f, 2.0f),   /* Home */
        make_waypoint(10.0f, 0.0f, -10.0f, 2.0f),  /* North */
        make_waypoint(10.0f, 10.0f, -10.0f, 2.0f), /* East */
        make_waypoint(0.0f, 10.0f, -10.0f, 2.0f)   /* West back */
    };
    mission_load(&mission, waypoints, 4, false);
    
    position_setpoint_t setpoint = {0};
    flight_mode_t mode_request;
    
    printf("Mission: Home → North 10m → East 10m → West back\n\n");

    /* Waypoint 0: Home */
    nav_state_3d_t pos = make_nav_state(0.0f, 0.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("WP0 (Home): Reached immediately\n");
    printf("  current_index = %d %s\n", mission.current_index,
           (mission.current_index == 1) ? "✓" : "✗");
    
    /* Move towards WP1 */
    pos = make_nav_state(10.0f, 0.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("\nWP1 (North): Reached N=10m\n");
    printf("  current_index = %d %s\n", mission.current_index,
           (mission.current_index == 2) ? "✓" : "✗");
    print_position("  Next setpoint", setpoint.north, setpoint.east, setpoint.down);
    
    /* Move towards WP2 */
    pos = make_nav_state(10.0f, 10.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("\nWP2 (East): Reached E=10m\n");
    printf("  current_index = %d %s\n", mission.current_index,
           (mission.current_index == 3) ? "✓" : "✗");
    print_position("  Next setpoint", setpoint.north, setpoint.east, setpoint.down);
    
    /* Reach final waypoint */
    pos = make_nav_state(0.0f, 10.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("\nWP3 (Final): Reached\n");
    printf("  is_complete() = %s %s\n", mission_is_complete(&mission) ? "true" : "false",
           mission_is_complete(&mission) ? "✓" : "✗");
    printf("  mode_request = %s %s\n\n", flight_mode_name(mode_request),
           (mode_request == MODE_RTL) ? "✓" : "✗");
}

/* ============================================================================
 * Test 5: Looping Mission
 * ========================================================================= */
void test_looping_mission(void) {
    printf("=== Test 5: Looping Mission ===\n\n");
    
    mission_t mission;
    waypoint_t waypoints[3] = {
        make_waypoint(0.0f, 0.0f, -10.0f, 2.0f),
        make_waypoint(5.0f, 0.0f, -10.0f, 2.0f),
        make_waypoint(5.0f, 5.0f, -10.0f, 2.0f)
    };
    mission_load(&mission, waypoints, 3, true);  /* Loop enabled */
    
    position_setpoint_t setpoint = {0};
    flight_mode_t mode_request;
    
    printf("Mission: 3 waypoints with loop=true\n\n");

    /* Complete all waypoints */
    nav_state_3d_t pos = make_nav_state(0.0f, 0.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("Reached WP0, index=%d\n", mission.current_index);
    
    pos = make_nav_state(5.0f, 0.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("Reached WP1, index=%d\n", mission.current_index);
    
    pos = make_nav_state(5.0f, 5.0f, -10.0f);
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("Reached WP2, index=%d\n", mission.current_index);
    
    printf("\nAfter completing all waypoints:\n");
    printf("  current_index = %d (should loop to 0) %s\n", mission.current_index,
           (mission.current_index == 0) ? "✓" : "✗");
    printf("  mode_request = %s (should stay MISSION) %s\n",
           flight_mode_name(mode_request),
           (mode_request == MODE_MISSION) ? "✓" : "✗");
    print_position("  Setpoint", setpoint.north, setpoint.east, setpoint.down);
    printf("  (should be WP0 again) %s\n\n",
           (fx_to_float(setpoint.north) == 0.0f) ? "✓" : "✗");
}

/* ============================================================================
 * Test 6: RTL Mode
 * ========================================================================= */
void test_rtl_mode(void) {
    printf("=== Test 6: Return-to-Launch Mode ===\n\n");
    
    mission_t mission;
    waypoint_t waypoints[3] = {
        make_waypoint(0.0f, 0.0f, -5.0f, 2.0f),   /* Home at 5m altitude */
        make_waypoint(20.0f, 20.0f, -10.0f, 2.0f),
        make_waypoint(40.0f, 0.0f, -10.0f, 2.0f)
    };
    mission_load(&mission, waypoints, 3, false);
    
    position_setpoint_t setpoint = {0};
    flight_mode_t mode_request;
    
    /* Vehicle is far from home */
    nav_state_3d_t pos = make_nav_state(30.0f, 30.0f, -15.0f);
    
    printf("Vehicle at N=30m E=30m D=-15m\n");
    printf("Home position at N=0m E=0m D=-5m\n\n");
    
    navigation_step(&pos, MODE_RTL, &mission, &setpoint, &mode_request);
    
    printf("RTL setpoint:\n");
    print_position("  Target", setpoint.north, setpoint.east, setpoint.down);
    printf("  Points to home %s\n", 
           (fx_to_float(setpoint.north) == 0.0f && 
            fx_to_float(setpoint.east) == 0.0f &&
            fx_to_float(setpoint.down) == -5.0f) ? "✓" : "✗");
    printf("  mode_request = %s %s\n\n", flight_mode_name(mode_request),
           (mode_request == MODE_RTL) ? "✓" : "✗");
}

/* ============================================================================
 * Test 7: Empty Mission Handling
 * ========================================================================= */
void test_empty_mission(void) {
    printf("=== Test 7: Empty Mission Error Handling ===\n\n");
    
    mission_t mission;
    mission_clear(&mission);
    
    position_setpoint_t setpoint = {0};
    flight_mode_t mode_request;
    nav_state_3d_t pos = make_nav_state(0.0f, 0.0f, -10.0f);
    
    printf("Enter MISSION mode with no mission loaded\n");
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    
    printf("  mode_request = %s (should request RTL) %s\n\n",
           flight_mode_name(mode_request),
           (mode_request == MODE_RTL) ? "✓" : "✗");
}

/* ============================================================================
 * Test 8: Variable Acceptance Radius
 * ========================================================================= */
void test_acceptance_radius(void) {
    printf("=== Test 8: Variable Acceptance Radius ===\n\n");
    
    mission_t mission;
    waypoint_t waypoints[2] = {
        make_waypoint(10.0f, 0.0f, -10.0f, 5.0f),   /* Large radius */
        make_waypoint(20.0f, 0.0f, -10.0f, 0.5f)    /* Tight radius */
    };
    mission_load(&mission, waypoints, 2, false);
    
    position_setpoint_t setpoint = {0};
    flight_mode_t mode_request;

    /* Test WP0 with 5m radius */
    nav_state_3d_t pos = make_nav_state(6.0f, 0.0f, -10.0f);  /* 4m from WP0 */
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("Test 8a: 4m from WP0 (5m acceptance radius)\n");
    printf("  Waypoint reached %s\n", (mission.current_index == 1) ? "✓" : "✗");
    
    /* Test WP1 with 0.5m radius - outside */
    pos = make_nav_state(19.3f, 0.0f, -10.0f);  /* 0.7m from WP1 */
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("\nTest 8b: 0.7m from WP1 (0.5m acceptance radius)\n");
    printf("  Waypoint NOT reached yet %s\n", (mission.current_index == 1) ? "✓" : "✗");
    
    /* Get closer */
    pos = make_nav_state(19.8f, 0.0f, -10.0f);  /* 0.2m from WP1 */
    navigation_step(&pos, MODE_MISSION, &mission, &setpoint, &mode_request);
    printf("\nTest 8c: 0.2m from WP1 (within 0.5m radius)\n");
    printf("  Waypoint reached %s\n", (mission.current_index == 2) ? "✓" : "✗");
    printf("  Mission complete, requests RTL %s\n\n",
           (mode_request == MODE_RTL) ? "✓" : "✗");
}

/* ============================================================================
 * Main Test Runner
 * ========================================================================= */
int main(void) {
    printf("==============================================================\n");
    printf(" MISSION SEQUENCER TEST SUITE - Section 10.3\n");
    printf("==============================================================\n\n");
    
    test_mission_loading();
    test_distance_calculations();
    test_single_waypoint();
    test_square_pattern();
    test_looping_mission();
    test_rtl_mode();
    test_empty_mission();
    test_acceptance_radius();
    
    printf("==============================================================\n");
    printf(" ALL TESTS COMPLETE\n");
    printf("==============================================================\n");
    
    return 0;
}
