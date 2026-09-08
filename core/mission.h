#ifndef MISSION_H
#define MISSION_H

#include <stdbool.h>
#include <stdint.h>
#include "fixed_point.h"
#include "flight_modes.h"
#include "sensor_fusion.h"

/**
 * MISSION.H
 *
 * Section 10.3: Mission Sequencer
 * Implements waypoint navigation for autonomous flight (Listing 10.2)
 */

/* ============================================================================
 * SECTION 10.3: Mission Waypoint Structure
 * ========================================================================= */

/**
 * Waypoint definition in local NED coordinates.
 * Each waypoint represents a target position the vehicle should reach.
 */
typedef struct {
    fx_t north;           /* North position (meters, Q16.16) */
    fx_t east;            /* East position (meters, Q16.16) */
    fx_t down;            /* Down position (meters, Q16.16, negative = altitude) */
    fx_t acceptance_radius;  /* Distance threshold to consider waypoint reached (m) */
} waypoint_t;

/**
 * Maximum number of waypoints in a mission.
 * Constrained by available SRAM (256 KB target).
 */
#define MAX_MISSION_WAYPOINTS 64

/**
 * Mission definition structure.
 * Contains the complete waypoint list and metadata.
 */
typedef struct {
    waypoint_t waypoints[MAX_MISSION_WAYPOINTS];
    uint8_t count;              /* Number of valid waypoints (0 = no mission) */
    uint8_t current_index;      /* Index of the current target waypoint */
    bool loop;                  /* If true, restart from waypoint 0 after completing */
} mission_t;

/* ============================================================================
 * SECTION 10.3: Position Setpoint Structure
 * ========================================================================= */

/**
 * Position setpoint output from the mission sequencer.
 * This is the target the position controller should track.
 */
typedef struct {
    fx_t north;      /* Target north position (m, Q16.16) */
    fx_t east;       /* Target east position (m, Q16.16) */
    fx_t down;       /* Target down position (m, Q16.16) */
    fx_t yaw;        /* Target yaw angle (rad, Q16.16) - optional */
} position_setpoint_t;

/**
 * Full 3D navigation state for mission sequencer.
 * Extends nav_state_t (horizontal only) with altitude.
 * Note: sensor_fusion.h defines nav_state_t as horizontal (N,E) + velocity.
 */
typedef struct {
    fx_t north;      /* Current north position (m, Q16.16) */
    fx_t east;       /* Current east position (m, Q16.16) */
    fx_t down;       /* Current down position (m, Q16.16) */
} nav_state_3d_t;

/* ============================================================================
 * SECTION 10.3: Waypoint Acceptance Thresholds
 * ========================================================================= */

/**
 * Default waypoint acceptance radius (meters).
 * When vehicle is within this distance of the target waypoint, consider it reached.
 */
#define DEFAULT_ACCEPTANCE_RADIUS fx_from_float(2.0f)  /* 2 meters */

/**
 * Minimum safe acceptance radius (meters).
 * Prevents overly tight tolerances that may cause hunting.
 */
#define MIN_ACCEPTANCE_RADIUS fx_from_float(0.5f)  /* 0.5 meters */

/* ============================================================================
 * SECTION 10.3: Mission Sequencer Functions (Listing 10.2)
 * ========================================================================= */

/**
 * Initialize a new mission with waypoints.
 * Copies waypoints into the mission structure and resets progress.
 *
 * @param mission Mission structure to initialize
 * @param waypoints Array of waypoints to load
 * @param count Number of waypoints (must be <= MAX_MISSION_WAYPOINTS)
 * @param loop If true, mission will restart from beginning after completion
 * @return true if successful, false if count exceeds maximum
 */
bool mission_load(mission_t *mission, const waypoint_t *waypoints, uint8_t count, bool loop);

/**
 * Clear the current mission.
 * Resets the mission structure to empty state.
 *
 * @param mission Mission structure to clear
 */
void mission_clear(mission_t *mission);

/**
 * Check if a mission is currently loaded and valid.
 *
 * @param mission Mission structure to check
 * @return true if mission has at least one waypoint
 */
static inline bool mission_is_loaded(const mission_t *mission) {
    return mission->count > 0;
}

/**
 * Check if the mission is complete (all waypoints reached).
 *
 * @param mission Mission structure to check
 * @return true if current_index has advanced past the last waypoint
 */
static inline bool mission_is_complete(const mission_t *mission) {
    return mission->current_index >= mission->count;
}

/**
 * Navigation step function (Section 10.3, Listing 10.2).
 * Called every control cycle to update mission progress and generate setpoints.
 *
 * Behavior by mode:
 * - MODE_MISSION: Track current waypoint, advance on acceptance, request RTL when complete
 * - MODE_RTL: Set home position as setpoint (waypoint 0)
 * - Other modes: No-op (setpoint unchanged)
 *
 * @param nav_est Current 3D navigation state estimate (vehicle position)
 * @param mode Current flight mode
 * @param mission Mission structure (modified: advances current_index when waypoint reached)
 * @param out_setpoint Output position setpoint for the position controller
 * @param out_mode_request Output mode request (e.g., RTL when mission complete)
 */
void navigation_step(
    const nav_state_3d_t *nav_est,
    flight_mode_t mode,
    mission_t *mission,
    position_setpoint_t *out_setpoint,
    flight_mode_t *out_mode_request
);

/**
 * Calculate 3D distance between current position and a waypoint.
 * Uses Euclidean distance: sqrt((N-n)² + (E-e)² + (D-d)²)
 *
 * @param nav_est Current 3D navigation state
 * @param wp Target waypoint
 * @return Distance in meters (Q16.16)
 */
fx_t waypoint_distance(const nav_state_3d_t *nav_est, const waypoint_t *wp);

/**
 * Calculate 2D horizontal distance to waypoint (ignoring altitude).
 * Uses: sqrt((N-n)² + (E-e)²)
 *
 * @param nav_est Current 3D navigation state
 * @param wp Target waypoint
 * @return Horizontal distance in meters (Q16.16)
 */
fx_t waypoint_distance_horizontal(const nav_state_3d_t *nav_est, const waypoint_t *wp);

#endif /* MISSION_H */
