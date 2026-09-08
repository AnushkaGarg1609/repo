#include "mission.h"
#include <string.h>

/**
 * MISSION.C
 *
 * Section 10.3: Mission Sequencer Implementation (Listing 10.2)
 */

/* ============================================================================
 * SECTION 10.3: Mission Management
 * ========================================================================= */

bool mission_load(mission_t *mission, const waypoint_t *waypoints, uint8_t count, bool loop) {
    /* Validate count */
    if (count > MAX_MISSION_WAYPOINTS) {
        return false;
    }

    /* Copy waypoints */
    memcpy(mission->waypoints, waypoints, count * sizeof(waypoint_t));
    mission->count = count;
    mission->current_index = 0;
    mission->loop = loop;

    return true;
}

void mission_clear(mission_t *mission) {
    mission->count = 0;
    mission->current_index = 0;
    mission->loop = false;
}

/* ============================================================================
 * SECTION 10.3: Distance Calculations
 * ========================================================================= */

fx_t waypoint_distance(const nav_state_3d_t *nav_est, const waypoint_t *wp) {
    /* Calculate deltas */
    fx_t dn = fx_sub(wp->north, nav_est->north);
    fx_t de = fx_sub(wp->east, nav_est->east);
    fx_t dd = fx_sub(wp->down, nav_est->down);

    /* Distance² = ΔN² + ΔE² + ΔD² */
    fx_t dist_sq = fx_add(
        fx_add(fx_mul(dn, dn), fx_mul(de, de)),
        fx_mul(dd, dd)
    );

    /* Return sqrt(distance²) */
    return fx_sqrt(dist_sq);
}

fx_t waypoint_distance_horizontal(const nav_state_3d_t *nav_est, const waypoint_t *wp) {
    /* Calculate horizontal deltas only */
    fx_t dn = fx_sub(wp->north, nav_est->north);
    fx_t de = fx_sub(wp->east, nav_est->east);

    /* Horizontal distance² = ΔN² + ΔE² */
    fx_t dist_sq = fx_add(fx_mul(dn, dn), fx_mul(de, de));

    /* Return sqrt(distance²) */
    return fx_sqrt(dist_sq);
}

/* ============================================================================
 * SECTION 10.3: Navigation Step (Listing 10.2)
 * ========================================================================= */

void navigation_step(
    const nav_state_3d_t *nav_est,
    flight_mode_t mode,
    mission_t *mission,
    position_setpoint_t *out_setpoint,
    flight_mode_t *out_mode_request
) {
    /* Default: no mode change requested */
    *out_mode_request = mode;

    /* Handle mode-specific behavior */
    switch (mode) {
        case MODE_MISSION:
            /* Check if mission is loaded */
            if (!mission_is_loaded(mission)) {
                /* No mission - request RTL immediately */
                *out_mode_request = MODE_RTL;
                return;
            }

            /* Check if mission is complete */
            if (mission_is_complete(mission)) {
                /* All waypoints reached */
                if (mission->loop) {
                    /* Loop mode: restart from beginning */
                    mission->current_index = 0;
                } else {
                    /* Non-loop mode: request RTL */
                    *out_mode_request = MODE_RTL;
                    return;
                }
            }

            /* Get current target waypoint */
            waypoint_t *current_wp = &mission->waypoints[mission->current_index];

            /* Calculate distance to current waypoint */
            fx_t distance = waypoint_distance(nav_est, current_wp);

            /* Get acceptance radius (use waypoint-specific or default) */
            fx_t acceptance_radius = current_wp->acceptance_radius;
            if (acceptance_radius == FX_ZERO || acceptance_radius < MIN_ACCEPTANCE_RADIUS) {
                acceptance_radius = DEFAULT_ACCEPTANCE_RADIUS;
            }

            /* Check if waypoint reached */
            if (distance <= acceptance_radius) {
                /* Advance to next waypoint */
                mission->current_index++;

                /* Check if mission complete after advancing */
                if (mission_is_complete(mission)) {
                    if (mission->loop) {
                        mission->current_index = 0;
                        current_wp = &mission->waypoints[0];
                    } else {
                        /* Mission complete - request RTL */
                        *out_mode_request = MODE_RTL;
                        return;
                    }
                } else {
                    /* Update current waypoint pointer */
                    current_wp = &mission->waypoints[mission->current_index];
                }
            }

            /* Output current waypoint as setpoint */
            out_setpoint->north = current_wp->north;
            out_setpoint->east = current_wp->east;
            out_setpoint->down = current_wp->down;
            out_setpoint->yaw = FX_ZERO;  /* Point towards waypoint (optional heading) */
            break;

        case MODE_RTL:
            /* Return to launch: set home position as setpoint (waypoint 0) */
            if (mission_is_loaded(mission) && mission->count > 0) {
                out_setpoint->north = mission->waypoints[0].north;
                out_setpoint->east = mission->waypoints[0].east;
                out_setpoint->down = mission->waypoints[0].down;
                out_setpoint->yaw = FX_ZERO;
            } else {
                /* No home position stored - hold current position */
                out_setpoint->north = nav_est->north;
                out_setpoint->east = nav_est->east;
                out_setpoint->down = nav_est->down;
                out_setpoint->yaw = FX_ZERO;
            }
            break;

        case MODE_POS_HOLD:
        case MODE_ALT_HOLD:
        case MODE_STABILIZE:
        case MODE_DISARMED:
        default:
            /* No autonomous navigation in these modes - setpoint unchanged */
            break;
    }
}
