#include "fixed_point.h"

/**
 * FIXED_POINT.C
 *
 * Section 2.4: CORDIC Implementation
 * Provides arctangent (vectoring mode) and sine/cosine (rotation mode).
 */

/* ============================================================================
 * CORDIC Arctangent Lookup Table
 * ========================================================================= */

/**
 * Precomputed CORDIC arctangent lookup table in Q16.16 format.
 * cordic_atan_table[i] = atan(2^(-i)) * 2^16
 * Per Listing 2.2 of the architecture document.
 */
static const fx_t cordic_atan_table[16] = {
    0x0000C910, /* atan(2^0)  = 0.785398 rad (45°) */
    0x000076B1, /* atan(2^-1) = 0.463647 rad */
    0x00003EB6, /* atan(2^-2) = 0.244978 rad */
    0x00001FD5, /* atan(2^-3) = 0.124355 rad */
    0x00000FFA, /* atan(2^-4) = 0.062418 rad */
    0x000007FF, /* atan(2^-5) = 0.031239 rad */
    0x00000400, /* atan(2^-6) = 0.015623 rad */
    0x00000200, /* atan(2^-7) = 0.007812 rad */
    0x00000100, /* atan(2^-8) = 0.003906 rad */
    0x00000080, /* atan(2^-9) = 0.001953 rad */
    0x00000040, /* atan(2^-10) = 0.000976 rad */
    0x00000020, /* atan(2^-11) = 0.000488 rad */
    0x00000010, /* atan(2^-12) = 0.000244 rad */
    0x00000008, /* atan(2^-13) = 0.000122 rad */
    0x00000004, /* atan(2^-14) = 0.000061 rad */
    0x00000002  /* atan(2^-15) = 0.000030 rad */
};

/**
 * CORDIC gain K product for 16 iterations.
 * K ≈ 0.607252935 in Q16.16 = 39797
 * This compensates for the vector magnitude growth during CORDIC rotation.
 */
#define CORDIC_K 39797

/* ============================================================================
 * SECTION 2.4: cordic_atan2 (Listing 2.2)
 * ========================================================================= */

void cordic_atan2(fx_t y, fx_t x, fx_t *angle_out) {
    fx_t angle = 0;
    fx_t x_new, y_new;

    /* Handle edge case: both x and y are zero */
    if (x == 0 && y == 0) {
        *angle_out = 0;
        return;
    }

    /* Handle quadrants by adjusting for x < 0 cases
     * CORDIC vectoring mode works best in right half-plane (x > 0) */
    int quadrant_adjust = 0;

    if (x < 0) {
        if (y >= 0) {
            quadrant_adjust = FX_PI;  /* Quadrant II */
        } else {
            quadrant_adjust = -FX_PI; /* Quadrant III */
        }
        x = -x;
        y = -y;
    }

    /* CORDIC vectoring mode: rotate vector (x, y) to align with x-axis
     * Final angle accumulation gives atan2(y, x) */
    for (int i = 0; i < 16; i++) {
        if (y > 0) {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            angle += cordic_atan_table[i];
        } else {
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            angle -= cordic_atan_table[i];
        }
        x = x_new;
        y = y_new;
    }

    /* Apply quadrant correction */
    angle += quadrant_adjust;

    /* Normalize to [-π, π] range */
    while (angle > FX_PI) angle -= FX_TWO_PI;
    while (angle < -FX_PI) angle += FX_TWO_PI;

    *angle_out = angle;
}

/* ============================================================================
 * SECTION 2.4: cordic_sincos (Rotation Mode)
 * ========================================================================= */

void cordic_sincos(fx_t angle, fx_t *sin_out, fx_t *cos_out) {
    fx_t x = CORDIC_K;  /* Start with K to compensate for gain */
    fx_t y = 0;
    fx_t current_angle = 0;
    fx_t x_new, y_new;

    /* Normalize angle to [-π, π] */
    while (angle > FX_PI) angle -= FX_TWO_PI;
    while (angle < -FX_PI) angle += FX_TWO_PI;

    /* CORDIC rotation mode works in [-π/2, π/2]
     * For angles outside this range, use trigonometric identities:
     * sin(π - θ) = sin(θ), cos(π - θ) = -cos(θ)
     * sin(-π - θ) = -sin(θ), cos(-π - θ) = -cos(θ) */
    int negate_cos = 0;

    if (angle > FX_HALF_PI) {
        /* Quadrant II: reduce to quadrant I, then negate cosine */
        angle = FX_PI - angle;
        negate_cos = 1;
    } else if (angle < -FX_HALF_PI) {
        /* Quadrant III: reduce to quadrant IV, then negate cosine */
        angle = -FX_PI - angle;
        negate_cos = 1;
    }

    /* CORDIC rotation mode: rotate vector (x, y) by angle
     * Final (x, y) gives (cos(angle), sin(angle)) scaled by K
     * Since we initialized x = K, the scaling cancels out */
    for (int i = 0; i < 16; i++) {
        if (angle >= current_angle) {
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            current_angle += cordic_atan_table[i];
        } else {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            current_angle -= cordic_atan_table[i];
        }
        x = x_new;
        y = y_new;
    }

    /* Apply quadrant correction */
    if (negate_cos) {
        *cos_out = -x;
    } else {
        *cos_out = x;
    }
    *sin_out = y;
}
