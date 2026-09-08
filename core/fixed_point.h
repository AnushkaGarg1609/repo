#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * FIXED_POINT_H
 *
 * Section 2: Numerical Representation Without Hardware Floating-Point
 * Provides Q16.16 fixed-point arithmetic primitives and CORDIC transcendentals.
 */

/* ============================================================================
 * SECTION 2.2: Q-Format Representation
 * ========================================================================= */

/**
 * Fixed-point type: signed 32-bit integer representing Q16.16 format.
 * Value = (fx_t >> 16) + (fx_t & 0xFFFF) / 65536.0
 */
typedef int32_t fx_t;

/** Number of fractional bits (binary point position) */
#define FX_SHIFT 16

/** Fixed-point representation of 1.0 */
#define FX_ONE (1 << FX_SHIFT)

/** Fixed-point representation of 0.0 */
#define FX_ZERO 0

/** Maximum representable value: ~32767.99998 */
#define FX_MAX ((fx_t)0x7FFFFFFF)

/** Minimum representable value: ~-32768.0 */
#define FX_MIN ((fx_t)0x80000000)

/* Common constants in Q16.16 */
#define FX_PI      205887  /* 3.14159265... * 2^16 */
#define FX_HALF_PI 102943  /* 1.57079632... * 2^16 */
#define FX_TWO_PI  411774  /* 6.28318530... * 2^16 */

/* ============================================================================
 * SECTION 2.3: Core Arithmetic Operations
 * ========================================================================= */

/**
 * Convert integer to fixed-point.
 * @param i Integer value
 * @return Fixed-point representation
 */
static inline fx_t fx_from_int(int32_t i) {
    return i << FX_SHIFT;
}

/**
 * Convert fixed-point to integer (truncates fractional part).
 * @param a Fixed-point value
 * @return Integer part only
 */
static inline int32_t fx_to_int(fx_t a) {
    return a >> FX_SHIFT;
}

/**
 * Convert float to fixed-point (for test harness only; not used on target).
 * @param f Floating-point value
 * @return Fixed-point representation
 */
static inline fx_t fx_from_float(float f) {
    return (fx_t)(f * (float)FX_ONE);
}

/**
 * Convert fixed-point to float (for test harness only; not used on target).
 * @param a Fixed-point value
 * @return Floating-point approximation
 */
static inline float fx_to_float(fx_t a) {
    return (float)a / (float)FX_ONE;
}

/**
 * Fixed-point addition.
 * No overflow protection — caller must ensure operands are in valid range.
 * @param a First operand (Q16.16)
 * @param b Second operand (Q16.16)
 * @return a + b (Q16.16)
 */
static inline fx_t fx_add(fx_t a, fx_t b) {
    return a + b;
}

/**
 * Fixed-point subtraction.
 * No overflow protection — caller must ensure operands are in valid range.
 * @param a Minuend (Q16.16)
 * @param b Subtrahend (Q16.16)
 * @return a - b (Q16.16)
 */
static inline fx_t fx_sub(fx_t a, fx_t b) {
    return a - b;
}

/**
 * Fixed-point multiplication.
 * Uses 64-bit intermediate to preserve precision, per Listing 2.1.
 * @param a First factor (Q16.16)
 * @param b Second factor (Q16.16)
 * @return a * b (Q16.16)
 */
static inline fx_t fx_mul(fx_t a, fx_t b) {
    int64_t product = (int64_t)a * (int64_t)b;
    return (fx_t)(product >> FX_SHIFT);
}

/**
 * Fixed-point division.
 * Uses 64-bit intermediate to preserve precision, per Listing 2.1.
 * @param a Numerator (Q16.16)
 * @param b Denominator (Q16.16), must not be zero
 * @return a / b (Q16.16)
 * @warning Returns FX_MAX or FX_MIN on divide-by-zero (no exception thrown)
 */
static inline fx_t fx_div(fx_t a, fx_t b) {
    if (b == 0) {
        return (a >= 0) ? FX_MAX : FX_MIN; /* Saturate on divide-by-zero */
    }
    int64_t numerator = (int64_t)a << FX_SHIFT;
    return (fx_t)(numerator / b);
}

/* ============================================================================
 * SECTION 7.3 & 8.3: Saturation and Clamping
 * ========================================================================= */

/**
 * Clamp fixed-point value to a range.
 * Used for integral windup mitigation (Sec 7.3) and mixer output (Sec 8.3).
 * @param val Value to clamp
 * @param min Minimum bound (inclusive)
 * @param max Maximum bound (inclusive)
 * @return Clamped value
 */
static inline fx_t fx_clamp(fx_t val, fx_t min, fx_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * Return the absolute value of a fixed-point number.
 * @param a Input value (Q16.16)
 * @return |a| (Q16.16)
 */
static inline fx_t fx_abs(fx_t a) {
    return (a < 0) ? -a : a;
}

/**
 * Negate a fixed-point number.
 * @param a Input value (Q16.16)
 * @return -a (Q16.16)
 */
static inline fx_t fx_neg(fx_t a) {
    return -a;
}

/* ============================================================================
 * SECTION 2.4 & 6.3: Square Root (Newton-Raphson Method)
 * ========================================================================= */

/**
 * Fixed-point square root using Newton-Raphson iteration.
 * Required for distance calculations (Listing 10.2) and vector norms (Sec 6.3).
 *
 * @param a Input value (Q16.16), must be non-negative
 * @return sqrt(a) (Q16.16)
 * @warning Returns 0 if input is negative
 */
static inline fx_t fx_sqrt(fx_t a) {
    if (a <= 0) return 0;
    if (a == FX_ONE) return FX_ONE;

    /* Initial guess: use the input value shifted right */
    fx_t x = a >> 1;
    if (x == 0) x = FX_ONE;

    /* Newton-Raphson: x_next = (x + a/x) / 2
     * Iterate 8 times — enough for Q16.16 convergence */
    for (int i = 0; i < 8; i++) {
        fx_t x_next = fx_add(x, fx_div(a, x)) >> 1; /* Divide by 2 via shift */
        if (fx_abs(fx_sub(x_next, x)) < 2) break; /* Converged to within LSB */
        x = x_next;
    }

    return x;
}

/* ============================================================================
 * SECTION 2.4: Comparison Helpers
 * ========================================================================= */

/**
 * Return the minimum of two fixed-point values.
 * @param a First value (Q16.16)
 * @param b Second value (Q16.16)
 * @return min(a, b) (Q16.16)
 */
static inline fx_t fx_min(fx_t a, fx_t b) {
    return (a < b) ? a : b;
}

/**
 * Return the maximum of two fixed-point values.
 * @param a First value (Q16.16)
 * @param b Second value (Q16.16)
 * @return max(a, b) (Q16.16)
 */
static inline fx_t fx_max(fx_t a, fx_t b) {
    return (a > b) ? a : b;
}

/**
 * Test equality within a tolerance (useful for unit tests).
 * @param a First value (Q16.16)
 * @param b Second value (Q16.16)
 * @param epsilon Tolerance (Q16.16), typically a few LSBs
 * @return true if |a - b| <= epsilon
 */
static inline bool fx_equal_epsilon(fx_t a, fx_t b, fx_t epsilon) {
    return fx_abs(fx_sub(a, b)) <= epsilon;
}

/* ============================================================================
 * SECTION 2.4: Approximating Transcendental Functions (CORDIC)
 * ========================================================================= */

/**
 * cordic_atan2: Computes the arctangent of y/x using CORDIC vectoring mode.
 * Returns angle in radians, represented in Q16.16.
 * @param y Y component (Q16.16)
 * @param x X component (Q16.16)
 * @param angle_out Output angle in radians (Q16.16), range [-π, π]
 */
void cordic_atan2(fx_t y, fx_t x, fx_t *angle_out);

/**
 * cordic_sincos: Computes sine and cosine of angle using CORDIC rotation mode.
 * Angle input is in radians, represented in Q16.16.
 * Outputs sin and cos in Q16.16 mapped values [-1.0, 1.0].
 * @param angle Input angle in radians (Q16.16)
 * @param sin_out Output sine value (Q16.16), range [-1.0, 1.0]
 * @param cos_out Output cosine value (Q16.16), range [-1.0, 1.0]
 */
void cordic_sincos(fx_t angle, fx_t *sin_out, fx_t *cos_out);

#endif /* FIXED_POINT_H */
