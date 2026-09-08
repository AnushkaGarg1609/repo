#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "fixed_point.h"

#define M_PI 3.14159265358979323846

void test_basic_math(void) {
    printf("--- Running Basic Math Tests ---\n");
    fx_t a = fx_from_float(1.5f);
    fx_t b = fx_from_float(2.25f);
    fx_t add_res = fx_add(a, b);
    printf("Add: 1.5 + 2.25 = %.4f (Expected: 3.7500)\n", fx_to_float(add_res));
    fx_t sub_res = fx_sub(a, b);
    printf("Sub: 1.5 - 2.25 = %.4f (Expected: -0.7500)\n", fx_to_float(sub_res));
    fx_t mul_res = fx_mul(a, b);
    printf("Mul: 1.5 * 2.25 = %.4f (Expected: 3.3750)\n", fx_to_float(mul_res));
    fx_t div_res = fx_div(a, b);
    printf("Div: 1.5 / 2.25 = %.4f (Expected: 0.6667)\n", fx_to_float(div_res));
    fx_t hz = fx_div(a, 0);
    printf("Div by zero (pos): 0x%08X (Expected: 0x7FFFFFFF)\n", hz);
    fx_t hzn = fx_div(fx_neg(a), 0);
    printf("Div by zero (neg): 0x%08X (Expected: 0x80000000)\n", hzn);
    fx_t neg_a = fx_neg(a);
    printf("Neg: -1.5 = %.4f (Expected: -1.5000)\n", fx_to_float(neg_a));
    printf("Abs: |-1.5| = %.4f (Expected: 1.5000)\n", fx_to_float(fx_abs(neg_a)));
    fx_t low = fx_from_float(-1.0f);
    fx_t high = fx_from_float(1.0f);
    printf("Clamp (-2.5, -1, 1) = %.4f (Expected: -1.0000)\n", fx_to_float(fx_clamp(fx_from_float(-2.5f), low, high)));
    printf("Clamp (2.5, -1, 1) = %.4f (Expected: 1.0000)\n", fx_to_float(fx_clamp(fx_from_float(2.5f), low, high)));
    printf("Clamp (0.5, -1, 1) = %.4f (Expected: 0.5000)\n", fx_to_float(fx_clamp(fx_from_float(0.5f), low, high)));
    fx_t sqrt_four = fx_sqrt(fx_from_float(4.0f));
    printf("Sqrt: sqrt(4.0) = %.4f (Expected: 2.0000)\n", fx_to_float(sqrt_four));
    fx_t sqrt_pf = fx_sqrt(fx_from_float(0.5f));
    printf("Sqrt: sqrt(0.5) = %.4f (Expected: 0.7071)\n", fx_to_float(sqrt_pf));
}

void test_cordic_atan2(void) {
    printf("\n--- Running CORDIC atan2 Tests ---\n");
    struct { float y; float x; } cases[] = {
        {0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f}, {1.0f, -1.0f},
        {0.0f, -1.0f}, {-1.0f, -1.0f}, {-1.0f, 0.0f}, {-1.0f, 1.0f}, {0.0f, 0.0f}
    };
    int num_cases = sizeof(cases) / sizeof(cases[0]);
    for (int i = 0; i < num_cases; i++) {
        fx_t fx_y = fx_from_float(cases[i].y);
        fx_t fx_x = fx_from_float(cases[i].x);
        fx_t fx_angle;
        cordic_atan2(fx_y, fx_x, &fx_angle);
        float float_angle = fx_to_float(fx_angle);
        float ref_angle = atan2f(cases[i].y, cases[i].x);
        float error = fabsf(float_angle - ref_angle);
        printf("cordic_atan2(%.2f, %.2f) = %8.5f rad (%7.2f°) | Ref: %8.5f rad | Error: %.5f\n", 
               cases[i].y, cases[i].x, float_angle, float_angle * 180.0f / M_PI, ref_angle, error);
    }
}

void test_cordic_sincos(void) {
    printf("\n--- Running CORDIC sincos Tests ---\n");
    float angles[] = {
        0.0f, (float)M_PI / 6.0f, (float)M_PI / 4.0f, (float)M_PI / 3.0f,
        (float)M_PI / 2.0f, 2.0f * (float)M_PI / 3.0f, (float)M_PI,
        -(float)M_PI / 4.0f, -(float)M_PI / 2.0f, -3.0f * (float)M_PI / 4.0f, -(float)M_PI
    };
    int num_angles = sizeof(angles) / sizeof(angles[0]);
    for (int i = 0; i < num_angles; i++) {
        fx_t fx_angle = fx_from_float(angles[i]);
        fx_t fx_sin, fx_cos;
        cordic_sincos(fx_angle, &fx_sin, &fx_cos);
        float float_sin = fx_to_float(fx_sin);
        float float_cos = fx_to_float(fx_cos);
        float ref_sin = sinf(angles[i]);
        float ref_cos = cosf(angles[i]);
        float err_sin = fabsf(float_sin - ref_sin);
        float err_cos = fabsf(float_cos - ref_cos);
        printf("Angle: %6.1f° | Sin: %8.5f (Ref: %8.5f, Err: %.5f) | Cos: %8.5f (Ref: %8.5f, Err: %.5f)\n",
               angles[i] * 180.0f / M_PI, float_sin, ref_sin, err_sin, float_cos, ref_cos, err_cos);
    }
}

int main(void) {
    printf("=== Starting Flight Controller Fixed-Point Math Test Suite ===\n\n");
    test_basic_math();
    test_cordic_atan2();
    test_cordic_sincos();
    printf("\n=== Test Suite Complete ===\n");
    return 0;
}
