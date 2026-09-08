#include "control.h"

/**
 * CONTROL.C
 *
 * Section 7: PID Controller Implementation
 * Section 8: Motor Mixing Implementation
 */

/* ============================================================================
 * SECTION 7.4: PID Controller (Listing 7.1)
 * ========================================================================= */

fx_t pid_update(pid_t *pid, fx_t setpoint, fx_t measured, fx_t dt) {
    /* Step 1: Calculate error */
    fx_t error = fx_sub(setpoint, measured);

    /* Step 2: Proportional term */
    fx_t p_term = fx_mul(pid->Kp, error);

    /* Step 3: Integral term with anti-windup clamping (Section 7.3) */
    pid->integral = fx_add(pid->integral, fx_mul(error, dt));

    /* Clamp integral to prevent windup */
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    if (pid->integral < fx_neg(pid->integral_limit)) {
        pid->integral = fx_neg(pid->integral_limit);
    }

    fx_t i_term = fx_mul(pid->Ki, pid->integral);

    /* Step 4: Derivative term (rate of change of error) */
    fx_t derivative = fx_div(fx_sub(error, pid->prev_error), dt);
    fx_t d_term = fx_mul(pid->Kd, derivative);

    /* Update previous error for next iteration */
    pid->prev_error = error;

    /* Step 5: Sum PID terms */
    fx_t output = fx_add(fx_add(p_term, i_term), d_term);

    /* Step 6: Output saturation (Section 8.3) */
    output = fx_clamp(output, pid->out_min, pid->out_max);

    return output;
}

/* ============================================================================
 * SECTION 8.3: Motor Mixing (Listing 8.1)
 * ========================================================================= */

void mix_and_output_motors(fx_t throttle, fx_t roll, fx_t pitch, fx_t yaw, fx_t *motor_out) {
    /* Mixing equations for X configuration (Section 8.2):
     *
     *   M1 (FL) = T + P + R - Y   (Front-Left, CW)
     *   M2 (FR) = T + P - R + Y   (Front-Right, CCW)
     *   M3 (RR) = T - P - R - Y   (Rear-Right, CW)
     *   M4 (RL) = T - P + R + Y   (Rear-Left, CCW)
     */

    /* Motor 1: Front-Left */
    motor_out[MOTOR_FL] = fx_add(fx_add(throttle, pitch), fx_sub(roll, yaw));

    /* Motor 2: Front-Right */
    motor_out[MOTOR_FR] = fx_add(fx_add(throttle, pitch), fx_sub(yaw, roll));

    /* Motor 3: Rear-Right */
    motor_out[MOTOR_RR] = fx_sub(fx_sub(throttle, pitch), fx_add(roll, yaw));

    /* Motor 4: Rear-Left */
    motor_out[MOTOR_RL] = fx_add(fx_sub(throttle, pitch), fx_add(roll, yaw));

    /* Saturate all motor outputs to valid PWM range (Section 8.3) */
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_out[i] = fx_clamp(motor_out[i], PWM_MIN, PWM_MAX);
    }
}
