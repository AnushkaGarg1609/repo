#ifndef CONTROL_H
#define CONTROL_H

#include "fixed_point.h"

/**
 * CONTROL.H
 *
 * Section 7: Control Theory for Full-Envelope Flight
 * Section 8: Motor Mixing
 */

/* ============================================================================
 * SECTION 7.4: PID Controller (Listing 7.1)
 * ========================================================================= */

/**
 * PID controller state and gains.
 * Single implementation reused for all control loops with different gains.
 */
typedef struct {
    fx_t Kp;             /* Proportional gain */
    fx_t Ki;             /* Integral gain */
    fx_t Kd;             /* Derivative gain */

    fx_t integral;       /* Accumulated integral term */
    fx_t integral_limit; /* Max absolute value for integral (windup mitigation) */

    fx_t prev_error;     /* Previous error for derivative calculation */

    fx_t out_min;        /* Minimum output saturation */
    fx_t out_max;        /* Maximum output saturation */
} pid_t;

/**
 * Initialize a PID controller with gains and limits.
 *
 * @param pid PID controller to initialize
 * @param Kp Proportional gain (Q16.16)
 * @param Ki Integral gain (Q16.16)
 * @param Kd Derivative gain (Q16.16)
 * @param integral_limit Maximum integral accumulation (Q16.16)
 * @param out_min Minimum output (Q16.16)
 * @param out_max Maximum output (Q16.16)
 */
static inline void pid_init(pid_t *pid, fx_t Kp, fx_t Ki, fx_t Kd,
                            fx_t integral_limit, fx_t out_min, fx_t out_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->integral_limit = integral_limit;
    pid->prev_error = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

/**
 * Reset PID controller state (clear integral and derivative terms).
 * Used when switching flight modes or after disarming.
 *
 * @param pid PID controller to reset
 */
static inline void pid_reset(pid_t *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

/**
 * Update PID controller (Listing 7.1).
 * Computes output as: P*error + I*∫error + D*d(error)/dt
 *
 * @param pid PID controller state and gains
 * @param setpoint Commanded value (Q16.16)
 * @param measured Current measured value (Q16.16)
 * @param dt Time step (seconds, Q16.16)
 * @return Control output (Q16.16), clamped to [out_min, out_max]
 */
fx_t pid_update(pid_t *pid, fx_t setpoint, fx_t measured, fx_t dt);

/* ============================================================================
 * SECTION 8: Motor Mixing for Quadrotor X Configuration
 * ========================================================================= */

/**
 * Quadrotor motor indices (X configuration).
 *
 *     Front
 *   M1     M2
 *      \ /
 *       X
 *      / \
 *   M4     M3
 *     Rear
 *
 * M1: Front-Left  (CW)
 * M2: Front-Right (CCW)
 * M3: Rear-Right  (CW)
 * M4: Rear-Left   (CCW)
 */
#define MOTOR_FL 0  /* Front-Left */
#define MOTOR_FR 1  /* Front-Right */
#define MOTOR_RR 2  /* Rear-Right */
#define MOTOR_RL 3  /* Rear-Left */

/**
 * Number of motors
 */
#define NUM_MOTORS 4

/**
 * PWM output range constants (typical ESC range: 1000-2000 μs)
 */
#define PWM_MIN fx_from_int(1000)  /* Minimum throttle (motors off) */
#define PWM_MAX fx_from_int(2000)  /* Maximum throttle (full power) */

/**
 * Motor mixing for quadrotor X configuration (Listing 8.1).
 * Combines throttle, roll, pitch, yaw commands into individual motor outputs.
 *
 * Mixing equations (Section 8.2):
 *   M1 (FL) = T + P + R - Y
 *   M2 (FR) = T + P - R + Y
 *   M3 (RR) = T - P - R - Y
 *   M4 (RL) = T - P + R + Y
 *
 * Where:
 *   T = Throttle (baseline thrust)
 *   P = Pitch   (forward/backward tilt)
 *   R = Roll    (left/right tilt)
 *   Y = Yaw     (rotation about vertical axis)
 *
 * @param throttle Base throttle command (Q16.16)
 * @param roll Roll correction (Q16.16, positive = right)
 * @param pitch Pitch correction (Q16.16, positive = nose up)
 * @param yaw Yaw correction (Q16.16, positive = clockwise)
 * @param motor_out Array of 4 motor outputs (Q16.16), clamped to [PWM_MIN, PWM_MAX]
 */
void mix_and_output_motors(fx_t throttle, fx_t roll, fx_t pitch, fx_t yaw, fx_t *motor_out);

/**
 * Convert motor command (Q16.16) to PWM pulse width (microseconds).
 * Used for final output to timer peripheral.
 *
 * @param motor_cmd Motor command in Q16.16
 * @return PWM pulse width in microseconds (uint16_t)
 */
static inline uint16_t motor_to_pwm(fx_t motor_cmd) {
    return (uint16_t)fx_to_int(motor_cmd);
}

#endif /* CONTROL_H */
