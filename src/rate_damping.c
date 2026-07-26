#include "rate_damping.h"
#include <math.h>
#include <stddef.h>

static bool config_is_valid(const rate_damping_config_t *config) {
    if (config == NULL) {
        return false;
    }

    return isfinite(config->damping_gain_nms) && 
           isfinite(config->wheel_torque_limit_nm) && 
           (config->damping_gain_nms >= 0.0F) && 
           (config->wheel_torque_limit_nm > 0.0F); //make sure all numbers are finite and that the damping gain is non-negative and the wheel torque limit is positive (¿0?, ask aocs)
}

rate_damping_status_t rate_damping_step(const rate_damping_config_t *config, float body_rate_rad_s, rate_damping_output_t *output) {
    float unsaturated_wheel_torque_nm;
    float wheel_torque_command_nm;

    if ((config == NULL) || (output == NULL)) {
        return RATE_DAMPING_STATUS_NULL_POINTER;
    }

    if (!config_is_valid(config)) {
        return RATE_DAMPING_STATUS_INVALID_CONFIG;
    }

    if (!isfinite(body_rate_rad_s)) {
        return RATE_DAMPING_STATUS_INVALID_BODY_RATE;
    }

    /*
     * Proportional rate-damping control law:
     *
     *   wheel torque = damping gain * spacecraft body rate
     *   tau_wheel    = Kd           * omega_body
     *
     * Units:
     *
     *   (N*m*s) * (rad/s) = N*m
     *
     */
    unsaturated_wheel_torque_nm = config->damping_gain_nms * body_rate_rad_s;

    /*
     * Two valid finite float values can still overflow when multiplied.
     * Reject that result instead of passing NaN or infinity to an actuator.
     */

    if (!isfinite(unsaturated_wheel_torque_nm)) {
        return RATE_DAMPING_STATUS_NUMERIC_ERROR;
    }

    /*
     * Apply the reaction wheel's physical torque limit. Saturation is a
     * valid controller result, so it is reported in the output rather than
     * returned as a software error.
     */
        wheel_torque_command_nm = unsaturated_wheel_torque_nm;
         output->wheel_saturated = false;
    
    if (wheel_torque_command_nm > config->wheel_torque_limit_nm) {
        wheel_torque_command_nm = config->wheel_torque_limit_nm;
        output->wheel_saturated = true;
    
    } else if (wheel_torque_command_nm < -config->wheel_torque_limit_nm) { 
        wheel_torque_command_nm = -config->wheel_torque_limit_nm;   //also check for negative saturation
        output->wheel_saturated = true;
    }

    output->wheel_torque_command_nm = wheel_torque_command_nm;
    return RATE_DAMPING_STATUS_OK;
}
