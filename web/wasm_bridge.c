#include "rate_damping.h"

#include <stdbool.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#define WEB_STEP_S 0.01F
#define WEB_SPACECRAFT_INERTIA_KG_M2 0.67F

static float body_rate_rad_s;
static float wheel_torque_nm;
static bool wheel_saturated;

/*
 * Start a new single-axis simulation.
 */
EMSCRIPTEN_KEEPALIVE
void web_reset(float initial_body_rate_rad_s) {
    body_rate_rad_s = initial_body_rate_rad_s;
    wheel_torque_nm = 0.0F;
    wheel_saturated = false;
}

/*
 * Run one 10 ms controller and rigid-body simulation step.
 */
EMSCRIPTEN_KEEPALIVE
int web_step(float damping_gain_nms, float wheel_torque_limit_nm) {
    const rate_damping_config_t config = {
        damping_gain_nms, wheel_torque_limit_nm
    };
    rate_damping_output_t output;
    rate_damping_status_t status;

    status = rate_damping_step(&config, body_rate_rad_s, &output);
    if (status != RATE_DAMPING_STATUS_OK) {
        wheel_torque_nm = 0.0F;
        wheel_saturated = false;
        return (int)status;
    }

    wheel_torque_nm = output.wheel_torque_command_nm;
    wheel_saturated = output.wheel_saturated;

    body_rate_rad_s +=
        (-wheel_torque_nm / WEB_SPACECRAFT_INERTIA_KG_M2) * WEB_STEP_S;

    return (int)status;
}

EMSCRIPTEN_KEEPALIVE
float web_get_body_rate(void) {
    return body_rate_rad_s;
}

EMSCRIPTEN_KEEPALIVE
float web_get_wheel_torque(void) {
    return wheel_torque_nm;
}

EMSCRIPTEN_KEEPALIVE
int web_get_wheel_saturated(void) {
    return wheel_saturated ? 1 : 0;
}
