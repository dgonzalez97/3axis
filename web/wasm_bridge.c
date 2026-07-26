#include "health_monitor.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

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
static bool wheel_saturated_since_health;
static rate_damping_status_t controller_status_since_health;
static hm_channel_state_t aocs_health_state;
static hm_channel_state_t battery_health_state;
static hm_channel_state_t gnss_health_state;
static hm_result_t health_result;

static const hm_aocs_config_t health_aocs_config = {
    1.5F, 6.7F, HM_AOCS_PERIOD_MS
};

static const hm_battery_config_t health_battery_config = {
    24.0F, 22.0F, 34.0F, 1.0F, 8.0F, HM_BATTERY_PERIOD_MS
};

static const hm_gnss_config_t health_gnss_config = {
    4.75F, 5.25F, 0.2F, 4U, HM_GNSS_PERIOD_MS
};

/*
 * Start a new single-axis simulation.
 */
EMSCRIPTEN_KEEPALIVE
void web_reset(float initial_body_rate_rad_s) {
    body_rate_rad_s = initial_body_rate_rad_s;
    wheel_torque_nm = 0.0F;
    wheel_saturated = false;
    wheel_saturated_since_health = false;
    controller_status_since_health = RATE_DAMPING_STATUS_OK;
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
        controller_status_since_health = status;
        wheel_torque_nm = 0.0F;
        wheel_saturated = false;
        return (int)status;
    }

    wheel_torque_nm = output.wheel_torque_command_nm;
    wheel_saturated = output.wheel_saturated;
    wheel_saturated_since_health =
        wheel_saturated_since_health || wheel_saturated;

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

EMSCRIPTEN_KEEPALIVE
void web_health_reset(void) {
    health_monitor_state_reset(&aocs_health_state);
    health_monitor_state_reset(&battery_health_state);
    health_monitor_state_reset(&gnss_health_state);
    health_result = (hm_result_t){0};
    wheel_saturated_since_health = false;
    controller_status_since_health = RATE_DAMPING_STATUS_OK;
}

/*
 * AOCS faults are latched between web reports so a short saturation is still
 * visible when the next five-second health message is published.
 */
EMSCRIPTEN_KEEPALIVE
int web_health_update_aocs(uint32_t timestamp_ms) {
    const hm_aocs_sample_t sample = {
        body_rate_rad_s,
        controller_status_since_health,
        wheel_saturated_since_health,
        (uint64_t)timestamp_ms
    };
    hm_status_t status;

    health_result = (hm_result_t){0};
    status = health_monitor_update_aocs(
        &health_aocs_config, &aocs_health_state, &sample, &health_result);

    wheel_saturated_since_health = false;
    controller_status_since_health = RATE_DAMPING_STATUS_OK;
    return (int)status;
}

EMSCRIPTEN_KEEPALIVE
int web_health_update_battery(
    float voltage_v,
    float current_a,
    uint32_t timestamp_ms) {
    const hm_battery_sample_t sample = {
        voltage_v, current_a, (uint64_t)timestamp_ms
    };

    health_result = (hm_result_t){0};
    return (int)health_monitor_update_battery(
        &health_battery_config, &battery_health_state, &sample,
        &health_result);
}

EMSCRIPTEN_KEEPALIVE
int web_health_update_gnss(
    float voltage_v,
    float satellites_in_view,
    int fix_valid,
    uint32_t timestamp_ms) {
    hm_gnss_sample_t sample;

    health_result = (hm_result_t){0};
    if (!isfinite(satellites_in_view) ||
        (satellites_in_view < 0.0F) ||
        (satellites_in_view > 255.0F)) {
        return (int)HM_STATUS_INVALID_INPUT;
    }

    sample.supply_voltage_v = voltage_v;
    sample.satellites_in_view = (uint8_t)satellites_in_view;
    sample.fix_valid = fix_valid != 0;
    sample.timestamp_ms = (uint64_t)timestamp_ms;

    return (int)health_monitor_update_gnss(
        &health_gnss_config, &gnss_health_state, &sample, &health_result);
}

EMSCRIPTEN_KEEPALIVE
int web_health_get_severity(void) {
    return (int)health_result.severity;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_health_get_faults(void) {
    return health_result.faults;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_health_get_actions(void) {
    return health_result.actions;
}
