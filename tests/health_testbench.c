#include "health_monitor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures;

static const hm_aocs_config_t aocs_test_config = {
    1.5F, 6.7F, HM_AOCS_PERIOD_MS
};  // Standard AOCS configuration for testing

static const hm_battery_config_t battery_test_config = {
    24.0F, 22.0F, 34.0F, 1.0F, HM_BATTERY_PERIOD_MS
};

static const hm_gnss_config_t gnss_test_config = {
    4.75F, 5.25F, 0.2F, 4U, HM_GNSS_PERIOD_MS
};

static void check(bool condition, const char *name) {
    if (condition) {
        (void)printf("[PASS] %s\n", name);
    } else {
        (void)printf("[FAIL] %s\n", name);
        failures++;
    }
}

//------------------------Testing -----------------

static void test_body_rate_faults(void) {
    hm_aocs_sample_t sample = {
        0.2F, RATE_DAMPING_STATUS_OK, false, 0U
    }; // Future cFE SB sample, filled directly for this native test

    hm_channel_state_t state;
    hm_result_t result;

    health_monitor_state_reset(&state);

    check(health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_NONE && !result.rate_valid,
                "first body-rate sample is healthy");

    /* A jump from 0.2 to 1.6 rad/s exceeds both configured limits. */
    sample.body_rate_rad_s = 1.6F;
    sample.timestamp_ms = 10U;   // (1.6 - 0.2) / 0.010 > 6.7

    check(health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == (HM_FAULT_BODY_RATE_LIMIT | HM_FAULT_BODY_RATE_CHANGE) &&
              result.severity == HM_SEVERITY_ERROR,
          "body-rate value and rate faults are detected");
}

static void test_controller_faults(void) {
    hm_aocs_sample_t sample = {
        0.2F, RATE_DAMPING_STATUS_INVALID_BODY_RATE, false, 0U
    };
    hm_channel_state_t state;
    hm_result_t result;

    health_monitor_state_reset(&state);
    check(health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_CONTROLLER_ERROR &&
              result.actions == HM_ACTION_REJECT_WHEEL_COMMAND,
          "controller error rejects wheel command");

    health_monitor_state_reset(&state);

    sample.controller_status = RATE_DAMPING_STATUS_OK;
    sample.wheel_saturated = true;

    check(health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_WHEEL_SATURATED &&
              result.severity == HM_SEVERITY_WARNING,
          "wheel saturation produces warning");
}

static void test_battery_faults(void) {
    hm_battery_sample_t sample = {28.0F, 0U};
    hm_channel_state_t state;
    hm_result_t result;

    health_monitor_state_reset(&state);
    check(health_monitor_update_battery(&battery_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_NONE,
          "nominal battery voltage is healthy");

    /* A 0.2 V drop in 100 ms is -2 V/s, beyond the configured -1 V/s. */
    sample.battery_voltage_v = 27.8F;
    sample.timestamp_ms = 100U;
    check(health_monitor_update_battery(&battery_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_BATTERY_DROP_RATE &&
              result.severity == HM_SEVERITY_WARNING,
          "fast battery voltage drop produces warning");

    health_monitor_state_reset(&state);
    sample.battery_voltage_v = 23.5F;
    sample.timestamp_ms = 0U;
    check(health_monitor_update_battery(&battery_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_BATTERY_LOW &&
              result.actions == HM_ACTION_REQUEST_AOCS_OFF,
          "low battery requests AOCS off");

    health_monitor_state_reset(&state);
    sample.battery_voltage_v = 21.5F;
    check(health_monitor_update_battery(&battery_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_BATTERY_CRITICAL &&
              result.severity == HM_SEVERITY_CRITICAL &&
              result.actions == HM_ACTION_REQUEST_AOCS_OFF,
          "critical battery produces critical health");

    health_monitor_state_reset(&state);
    sample.battery_voltage_v = 35.0F;
    check(health_monitor_update_battery(&battery_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_BATTERY_OVERVOLTAGE,
          "battery overvoltage is detected");
}

static void test_gnss_faults(void) {
    hm_gnss_sample_t sample = {5.0F, 8U, true, 0U};
    hm_gnss_config_t boundary_config = gnss_test_config;
    hm_channel_state_t state;
    hm_result_t result;

    boundary_config.maximum_voltage_rate_v_s = 0.25F;
    health_monitor_state_reset(&state);
    check(health_monitor_update_gnss(&boundary_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_NONE,
          "nominal GNSS fix is healthy");

    /* Both the maximum voltage and maximum voltage rate are inclusive limits. */
    sample.supply_voltage_v = 5.25F;
    sample.timestamp_ms = 1000U;
    check(health_monitor_update_gnss(&boundary_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == (HM_FAULT_GNSS_VOLTAGE | HM_FAULT_GNSS_VOLTAGE_RATE) &&
              result.severity == HM_SEVERITY_ERROR &&
              result.actions == HM_ACTION_USE_BACKUP_NAVIGATION,
          "GNSS maximum voltage and rate are inclusive");

    health_monitor_state_reset(&state);
    sample.supply_voltage_v = 4.5F;
    sample.timestamp_ms = 0U;
    check(health_monitor_update_gnss(&gnss_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == HM_FAULT_GNSS_VOLTAGE &&
              result.actions == HM_ACTION_USE_BACKUP_NAVIGATION,
          "bad GNSS voltage selects backup navigation");

    health_monitor_state_reset(&state);
    sample.supply_voltage_v = 5.0F;
    sample.satellites_in_view = 2U;
    sample.fix_valid = false;
    check(health_monitor_update_gnss(&gnss_test_config, &state, &sample, &result) == HM_STATUS_OK &&
              result.faults == (HM_FAULT_GNSS_SATELLITES | HM_FAULT_GNSS_FIX) &&
              result.actions == HM_ACTION_USE_BACKUP_NAVIGATION,
          "bad GNSS fix and satellite count are detected");
}

static void test_invalid_input_and_timestamp(void) {
    hm_aocs_sample_t sample = {
        0.2F, RATE_DAMPING_STATUS_OK, false, 100U
    };
    hm_channel_state_t state;
    hm_result_t result;

    health_monitor_state_reset(&state);

    check(health_monitor_update_aocs(NULL, &state, &sample, &result) == HM_STATUS_NULL_POINTER,
          "null configuration is rejected");

    check(health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result) == HM_STATUS_OK,
          "valid health sample is accepted");

    check(health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result) == HM_STATUS_TIMESTAMP_ERROR &&
              result.faults == HM_FAULT_TIMESTAMP,
          "repeated timestamp is rejected");
}

static void test_sampling_frequency(void) {
    hm_aocs_sample_t aocs_sample = {
        0.2F, RATE_DAMPING_STATUS_OK, false, 0U
    };
    hm_battery_sample_t battery_sample = {28.0F, 0U};
    hm_gnss_sample_t gnss_sample = {5.0F, 8U, true, 0U};
    hm_channel_state_t aocs_state;
    hm_channel_state_t battery_state;
    hm_channel_state_t gnss_state;
    hm_result_t result;
    uint64_t time_ms;

    health_monitor_state_reset(&aocs_state);
    health_monitor_state_reset(&battery_state);
    health_monitor_state_reset(&gnss_state);

    /* The C functions are really called at 100 Hz, 10 Hz and 1 Hz. */
    for (time_ms = 0U; time_ms < 2000U; time_ms++) {
        if ((time_ms % aocs_test_config.nominal_period_ms) == 0U) {
            aocs_sample.timestamp_ms = time_ms;
            (void)health_monitor_update_aocs(&aocs_test_config, &aocs_state, &aocs_sample, &result);
        }

        if ((time_ms % battery_test_config.nominal_period_ms) == 0U) {
            battery_sample.timestamp_ms = time_ms;
            (void)health_monitor_update_battery(&battery_test_config, &battery_state, &battery_sample, &result);
        }

        if ((time_ms % gnss_test_config.nominal_period_ms) == 0U) {
            gnss_sample.timestamp_ms = time_ms;
            (void)health_monitor_update_gnss(&gnss_test_config, &gnss_state, &gnss_sample, &result);
        }
    }

    check((aocs_state.sample_count == 200U) &&
              (battery_state.sample_count == 20U) &&
              (gnss_state.sample_count == 2U),
          "health channels execute at 100 Hz, 10 Hz and 1 Hz");
}

int main(void) {
    test_body_rate_faults();
    test_controller_faults();
    test_battery_faults();
    test_gnss_faults();
    test_invalid_input_and_timestamp();
    test_sampling_frequency();

    if (failures == 0) {
        (void)printf("All health-monitor checks passed.\n");
        return 0;
    }

    (void)printf("%d health-monitor check(s) failed.\n", failures);
    return 1;
}
