#include "health_monitor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures;

static const hm_aocs_config_t aocs_test_config = {
    1.5F, 6.7F, HM_AOCS_PERIOD_MS
};  // Standard AOCS configuration for testing

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
    hm_aocs_sample_t sample = {
        0.2F, RATE_DAMPING_STATUS_OK, false, 0U
    };
    hm_channel_state_t state;
    hm_result_t result;
    uint64_t time_ms;

    health_monitor_state_reset(&state);

    /* A 10 ms period produces 200 real health updates in 2 seconds. */
    for (time_ms = 0U; time_ms < 2000U; time_ms++) {
        if ((time_ms % aocs_test_config.nominal_period_ms) == 0U) {

            sample.timestamp_ms = time_ms;
            (void)health_monitor_update_aocs(&aocs_test_config, &state, &sample, &result);
        }
    }

    check(state.sample_count == 200U, "AOCS health channel executes at 100 Hz");
}

int main(void) {
    test_body_rate_faults();
    test_controller_faults();
    test_invalid_input_and_timestamp();
    test_sampling_frequency();

    if (failures == 0) {
        (void)printf("All health-monitor checks passed.\n");
        return 0;
    }

    (void)printf("%d health-monitor check(s) failed.\n", failures);
    return 1;
}
