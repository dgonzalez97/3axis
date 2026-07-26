#include "health_monitor.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

static bool aocs_config_is_valid(const hm_aocs_config_t *config) {
    if (config == NULL) {
        return false;
    }

    return isfinite(config->maximum_absolute_body_rate_rad_s) &&
           isfinite(config->maximum_absolute_rate_change_rad_s2) &&
           (config->maximum_absolute_body_rate_rad_s > 0.0F) &&
           (config->maximum_absolute_rate_change_rad_s2 > 0.0F) &&
           (config->nominal_period_ms > 0U);
}

static void prepare_result_for_sample(hm_result_t *result, float value) {
    result->monitored_value = value;
    result->calculated_rate_per_s = 0.0F;
    result->rate_valid = false;
    result->sample_count = 0U;
    result->faults = HM_FAULT_NONE;
    result->actions = HM_ACTION_NONE;
    result->severity = HM_SEVERITY_OK;
}

static void record_fault(hm_result_t *result, hm_fault_flags_t fault, hm_severity_t severity, hm_action_flags_t action) {
    /*
     * faults and actions are bit masks. OR keeps anything that was already
     * found during this sample and adds the new fault and recommended action.
     */
    result->faults |= fault;
    result->actions |= action;

    /* The severity enum is ordered from OK to ERROR. */
    if (severity > result->severity) {
        result->severity = severity;
    }
}

static hm_status_t check_timing_and_calculate_rate(hm_channel_state_t *state, float value, uint64_t timestamp_ms, hm_result_t *result) {
    /*
     * Rate of change is calculated from two consecutive samples:
     *
     *     rate = (current value - previous value) / elapsed time
     *
     * The actual timestamp difference is used, even if the scheduler is late.
     */
    if (state->initialised) {
        uint64_t elapsed_time_ms;
        float elapsed_time_s;

        /* Equal or older timestamps would cause an invalid time division. */
        if (timestamp_ms <= state->previous_timestamp_ms) {
            record_fault(result, HM_FAULT_TIMESTAMP, HM_SEVERITY_ERROR, HM_ACTION_NONE);
            state->active_faults = result->faults;
            result->sample_count = state->sample_count;
            return HM_STATUS_TIMESTAMP_ERROR;
        }

        elapsed_time_ms = timestamp_ms - state->previous_timestamp_ms;
        elapsed_time_s = (float)elapsed_time_ms / 1000.0F;
        result->calculated_rate_per_s = (value - state->previous_value) / elapsed_time_s;

        if (!isfinite(result->calculated_rate_per_s)) {
            return HM_STATUS_NUMERIC_ERROR;
        }

        result->rate_valid = true;
    }

    /*
     * Only save the sample after all calculations succeed.
     */
    state->initialised = true;
    state->previous_value = value;
    state->previous_timestamp_ms = timestamp_ms;
    if (state->sample_count < UINT32_MAX) {
        state->sample_count++;
    }
    result->sample_count = state->sample_count;
    return HM_STATUS_OK;
}

void health_monitor_state_reset(hm_channel_state_t *state) {
    if (state == NULL) {
        return;
    }

    state->initialised = false;
    state->previous_value = 0.0F;
    state->previous_timestamp_ms = 0U;
    state->sample_count = 0U;
    state->active_faults = HM_FAULT_NONE;
}

hm_status_t health_monitor_update_aocs(const hm_aocs_config_t *config, hm_channel_state_t *state, const hm_aocs_sample_t *sample, hm_result_t *result) {
    hm_status_t status;

    if ((config == NULL) || (state == NULL) || (sample == NULL) || (result == NULL)) {
        return HM_STATUS_NULL_POINTER;
    }
    if (!aocs_config_is_valid(config)) {
        return HM_STATUS_INVALID_CONFIG;
    }
    if (!isfinite(sample->body_rate_rad_s)) {
        return HM_STATUS_INVALID_INPUT;
    }

    prepare_result_for_sample(result, sample->body_rate_rad_s);
    status = check_timing_and_calculate_rate(state, sample->body_rate_rad_s, sample->timestamp_ms, result);
    if (status != HM_STATUS_OK) {
        return status;
    }

    /* FDIR checks for the current AOCS sample. */

    if (fabsf(sample->body_rate_rad_s) > config->maximum_absolute_body_rate_rad_s) {
        record_fault(result, HM_FAULT_BODY_RATE_LIMIT, HM_SEVERITY_ERROR, HM_ACTION_NONE);
    }
    if (result->rate_valid && (fabsf(result->calculated_rate_per_s) > config->maximum_absolute_rate_change_rad_s2)) {
        record_fault(result, HM_FAULT_BODY_RATE_CHANGE, HM_SEVERITY_WARNING, HM_ACTION_NONE);
    }
    if (sample->controller_status != RATE_DAMPING_STATUS_OK) {
        record_fault(result, HM_FAULT_CONTROLLER_ERROR, HM_SEVERITY_ERROR, HM_ACTION_REJECT_WHEEL_COMMAND);
    } else if (sample->wheel_saturated) {
        record_fault(result, HM_FAULT_WHEEL_SATURATED, HM_SEVERITY_WARNING, HM_ACTION_NONE);
    }

    state->active_faults = result->faults;
    return HM_STATUS_OK;
}
