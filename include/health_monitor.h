#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include "rate_damping.h"

#include <stdbool.h>
#include <stdint.h>

#define HM_AOCS_PERIOD_MS 10U
#define HM_BATTERY_PERIOD_MS 100U
#define HM_GNSS_PERIOD_MS 1000U

typedef enum {
    HM_STATUS_OK = 0,
    HM_STATUS_NULL_POINTER,
    HM_STATUS_INVALID_CONFIG,
    HM_STATUS_INVALID_INPUT,
    HM_STATUS_TIMESTAMP_ERROR,
    HM_STATUS_NUMERIC_ERROR
} hm_status_t;

/* Higher values represent more serious conditions. */
typedef enum {
    HM_SEVERITY_OK = 0,
    HM_SEVERITY_WARNING,
    HM_SEVERITY_ERROR,
    HM_SEVERITY_CRITICAL
} hm_severity_t;

/*
 * Faults are stored as bits in one 32-bit value. This allows several faults
 * to be reported in the same sample and is easy to transmit in telemetry.
 *
 */
typedef uint32_t hm_fault_flags_t;

#define HM_FAULT_NONE 0x00000000U
#define HM_FAULT_BODY_RATE_LIMIT 0x00000001U
#define HM_FAULT_BODY_RATE_CHANGE 0x00000002U
#define HM_FAULT_CONTROLLER_ERROR 0x00000004U
#define HM_FAULT_WHEEL_SATURATED 0x00000008U
#define HM_FAULT_TIMESTAMP 0x00000010U
#define HM_FAULT_BATTERY_LOW 0x00000020U
#define HM_FAULT_BATTERY_CRITICAL 0x00000040U
#define HM_FAULT_BATTERY_OVERVOLTAGE 0x00000080U
#define HM_FAULT_BATTERY_DROP_RATE 0x00000100U
#define HM_FAULT_GNSS_VOLTAGE 0x00000200U
#define HM_FAULT_GNSS_VOLTAGE_RATE 0x00000400U
#define HM_FAULT_GNSS_SATELLITES 0x00000800U
#define HM_FAULT_GNSS_FIX 0x00001000U

typedef uint32_t hm_action_flags_t;  //Easy to transmit in telemetry, can be used to trigger actions in the cFE application. The actions are not mutually exclusive, so several bits can be set at once.

#define HM_ACTION_NONE 0x00000000U
#define HM_ACTION_REJECT_WHEEL_COMMAND 0x00000001U
#define HM_ACTION_REQUEST_AOCS_OFF 0x00000002U
#define HM_ACTION_USE_BACKUP_NAVIGATION 0x00000004U

typedef struct {
    bool initialised;
    float previous_value;
    uint64_t previous_timestamp_ms;
    uint32_t sample_count;
    hm_fault_flags_t active_faults;
} hm_channel_state_t;

typedef struct {
    float monitored_value;
    float calculated_rate_per_s;
    bool rate_valid;
    uint32_t sample_count;
    hm_fault_flags_t faults;
    hm_action_flags_t actions;
    hm_severity_t severity;
} hm_result_t;

typedef struct {
    float maximum_absolute_body_rate_rad_s;
    float maximum_absolute_rate_change_rad_s2;
    uint32_t nominal_period_ms;
} hm_aocs_config_t;

/*
 * Values received by the AOCS health monitor for one sample. In a future
 * cFE application, these fields could come from a Software Bus message.
 */
typedef struct {
    float body_rate_rad_s;
    rate_damping_status_t controller_status;
    bool wheel_saturated;
    uint64_t timestamp_ms;
} hm_aocs_sample_t;

typedef struct {
    float aocs_off_voltage_v;
    float critical_voltage_v;
    float maximum_voltage_v;
    float maximum_drop_rate_v_s;
    uint32_t nominal_period_ms;
} hm_battery_config_t;

typedef struct {
    float battery_voltage_v;
    uint64_t timestamp_ms;
} hm_battery_sample_t;

typedef struct {
    float minimum_supply_voltage_v;
    float maximum_supply_voltage_v;
    float maximum_voltage_rate_v_s;
    uint8_t minimum_satellites_in_view;
    uint32_t nominal_period_ms;
} hm_gnss_config_t;

typedef struct {
    float supply_voltage_v;
    uint8_t satellites_in_view;
    bool fix_valid;
    uint64_t timestamp_ms;
} hm_gnss_sample_t;

/*
 * Clears the saved sample, timestamp, counter and active faults.
 * Call this once before using a channel state for the first time.
 */
void health_monitor_state_reset(hm_channel_state_t *state);

/*
 * Checks one AOCS body-rate sample, its rate of change, the return status from
 * rate_damping_step() and wheel saturation. The previous sample is kept in
 * state; faults, severity and recommended actions are written to result.
 *
 * sample->timestamp_ms must increase on every call. A controller error causes
 * the reaction-wheel torque command to be rejected.
 */
hm_status_t health_monitor_update_aocs(
    const hm_aocs_config_t *config,
    hm_channel_state_t *state,
    const hm_aocs_sample_t *sample,
    hm_result_t *result);

/*
 * Checks battery voltage and voltage drop rate. Low battery recommends that
 * the mode manager switches AOCS off; the health monitor does not perform the
 * mode change directly.
 */
hm_status_t health_monitor_update_battery(
    const hm_battery_config_t *config,
    hm_channel_state_t *state,
    const hm_battery_sample_t *sample,
    hm_result_t *result);

/*
 * Checks GNSS supply voltage, voltage rate, fix validity and satellites in
 * view. Invalid voltage or navigation data recommends using the backup
 * navigation source; voltage rate alone is reported as a warning.
 */
hm_status_t health_monitor_update_gnss(
    const hm_gnss_config_t *config,
    hm_channel_state_t *state,
    const hm_gnss_sample_t *sample,
    hm_result_t *result);

/*
 * GNSS provides position and time, not a TLE. A TLE would be monitored
 * separately as input to an orbit propagator.
 *
 * A future cFE application can call these functions from its scheduled
 * wake-up and publish fault transitions through Event Services. This follows
 * the periodic supervision idea of cFS Health and Safety (HS), while keeping
 * the limit checks independent from cFE and easy to test natively.
 */

#endif
