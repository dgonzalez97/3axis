#ifndef RATE_DAMPING_H
#define RATE_DAMPING_H

#include <stdbool.h>

typedef struct {   // This struct contains the configuration parameters for the rate damping algorithm. Should be either in cFE Tables or from AOCS algorithm 
    float damping_gain_nms;
    float wheel_torque_limit_nm;
} rate_damping_config_t;

typedef enum {
    RATE_DAMPING_STATUS_OK = 0,
    RATE_DAMPING_STATUS_NULL_POINTER,
    RATE_DAMPING_STATUS_INVALID_CONFIG,
    RATE_DAMPING_STATUS_INVALID_BODY_RATE,
    RATE_DAMPING_STATUS_NUMERIC_ERROR
} rate_damping_status_t; // for the HS app 

typedef struct {  // Struct to hold the output of the rate damping algorithm. Contains the wheel torque command and a flag indicating if the wheel is saturated.
    float wheel_torque_command_nm;
    bool wheel_saturated;
} rate_damping_output_t;  // Will be shared with SB from cFE

/*
 * Control law:
 *   wheel_torque_command = damping_gain * body_rate
 *
 * Positive body rate commands positive reaction-wheel torque.
 * The equal-and-opposite spacecraft body torque therefore damps the rate.
 *
 * RATE_DAMPING_STATUS_OK means output contains a valid command.
 * wheel_saturated reports whether that command was limited.
 */
rate_damping_status_t rate_damping_step(
    const rate_damping_config_t *config,
    float body_rate_rad_s,
    rate_damping_output_t *output);

#endif
