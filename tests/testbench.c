#include "rate_damping.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures;

static void check(bool condition, const char *name) {   //Implemented my own checks, here i normally use either assert, or ceedling/Unity 
    if (condition) {
        (void)printf("[PASS] %s\n", name);
    } else {
        (void)printf("[FAIL] %s\n", name);
        failures++;
    }
}

static bool close_to(float actual, float expected)
    { return fabsf(actual - expected) < 1.0e-6F; }  // Check if two floating-point values are close to each other, AOCS standards these values. The math library implementation is to be tested

//------------------------Testing -----------------

    static void test_controller_values(void) {
    
    const rate_damping_config_t config = {0.5F, 0.2F}; // TO_DO, for testing purposes, we can change the values of damping gain and wheel torque limit to see how the controller behaves with different configurations.
    rate_damping_output_t output;

    check(rate_damping_step(&config, 0.0F, &output) == RATE_DAMPING_STATUS_OK && close_to(output.wheel_torque_command_nm, 0.0F) && !output.wheel_saturated, "zero rate produces zero torque");
    check(rate_damping_step(&config, 0.2F, &output) == RATE_DAMPING_STATUS_OK && close_to(output.wheel_torque_command_nm, 0.1F) && !output.wheel_saturated, "positive rate produces positive wheel torque");
    check(rate_damping_step(&config, -0.2F, &output) == RATE_DAMPING_STATUS_OK && close_to(output.wheel_torque_command_nm, -0.1F) && !output.wheel_saturated, "negative rate produces negative wheel torque");
    check(rate_damping_step(&config, 2.0F, &output) == RATE_DAMPING_STATUS_OK && close_to(output.wheel_torque_command_nm, 0.2F) && output.wheel_saturated, "positive torque is limited");
    check(rate_damping_step(&config, -2.0F, &output) == RATE_DAMPING_STATUS_OK && close_to(output.wheel_torque_command_nm, -0.2F) && output.wheel_saturated, "negative torque is limited");
}

static void test_invalid_inputs(void) {
    const rate_damping_config_t valid_config = {0.5F, 0.2F};
    const rate_damping_config_t invalid_config = {-0.5F, 0.2F};
    const rate_damping_config_t overflow_config = {FLT_MAX, FLT_MAX};
    rate_damping_output_t output;

    check(rate_damping_step(NULL, 0.1F, &output) == RATE_DAMPING_STATUS_NULL_POINTER, "null configuration is rejected");
    check(rate_damping_step(&valid_config, 0.1F, NULL) == RATE_DAMPING_STATUS_NULL_POINTER, "null output is rejected");
    check(rate_damping_step(&invalid_config, 0.1F, &output) == RATE_DAMPING_STATUS_INVALID_CONFIG, "negative damping gain is rejected");
    check(rate_damping_step(&valid_config, NAN, &output) == RATE_DAMPING_STATUS_INVALID_BODY_RATE, "NaN body rate is rejected");
    check(rate_damping_step(&overflow_config, FLT_MAX, &output) == RATE_DAMPING_STATUS_NUMERIC_ERROR, "numeric overflow is rejected");
}

static void test_closed_loop_convergence(void) {
    
    const rate_damping_config_t config = {0.8F, 0.15F};
    const float spacecraft_inertia_kg_m2 = 0.67F; // asumtion, to be fuzzy tested later
    
    const uint32_t controller_period_ms = UINT32_C(10);
    const uint32_t simulation_duration_ms = UINT32_C(4500);
    const uint32_t simulation_steps = simulation_duration_ms / controller_period_ms;
    const float controller_period_s = (float)controller_period_ms / 1000.0F;
    
    
    const float initial_body_rate_rad_s = 0.6F;
    const float settling_limit_rad_s = initial_body_rate_rad_s * 0.02F;

    rate_damping_output_t output;
    float body_rate_rad_s = initial_body_rate_rad_s; // initial body rate for the closed-loop simulation, to be fuzzy tested later

    uint32_t step;

    for (step = 0U; step < simulation_steps; step++) {
        if (rate_damping_step(&config, body_rate_rad_s, &output) != RATE_DAMPING_STATUS_OK) {
            check(false, "closed-loop simulation executes");
            return;
        }

        body_rate_rad_s += (-output.wheel_torque_command_nm / spacecraft_inertia_kg_m2) * controller_period_s;
    }

    (void)printf("Simulated time: %u ms\n", (unsigned int)simulation_duration_ms);
    (void)printf("Final simulated body rate: %.6f rad/s\n", (double)body_rate_rad_s);

    check(fabsf(body_rate_rad_s) < settling_limit_rad_s, "closed-loop body rate settles below 2% of initial rate");
}

int main(void) {

    test_controller_values();
    test_invalid_inputs();
    test_closed_loop_convergence();

    if (failures == 0) {
        (void)printf("All rate-damping checks passed.\n");
        return 0;
    }

    (void)printf("%d check(s) failed.\n", failures);
    return 1;
}
