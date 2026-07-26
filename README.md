# 3axis

Small C11 proof of concept for a single-axis spacecraft rate-damping
controller and a three-channel health monitor.

The controller converts body angular velocity into a reaction-wheel torque
command and applies a configurable torque limit. The health monitor checks
AOCS, battery and GNSS samples at different frequencies.

## Build and run

Build both test programs:

```sh
make
```

```sh
make test
```

The build enables strict compiler warnings:

```text
-Wall -Wextra -Wpedantic -Wconversion -Wshadow
```

## Functionality

The rate-damping controller uses:

wheel torque = damping gain * body rate

| Channel | Frequency | Checks | Recommended action |
|---|---:|---|---|
| AOCS body rate | 100 Hz | 1.5 rad/s value limit, 6.7 rad/s2 rate limit, controller status and wheel saturation | Reject an invalid wheel command |
| Battery | 10 Hz | Below 24 V, critical below 22 V, above 34 V, a drop faster than 1 V/s, or current at/above 8 A | Request AOCS off for low voltage or overcurrent |
| GNSS | 1 Hz | 4.75-5.25 V supply, 0.2 V/s voltage rate, at least four satellites and a valid fix | Request backup navigation for invalid voltage or navigation data |

Rate of change is calculated using the actual difference between timestamps.
The first sample creates the history and does not produce a rate.

`hm_status_t` reports whether the health calculation ran correctly. It does
not report whether the subsystem is healthy. When the return status is
`HM_STATUS_OK`, `hm_result_t` contains the current fault bits, highest severity
and recommended action bits.

The return value from `rate_damping_step()` is included in the AOCS health
sample. A controller error raises `HM_FAULT_CONTROLLER_ERROR` and recommends
`HM_ACTION_REJECT_WHEEL_COMMAND`. Wheel saturation is a valid controller result
and is reported as a warning.

## Expected result

`make test` should finish with output similar to:

```text
[PASS] zero rate produces zero torque
[PASS] positive rate produces positive wheel torque
[PASS] negative rate produces negative wheel torque
[PASS] positive torque is limited
[PASS] negative torque is limited
[PASS] null configuration is rejected
[PASS] null output is rejected
[PASS] negative damping gain is rejected
[PASS] NaN body rate is rejected
[PASS] numeric overflow is rejected
Simulated time: 4500 ms
Final simulated body rate: 0.007703 rad/s
[PASS] closed-loop body rate settles below 2% of initial rate
All rate-damping checks passed.
[PASS] first body-rate sample is healthy
[PASS] body-rate value and rate faults are detected
[PASS] controller error rejects wheel command
[PASS] wheel saturation produces warning
[PASS] nominal battery voltage is healthy
[PASS] fast battery voltage drop produces warning
[PASS] low battery requests AOCS off
[PASS] critical battery produces critical health
[PASS] battery overvoltage is detected
[PASS] battery overcurrent requests AOCS off
[PASS] nominal GNSS fix is healthy
[PASS] GNSS maximum voltage and rate are inclusive
[PASS] bad GNSS voltage selects backup navigation
[PASS] bad GNSS fix and satellite count are detected
[PASS] null configuration is rejected
[PASS] valid health sample is accepted
[PASS] repeated timestamp is rejected
[PASS] health channels execute at 100 Hz, 10 Hz and 1 Hz
All health-monitor checks passed.
```

The exact final printed rate can vary slightly with the compiler and platform,
but it should remain close to zero and all checks should pass.

## Real-time and cFE use

There is currently no cFE or FreeRTOS dependency in the algorithm. 

A future cFE wrapper can receive samples through Software Bus, load limits through Table
Services and report fault transitions through Event Services. 

Action flags are recommendations for a future mode-management application; the health monitor
does not switch off AOCS hardware itself.

The periodic supervision follows the idea of
[cFS Health and Safety (HS)](https://github.com/nasa/HS). The configurable
threshold checks are similar to
[cFS Limit Checker (LC)](https://github.com/nasa/LC).

## TODO

- Add native timing measurements and the results report.
- Add a deterministic combined command-line simulation.
- Add a cFE adapter around the independent C algorithms.
- Add continuous-integration checks.
- Add the WebAssembly and GitHub Pages demonstration.
