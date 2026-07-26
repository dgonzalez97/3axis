# 3axis

Small C11 proof of concept for a single-axis spacecraft rate-damping
controller. The controller converts body angular velocity into a
reaction-wheel torque command and applies a configurable torque limit.


Build:

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
```

The exact final printed rate can vary slightly with the compiler and platform,
but it should remain close to zero and all checks should pass,

## TODO

- Implement the three-channel health-monitoring module.
- Add deterministic 100 Hz, 10 Hz, and 1 Hz channel scheduling.
- Add value and rate-of-change fault injection tests.
- Add native timing measurements and the results report.
- Add continuous-integration checks.
