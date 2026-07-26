# 3axis

Small C11 proof of concept for a single-axis spacecraft rate-damping
controller and a three-channel health monitor.

The controller converts body angular velocity into a reaction-wheel torque
command and applies a configurable torque limit. The health monitor checks
AOCS, battery and GNSS samples and returns 32-bit fault and action words.

## Browser demo

The live WebAssembly demo is available at
[dgonzalez97.github.io/3axis](https://dgonzalez97.github.io/3axis/).
Its browser and deployment files stay on the `github-pages` branch.

## Build and test

```sh
make
make test
```

Remove generated files with `make clean`.

The build uses:

```text
-std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wshadow
```

## Functionality

The controller uses:

```text
wheel torque = damping gain * body rate
```

The equal-and-opposite torque applied to the spacecraft body reduces its body
rate. These health thresholds are demonstration values, not mission
requirements:

| Channel | Frequency | Checks | Recommended action |
|---|---:|---|---|
| AOCS | 100 Hz | Body rate above 1.5 rad/s, rate change above 6.7 rad/s2, controller status and wheel saturation | Reject an invalid wheel command |
| Battery | 10 Hz | Below 24 V, critical at or below 22 V, at or above 34 V, or a drop faster than 1 V/s | Request AOCS off for low or critical voltage |
| GNSS | 1 Hz | Supply outside 4.75 V to less than 5.25 V, voltage rate at or above 0.2 V/s, fewer than four satellites, or an invalid fix | Use backup navigation |

Rate of change uses the actual difference between timestamps. The first sample
creates the history and does not produce a rate.

`hm_status_t` reports whether the calculation ran correctly; it does not
describe subsystem health. When it returns `HM_STATUS_OK`, `hm_result_t`
contains the current fault word, highest severity and recommended action word.
Several fault or action bits can be set in the same result.

## Tests

`make test` runs the rate-damping and health-monitor testbenches. A successful
run ends with:

```text
All rate-damping checks passed.
All health-monitor checks passed.
```

## Real-time and cFE use

The algorithm code has no dynamic allocation, blocking calls, file access or
console output. Each update performs bounded O(1) work and keeps its history in
an explicit state structure.

There is currently no cFE or FreeRTOS dependency. A future cFE wrapper can
receive samples through Software Bus, load limits through Table Services and
report fault transitions through Event Services. Action flags are
recommendations; the health monitor does not switch hardware off directly.

The periodic supervision follows the idea of
[cFS Health and Safety (HS)](https://github.com/nasa/HS). The configurable
threshold checks are similar to
[cFS Limit Checker (LC)](https://github.com/nasa/LC).

GNSS provides position and time, not a TLE. A TLE would be separate input to an
orbit propagator and could be monitored as another channel later.

## TODO

- Add native timing measurements and the results report.
- Add a deterministic combined command-line simulation.
- Add a cFE adapter around the independent C algorithms.
- Add continuous-integration checks for `main`.
