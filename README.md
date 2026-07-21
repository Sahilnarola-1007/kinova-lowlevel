# kinova-lowlevel

1 kHz low-level cyclic control experiments on a Kinova Gen3 7-DOF arm, using the
Kortex `BaseCyclic` interface over UDP.

This repository is the platform layer for a force-conditioned manipulation
project: it establishes the **joint-velocity command primitive** that every
higher layer (differential IK, admittance reflex, policy output) ultimately
terminates in. The Gen3's low-level interface accepts joint *positions*, so a
correct host-side integration step is what turns a commanded velocity into
actual motion. Getting that step wrong is silent — the arm moves, just far less
than commanded. See `design.md` for the failure and the fix.

**Scope.** Single-joint validation only. No Cartesian control, no force control,
no ROS 2. Standalone binary, hardware-only.

---

## Contents

```
src/low_level_feedback_loop.cpp   1 kHz sinusoid tracking on joint 7
analysis/fit_k.py                 least-squares estimate of servo gain K
analysis/plot_tracking.py         position + gap plots from a run log
design.md                         integration scheme: failure, cause, fix
```

## Requirements

- Ubuntu 24.04, GCC 13+, CMake 3.16+
- Kortex C++ SDK, extracted (tested against the vendor `kortex_api` bundle)
- Python 3 with `pandas`, `numpy`, `matplotlib` for the analysis scripts
- Kinova Gen3 reachable at `192.168.1.10`

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DKORTEX_DIR=$HOME/kinova_learning/kortex/api_cpp/examples/kortex_api
cmake --build build -j
```

`KORTEX_DIR` defaults to the path above; override it if your SDK lives
elsewhere. The build fails fast with a clear message if the SDK is missing.

This binary has **no mock path**. The Kortex mock used elsewhere in this project
has no UDP transport, and low-level cyclic control requires UDP. It builds
without the arm; it does not run without the arm.

## Run

```bash
./build/low_level_feedback_loop
```

**Before running:**

1. Jog joint 7 to roughly 90°. The binary refuses to start within 15° of the
   0/360 seam — a wrap mid-run would corrupt the measurement by 360°.
2. Clear the arm's workspace. The commanded motion is ±5° on the wrist roll.
3. Keep a hand on the e-stop.

The run takes 10 s (10 000 cycles at 1 kHz), prints a summary, and writes
`sinusoid_tracking.csv`.

## Analyse

```bash
python3 analysis/fit_k.py sinusoid_tracking.csv
python3 analysis/plot_tracking.py sinusoid_tracking.csv results/tracking.png
```

## Log columns

| column | meaning |
|---|---|
| `cycle` | loop iteration index |
| `cycle_dt_us` | wall time since the previous cycle start (target 1000 µs) |
| `work_dt_us` | time inside `Refresh()` only — the UDP round-trip |
| `q_ideal_deg` | closed-form reference. Log-only, never fed back |
| `q_send_deg` | integrator state actually commanded |
| `q_meas_deg` | measured joint position |
| `error_deg` | `q_ideal − q_meas` — did the joint go where it was asked? |
| `gap_deg` | `q_send − q_meas` — what the arm's internal servo acts on |
| `drift_deg` | `q_send − q_ideal` — forward-Euler integration drift |

`gap` and `error` are nearly equal under the current scheme, and that is the
intended behaviour rather than a redundancy: their near-equality is the signature
of a correctly integrating command, and their divergence is what exposed the
earlier defect. `drift` is their difference, made explicit.

## Results

Measured on hardware, joint 7, 5.0° amplitude, 0.2 Hz, 10 000 cycles.
Measured-anchor: one run. Commanded-anchor: three consecutive runs.

| quantity | measured-anchor | commanded-anchor (n = 3) |
|---|---|---|
| tracked amplitude | 0.174° | 5.003 – 5.004° |
| amplitude leak | 96.5 % | below 0.1 %, within estimator noise |
| RMS tracking error | — | 0.0795 – 0.0800° |
| max tracking error | — | 0.644 – 0.655°, entirely in the startup transient |
| servo gain K | — | 63.4 – 64.6 /s  (τ ≈ 15.7 ms) |

**Startup transient.** The joint does not respond for the first ~104 cycles
(~104 ms) after the loop begins, while the commanded position integrates away
from it. This accounts for the entire max-error figure above and is reproducible
to ±1 cycle across runs. The cause is **unverified** — dead time and breakaway
friction are both consistent with the data and are distinguished by an experiment
not yet run. Higher layers must treat the first ~150 ms as unusable.

**Estimator note.** Fitting K over all cycles rather than excluding the transient
returns 48.9 /s — repeatable to 0.2 % across runs, and wrong by 30 %. The binary
prints both fits for this reason.

Loop timing: 1000.0 µs mean cycle; 0.3 – 3.0 % of cycles over 1.1 ms; worst case
~4.7 ms, located in the `Refresh()` UDP round-trip rather than in local compute.
Adequate for this measurement; **not** yet adequate for control computation
inside the loop.

## Known limitations

- Single joint, single arm, single frequency. Nothing here is established for the
  other six joints, which are not all the same actuator class.
- The servo model `v = K·gap` is inferred from these experiments, not taken from
  Kortex documentation. It predicted both the failure and the fix and is
  repeatable to 1.9 %, but it is not the servo's actual control law and has not
  been tested away from 0.2 Hz.
- The cause of the ~104 ms startup transient is unverified.
- No real-time scheduling. `SCHED_FIFO` and CPU pinning are not applied; the
  outlier cycles above are the direct consequence. This is a prerequisite for
  putting IK inside the loop.
- Whether low-level position mode expects a velocity feedforward field alongside
  position is unverified against the Kortex documentation.
- Kortex's `position()` wrapping convention is unverified; the run is kept away
  from the seam rather than handling wrap correctly.
- Joint 7's actuator class (and therefore its firmware velocity-error ceiling) is
  unverified.
