# Design — 1 kHz joint-velocity command primitive

## Problem

The Kinova Gen3's low-level cyclic interface (`BaseCyclic::Refresh`) accepts
joint **positions**. Every actuation path in the wider project produces a
**velocity**: differential IK maps a Cartesian twist to joint rates, and the
admittance reflex converts a measured wrench to a Cartesian velocity correction.
Something must bridge velocity to position at 1 kHz, and that bridge is
load-bearing for everything above it. If it is wrong, a correct IK solver and a
correct admittance law will still produce an arm that barely moves — and the
debugging effort will be spent on the wrong layer.

This document records how that bridge was first built wrong, why, and what
replaced it.

## Interface

Per cycle, at 1 kHz:

| symbol | side | units | frame | description |
|---|---|---|---|---|
| `q_meas` | observation | deg | joint | measured actuator position from `Refresh()` |
| `q_send` | action | deg | joint | commanded destination passed to `set_position()` |
| `q̇_des` | input | deg/s | joint | demanded joint velocity |
| `q_ideal` | log-only | deg | joint | closed-form reference; never enters the loop |

The arm's internal joint servo is not directly observable. Its behaviour is
modelled — see *Servo model*, and the caveat attached to it.

## First implementation: measured-anchor integration

```
q_send[k] = q_meas[k] + clamp(q̇_des(t)·dt, ±MAX_STEP)
```

**Rationale at the time — contact safety.** If the arm is physically blocked,
`q_meas` stops advancing, so `q_send` stops advancing with it. No position error
can accumulate against the obstacle, and therefore nothing discharges violently
when the obstruction is removed. This is integrator-windup immunity obtained
structurally, without a monitor.

**Result on hardware.** Commanding 5.0° at 0.2 Hz on joint 7 for 10 000 cycles
produced a measured amplitude of 0.1741° — the joint tracked 3.48 % of the
commanded motion, a 96.5 % amplitude leak. Phase and shape were correct; only
the magnitude was wrong. No coding defect was found.

## Diagnosis

`set_position()` takes a **destination**, not a displacement. The joint servo
converts the difference `(q_send − q_meas)` into motion. Substituting the
measured-anchor command:

```
gap = q_send[k] − q_meas[k] = q̇_des(t)·dt        identically, every cycle
```

`q_meas` cancels. The gap is the same value whether the joint is exactly on
trajectory or 5° behind it. **It is an increment, not an error** — there is no
reference for it to be an error against, because `q_ideal` was never
accumulated anywhere in the loop. Consequently no correction mechanism exists,
and the realised velocity is a fixed fraction `K·dt` of the demanded velocity,
independent of how far behind the joint has fallen.

Restated: each cycle the servo is handed a gap and closes a fraction of it
within 1 ms. Re-anchoring to the measurement deletes the unclosed remainder
before the next cycle. The scheme discards the servo's accumulated residual —
it bought windup immunity by removing the actuation itself.

## Second implementation: commanded-anchor integration

```
q_send[0] = q_hold
q_send[k] = q_send[k−1] + clamp(q̇_des(t)·dt, ±MAX_STEP)
```

Summing the reference velocity from the reference start position reconstructs
the reference trajectory:

```
q_send[k] = q_hold + Σ q̇_des·dt  ≈  q_hold + ∫q̇_des dt  =  q_ideal[k]
```

So `q_send` *is* the desired position, and `(q_send − q_meas)` is a true
tracking error which the arm's internal servo closes. The host integrator is a
**reference generator**; the controller lives in the arm. The unclosed residual
now persists in `q_send`, so the gap grows until the resulting velocity equals
the demanded velocity, and tracking closes.

**This is not a return to a rejected design.** The windup concern was real; what
was wrong was the belief that suppressing the gap was the only way to address
it. The safety property is now explicit rather than structural.

## Windup safety

A monitor aborts the loop when `|q_send − q_meas| > GAP_ABORT_DEG` (1.5°),
covering physical blocks, joint limits, and actuator faults.

**Sizing constraint.** The threshold must exceed every gap that occurs in normal
operation. Setting it lower turns the monitor into a rate limiter, capping
velocity at `K·threshold` and reproducing the original leak in a new form.

Two lower bounds apply, and the second is the binding one:

| bound | value | source |
|---|---|---|
| steady-state gap `q̇_max/K` | 0.099° | 6.28 deg/s ÷ 63.8 /s |
| startup transient peak gap | 0.66° | measured, n = 3 |

The startup transient exceeds the steady-state gap by 6.7×, so sizing the
monitor from the steady-state gap alone would abort every run before the joint
ever moved. 1.5° clears the transient by 2.3× and the steady-state gap by 15×,
and still catches a genuine stall within about 0.24 s of demanded travel.

`MAX_STEP_DEG` (0.05°) is retained but bounds the per-cycle **increment**, not
the gap. Peak legitimate increment is 0.00628°, so it never fires during normal
motion; it exists to bound a single bad velocity sample.

## Servo model

The gap and the demanded velocity are related by a through-origin fit:

```
q̇_des = K · gap        K = Σ(q̇·gap) / Σ(gap²)
```

**Measured: K = 63.8 /s, spread 1.9 % over three consecutive runs**
(63.40, 64.61, 63.44). Equivalently τ = 1/K = **15.7 ms**, first-order corner at
**10.2 Hz**.

Two estimator choices are load-bearing, and both were arrived at by getting them
wrong first.

**Least squares, not a peak ratio.** A peak-based estimate `q̇_peak/|gap|_peak`
divides by a single sample. Under non-real-time scheduling the largest gap is a
stall artefact rather than a steady-state value, and on the first commanded-anchor
run this biased `K` low by roughly a factor of six.

**The startup transient is excluded from the fit.** The first ~104 cycles pair
normal demanded velocities with gaps of up to 0.66° — points lying nowhere near
the true line. Including them dragged the fitted slope from 63.8 down to 48.9 /s,
a 30 % error, reproducibly in all three runs:

| run | K, all cycles | K, skip 300 |
|---|---|---|
| 1 | 48.90 | 63.40 |
| 2 | 48.81 | 64.61 |
| 3 | 49.04 | 63.44 |

The generalisable lesson: **least squares gives no warning when part of the data
does not obey the model.** It silently averages the offending points in and
returns a plausible number. The all-cycles fit was self-consistent, repeatable to
0.2 %, and wrong by 30 %. Repeatability is not correctness. The binary therefore
prints both fits every run, so that a change in the transient is visible rather
than absorbed into the headline figure.

**Caveat.** `v = K·gap` is inferred from these experiments, not read from Kortex
documentation. It predicted the measured-anchor failure, predicted the fix, and
is repeatable to 1.9 %, but it is not the servo's actual control law and the fit
covers a single joint at a single frequency. A frequency sweep would test whether
the first-order form holds away from 0.2 Hz; that is not yet done.

**Prior discrepancy, resolved.** An earlier revision of this document recorded
that the two implementations disagreed on `K` by a factor of ~2 (≈35 vs ≈60) and
treated this as an open problem. It was not a discrepancy. The ~35 figure inverts
the measured-anchor leak under the assumption `gap ≡ q̇·dt`; it is a consistency
check on the failure mode, not a measurement of the servo. The two numbers were
never measuring the same quantity.

## Startup transient

For the first ~104 cycles of every run the joint does not move at all while
`q_send` integrates away from it:

```
cycle    0    q_meas − q₀ = +0.0000    q_send − q₀ = +0.0063
cycle   50    q_meas − q₀ = +0.0000    q_send − q₀ = +0.3203
cycle  100    q_meas − q₀ = +0.0000    q_send − q₀ = +0.6330
cycle  110    q_meas − q₀ = +0.2006    q_send − q₀ = +0.6953
cycle  300    q_meas − q₀ = +1.7669    q_send − q₀ = +1.8467
```

This is the sole cause of the worst-case tracking error, which occurs at cycle
103–105 in all three runs (0.644°, 0.649°, 0.655°) with `cycle_dt_us` ≈ 1005 µs.
It is **not** a scheduler stall: the cycle timing on those samples is nominal, and
the reproducibility to ±1 cycle across independent runs rules out a random event.

**Cause: fixed dead time (measured).** A frequency sweep at constant amplitude
discriminates the two hypotheses. Dead time predicts breakaway at a fixed *time*;
breakaway friction predicts a fixed *gap*. The gap builds as `∫q̇·dt`, so the two
predictions diverge by the frequency ratio:

| f (Hz) | peak vel (deg/s) | breakaway cycle | gap at breakaway | dead-time predicts | friction predicts |
|---|---|---|---|---|---|
| 0.1 | 3.14 | 105 | 0.305° | ~105 | ~210 |
| 0.2 | 6.28 | 106 | 0.627° | ~105 | ~105 |
| 0.4 | 12.57 | 106 | 1.284° | ~105 | ~53 |

Breakaway cycle is constant at 105–106 across a 4× velocity range; the friction
hypothesis predicted 53 and 210 and is rejected. The gap at breakaway instead
scales with velocity (ratio 2.05× per doubling), which is exactly `∫q̇·dt` over a
fixed ~105 ms window — the same fact seen from the other side.

**Conclusion: ~105 ms of dead time between entering `LOW_LEVEL_SERVOING` and the
joint responding to position commands, independent of commanded velocity.**

The mechanism is not localised further. Fixed dead time is consistent with
servo-loop transport delay, servoing-mode engagement latency, or command-buffer
fill; the sweep distinguishes "fixed time" from "fixed gap" but not among these.
No Kortex documentation of a settling interval has been located. The claim is the
measured behaviour, not its internal cause.

**Consequence for layers above.** Until this is characterised, the first ~150 ms
after entering low-level servoing must be treated as unusable for control. Any
higher layer must ramp in from rest rather than assuming the arm responds to the
first command it receives.

## Results

Joint 7, 5.0° amplitude, 0.2 Hz, 10 000 cycles. Measured-anchor: single run.
Commanded-anchor: **three consecutive runs**, spread shown.

| quantity | measured-anchor | commanded-anchor (n = 3) |
|---|---|---|
| tracked amplitude | 0.1741° | 5.003 – 5.004° |
| amplitude leak | 96.52 % | −0.05 % to −0.08 % |
| RMS tracking error | — | 0.0795 – 0.0800° |
| max tracking error | — | 0.644 – 0.655° (all in the startup transient) |
| K (skip 300) | — | 63.40 – 64.61 /s |

The maximum error is reported separately from the RMS deliberately. It is
entirely a startup-transient artefact; quoting it as a steady-state figure would
misrepresent the controller by roughly 8×, and omitting it would hide a real
100 ms window in which the arm does not respond.

The negative leak is not overshoot. Amplitude is estimated as half the
peak-to-peak of `q_meas`, and `max` and `min` each capture an encoder-noise
excursion, biasing the estimate slightly high. The consistent sign across three
runs identifies it as estimator bias rather than a physical effect: the honest
statement is **leak below 0.1 %, at the noise floor of the estimator**, not
"leak = −0.06 %".

Amplitude is essentially unaffected by the residual lag, which is expected: a
first-order lag attenuates amplitude by `1/√(1+(ω/K)²)`, which at ω = 1.26 rad/s
and K = 63.8 /s is 0.02 % — the gap costs phase, not magnitude. The predicted
steady-state gap is `q̇_peak/K` = 0.099°, RMS 0.070°. This is precisely why the
measured-anchor failure was a *leak* rather than a lag: there the gap was pinned,
so the velocity was pinned, and the swing genuinely never developed.

## Instrumentation

Three per-cycle quantities are logged, and they are distinct:

- `gap = q_send − q_meas` — the servo's input. Expected non-zero; it is what
  produces motion.
- `error = q_ideal − q_meas` — the tracking question.
- `drift = q_send − q_ideal` — forward-Euler integration error, measured at
  ~0.004° over 10 s.

Under commanded-anchor `gap ≈ error`, since `q_send ≈ q_ideal`. That near-equality
is the signature of a correctly integrating command. Under measured-anchor they
diverged completely — gap pinned at 0.006° while error swung to several degrees —
and that divergence in a single column is what makes the defect visible. Both are
retained for this reason, with `drift` logging their difference explicitly.

`q_ideal` is computed from the closed form and never derived from `q_send`.
Logging `q_send` against `q_meas` alone would have been circular under the
measured-anchor scheme, since the command was a function of the measurement, and
could not have revealed the leak.

## Timing

Across the three commanded-anchor runs: mean cycle 1000.0 µs; cycles over 1.1 ms
**2.24 %, 2.95 %, 2.37 %**; worst case ~4.7 ms. `Refresh()` round-trip: mean
~383 µs, worst case ~4.7 ms — the outliers are in the network call, not in local
compute, and the cycle-level and work-level outliers are the same events.

The miss rate is quoted as a range, not a figure: it varied by 30 % across three
identical back-to-back runs, and an earlier session recorded 1.39 %. It is not a
stable property of the system.

This is adequate for measuring an amplitude leak and inadequate for closed-loop
control with computation in the loop. Real-time scheduling was previously
deferred on the evidence available at the time (0.07 % miss rate); that evidence
has changed. `SCHED_FIFO` with CPU pinning is a prerequisite for adding
differential IK to this loop.

Worth recording precisely because it was the wrong guess: the worst tracking
error was expected to coincide with the worst timing outlier. It does not. The
two are unrelated, and the check that established this cost one query against
data already on disk.

## Open questions

1. **Mechanism of the ~105 ms startup dead time.** Established as fixed dead
   time (not friction) by the frequency sweep above. Whether it is servo-loop
   transport delay, mode-engagement latency, or buffer fill is not resolved, and
   no Kortex documentation of a settling interval has been found. Lower priority
   now that its *behaviour* is characterised: higher layers ramp in from rest
   regardless of the internal cause.
2. Does low-level position mode expect a velocity feedforward field alongside
   position? Unverified against Kortex documentation.
3. Kortex `position()` wrapping convention — unverified. Currently handled by
   refusing to run near the 0/360 seam rather than by handling wrap.
4. Joint 7's actuator class, and therefore its firmware velocity-error ceiling.
   `MAX_STEP_DEG` is sized against the lower of the two published limits as
   insurance.
5. Whether the first-order model holds away from 0.2 Hz. A frequency sweep to
   ~8 Hz would test it against the predicted 10.2 Hz corner.
6. Single joint, single arm. Nothing here is established for the other six
   joints, which are not all the same actuator class.

## Next

Cartesian differential IK (damped least squares) inside this loop, producing
`q̇_des` from a task-frame twist, with the integration primitive established
here unchanged beneath it.
