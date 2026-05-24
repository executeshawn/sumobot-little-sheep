# Control-System Analysis — PID Design, Sensor Filtering, and Firmware Mapping

**Companion document to** [`SFG_Report.md`](SFG_Report.md).
**Status disclaimer:** the Heatseeker is presently under integration and
testing. No PID gains, plant parameters, or filter coefficients quoted
below are experimentally measured. Numerical guidance is restricted to
*orders of magnitude* derived from component datasheets, and every
section that anticipates a measured result is labelled accordingly.

---

## 1. Scope

This document covers three topics that complement the SFG report:

1. The **PID-tuning methodology** that will be applied once the
   inner-loop sensor is in place.
2. The **sensor-filtering architecture**: continuous ToF ranging, cached
   polling, priority-based edge detection, asynchronous timing, and a
   discussion of forward-compatible noise mitigation (moving average /
   Kalman).
3. The **mapping** between the SFG model and the actual ESP32-S3
   firmware modules.

---

## 2. PID Tuning Discussion (theoretical)

### 2.1 Controller form

The firmware reserves the standard parallel-form PID:

$$
u(t) = K_p\,e(t) + K_i \int_{0}^{t} e(\tau)\,d\tau + K_d\,\frac{de(t)}{dt}
$$

Discrete-time implementation (planned for `pid.cpp`) uses the velocity
form to avoid integral-wind-up during state transitions:

$$
\Delta u_k = K_p\,(e_k - e_{k-1})
           + K_i\,T_s\,e_k
           + \frac{K_d}{T_s}\,(e_k - 2e_{k-1} + e_{k-2})
$$

with `T_s = FSM_UPDATE_MS = 5 ms` taken from `config.h`. The discrete
sample period is short relative to the assumed motor time constant
`τ_m ≈ 50 ms`, so the bilinear / Tustin approximation will be acceptable
for tuning purposes.

### 2.2 Role of each term

* **Proportional `K_p`.** Provides immediate corrective torque
  proportional to bearing error. Increasing `K_p` raises bandwidth and
  reduces rise time but, in the third-order plant of §6 of the SFG
  report, will eventually push complex poles toward the imaginary axis
  and cause oscillation.
* **Integral `K_i`.** Eliminates steady-state error against a constant
  disturbance — the case of greatest interest for sumo, where the
  opponent applies a sustained push. The cost is added phase lag and an
  increased risk of overshoot.
* **Derivative `K_d`.** Damps the oscillation introduced by aggressive
  `K_p`. Because the ToF bearing signal is quantised and noisy,
  derivative action must be implemented either on the *measurement* with
  a first-order low-pass pre-filter, or on the *error* with the
  setpoint-weighting trick `Kd · d(measurement)/dt`. Both are
  implementation choices for the tuning phase.

### 2.3 Intended tuning workflow

The workflow is staged so that each step is checkable against the SFG
model before proceeding:

1. **Inner-sensor installation.** Add either wheel encoders or an IMU
   (BNO055 / MPU-6050). Until then, no measurable closed-loop response
   exists.
2. **Open-loop step identification.** Command a step in PWM and log
   the resulting wheel velocity (or yaw rate). Fit a first-order model
   `Gm(s) = Km/(τm s + 1)` by reading the 63.2 % rise time and the
   steady-state gain. This replaces the assumed datasheet values in the
   SFG report with measured ones.
3. **Manual / Ziegler–Nichols closed-loop tuning.** With `K_i = K_d = 0`,
   raise `K_p` until sustained oscillation begins (`K_u`) and measure
   the oscillation period `T_u`. Apply the classical ZN table for the
   PID variant, then move to the *no-overshoot* ZN row if oscillation
   is observed in the sumo manoeuvre.
4. **Verification against the analytic transfer function.** Substitute
   the measured `Km, τm` and the tuned `Kp, Ki, Kd` into the
   characteristic polynomial from SFG §6 and confirm closed-loop pole
   locations via Routh–Hurwitz or a Bode plot. This step is where the
   SFG report becomes a tool, not an exercise.
5. **Disturbance test.** Apply a manual lateral push and confirm that
   the bot returns to its commanded bearing without sustained offset
   (verifies the disturbance rejection property of §7 of the SFG
   report).

### 2.4 Stability and anti-oscillation goals

Concrete numerical targets that will be checked once tuning data exists:

* **Phase margin** ≥ 45° on the loop gain `Gc(jω)Kdrv Gm(jω)Gr(jω)`.
* **Gain margin** ≥ 6 dB.
* **Closed-loop overshoot** ≤ 15 % to a step heading command. The sumo
  context tolerates some overshoot (the bot can re-attack), but excessive
  ringing during ATTACK causes the wheels to lose traction.
* **Integral anti-wind-up** clamp the integrator to the PWM range
  (-255..+255 in units of duty) and freeze it when the PWM is saturated;
  this is standard practice for actuator-limited controllers and avoids
  the post-saturation overshoot characteristic of naive PID.
* **Derivative filter** first-order low-pass at ~30 Hz to keep `Kd`
  from amplifying ToF measurement noise.

> All numerical figures in this section are *targets*. They will be
> validated empirically and reported in a tuning addendum to this
> document.

---

## 3. Sensor Filtering Discussion

### 3.1 VL53L0X continuous ranging

The firmware drives each VL53L0X in continuous-ranging mode with two
inter-measurement periods:

* Front group (FL / FC / FR): 25 ms — engagement-critical.
* Side / rear group (L / R / RR): 70 ms — flank awareness only.

`pollFront()` and `pollSide()` are *non-blocking harvesters*: they call
`isRangeComplete()`; if the chip is still ranging, the previous cached
value is left in place. The FSM only ever reads the cache, so the inner
loop is never blocked on an I²C round-trip.

This replaces the previous single-shot `rangingTest()` topology in which
the loop could block for the sum of six measurement budgets (≈ 200 ms),
which is the same order of magnitude as the time required to drive off
the dohyo at top speed — an unacceptable blind interval.

### 3.2 Cached polling and asynchronous timing

The system runs four asynchronous timers in `main.cpp`:

| Timer | Period | Cadence label |
|---|---|---|
| `Edge::poll()` | 8 ms | FAST (priority) |
| `ToF::pollFront()` | 30 ms | MEDIUM |
| `ToF::pollSide()` | 90 ms | SLOW |
| `Strategy::update()` | 5 ms | FSM tick (reads caches only) |

The asynchronous structure decouples acquisition rate from consumption
rate: the FSM ticks faster than any sensor refreshes, so when it queries
the cache it always gets the freshest available value, but it never
waits. From a control-systems perspective this is a zero-order hold on
each sensor channel; the SFG report deliberately lumps these into the
reference path because each sample period is small relative to the
plant's dominant time constant.

### 3.3 Priority-based edge detection

The edge sensor is treated as a **priority interrupt** rather than a
proportional feedback signal. Three architectural choices implement
this:

1. The QTR is polled fastest (8 ms).
2. The cache it writes is read **first** by the FSM in `SEARCH`,
   `ATTACK`, and `EVADE` states, *before* any opponent logic.
3. Edge detection unconditionally transitions to `EVADE`, overriding
   any in-progress attack command.

The justification is rule-based: a sumobot that drives off the dohyo
loses the round regardless of whether it was hitting the opponent at
the time. Edge survival therefore strictly dominates opponent pursuit.

### 3.4 Noise mitigation, current

The present firmware applies three implicit noise-mitigation steps:

* **Threshold hysteresis (implicit).** The ToF detection function picks
  the *closest* sensor below its threshold. Marginal readings that
  hover near the threshold do not cause direction flapping because the
  selected bearing requires a clearly-shorter distance than competing
  channels.
* **Calibrated edge readings.** The QTR is calibrated during the
  `CALIBRATE` state by a slow rotation that sweeps the array over both
  mat and ring, so the threshold operates on a normalised 0..1000 range
  rather than raw timing.
* **Out-of-range sentinel.** ToF status code 4 maps to
  `TOF_OUT_OF_RANGE = 9999 mm`, which is well above every threshold and
  therefore cannot falsely trigger a `detectOpponent()` decision.

### 3.5 Noise mitigation, planned

Two upgrade paths are explicitly anticipated:

* **Moving-average filter on bearing.** Apply an N-tap rolling mean
  (N = 3..5) over the front ToF distances before threshold comparison.
  This is the lowest-cost upgrade and addresses ToF photon-shot
  jitter; it costs `N × sizeof(uint16_t)` of RAM per sensor.
* **Kalman filter on opponent bearing.** With six ToF channels arranged
  in a ring, the bearing can be modelled as a 1-D state with a constant
  + Brownian-motion velocity model. A scalar Kalman filter fuses the
  six measurements into a single estimate of opponent bearing with an
  associated covariance, which feeds the controller reference `R(s)`.
  This is the natural successor to the moving average once tuning data
  identifies bearing noise as the dominant performance limit.

Both filters operate on the *reference* path (outer loop) and therefore
do not change the SFG of the inner loop; their effect is to soften the
step character of `R(s)` and reduce derivative-term excitation.

---

## 4. Mapping the SFG onto the ESP32-S3 Firmware

The SFG of [`SFG_Report.md`](SFG_Report.md) is realised in firmware as
follows. The table is the closest thing this report has to a
"code-vs-theory" cross-reference.

### 4.1 Element-by-element mapping

| SFG element | Firmware artefact | Status |
|---|---|---|
| Reference `R(s)` | `Strategy::update()` in [`src/strategy.cpp`](../src/strategy.cpp); `driveTowards()` maps an `OpponentDir` to a differential-drive command | ✅ implemented |
| Summing junction `R − H·Y` | Implicit in the planned PID; today the FSM issues commands without a measured `Y` | 🟡 partial |
| Controller `Gc(s)` | Provisioned `pid.cpp` module (not yet present in tree). Today the FSM issues fixed-PWM commands via `Motors::forward/turnLeft/turnRight` | 🟡 design only |
| Driver gain `K_drv` | Direct LEDC PWM at 5 kHz / 8-bit + `digitalWrite()` on IN1/IN2/STBY ([`src/motors.cpp`](../src/motors.cpp)) | ✅ implemented |
| Motor dynamics `Gm(s)` | Physical N20 + gearbox | ✅ hardware |
| Kinematic integrator `Gr(s)` | Robot body (yaw rate → heading) | ✅ hardware |
| Feedback path `H` | Not yet closed — no encoder/IMU. ToF bearing acts as a slow proxy via the supervisory layer | ❌ open |
| Disturbance `D(s)` | Opponent contact, slip; handled non-linearly by EVADE override | ✅ supervisory only |

### 4.2 FSM as supervisory logic

Mason's formula in the SFG report assumes a *single* linear closed loop.
The FSM does not appear in that derivation because it operates on a
slower, discrete time-scale: it sets the *reference* `R(s)` and the
*structure* (which state's behaviour applies), then hands off to the
linear inner loop for execution. In control-theory language the FSM is
a **supervisory mode-switching controller**; classical SFG analysis
applies *within* each mode.

### 4.3 PID-loop interaction (planned)

Once `pid.cpp` is populated, the FSM will hand a single reference
(target yaw rate or target velocity) to the PID each tick, and the PID
will produce a left/right PWM pair via `Motors::drive()`. The FSM keeps
its present responsibility for **switching** references on transitions
(e.g. EVADE flips the reference sign).

### 4.4 Motor actuation

PWM via ESP32 LEDC on channels 0 and 1, 5 kHz, 8-bit, sharing one timer
in the low-speed group. The TB6612FNG STBY pin is brought high in
`Motors::begin()` and pulled low again in `Motors::stop()` so that the
STOP state of the FSM coasts the motors and removes their holding
current.

### 4.5 Feedback acquisition

In the current revision, all *measurable* feedback comes through the
sensor cache (ToF bearing + edge). The SFG's inner unity-feedback path
`H = 1` is therefore currently realised at a very coarse, supervisory
granularity rather than the fine-grained continuous form Mason's
formula assumes. Closing that gap is, again, the entire purpose of the
encoder/IMU upgrade in the future-work list.

### 4.6 Disturbance correction

Two paths, exactly mirroring SFG §7:

* **Linear (after tuning):** PID integrator nulls DC disturbance.
* **Non-linear (active now):** EVADE state pre-empts the inner loop
  when the edge cache reports a tripped sensor.

### 4.7 Closed-loop behaviour summary

A truthful statement of the present state:

> The Heatseeker firmware implements the structural skeleton of the
> control system depicted in the SFG. The outer (supervisory) loop is
> exercised on hardware; the inner (continuous) loop is wired and ready
> to receive `Gc(s)` from a tuned PID, but currently runs open-loop
> with hand-picked PWM constants chosen for benchtop debugging.
> Substituting a tuned PID and a measured inner-loop sensor is the
> single remaining step that elevates the implementation from
> "structurally complete" to "control-theoretically closed."

---

## 5. Open Issues Carried Into the Tuning Phase

1. **Inner-loop sensor selection.** Encoder vs. IMU trade-off — encoders
   give wheel velocity (closest to the SFG's `ω` node); an IMU gives
   robot yaw rate (closest to `Y`). Decision pending.
2. **Anti-wind-up clamp value.** Choose between PWM-saturation clamping
   and conditional integration; both are common and the choice should be
   driven by observed wind-up behaviour during ZN tuning.
3. **Derivative filter cut-off.** Initial guess 30 Hz, refine once ToF
   noise spectrum is characterised.
4. **ToF Kalman vs. moving average.** Defer until empirical bearing
   noise is measured.
5. **Match-rule edge cases.** What happens if both bots collide head-on
   and the QTR sees no edge for several seconds? Currently the ATTACK
   10-second watchdog reverts to SEARCH; this needs match-rule review.

---

## 6. References

(See also `SFG_Report.md` §11.)

1. K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and
   Tuning*, 2nd ed., ISA, 1995.
2. J. G. Ziegler and N. B. Nichols, "Optimum settings for automatic
   controllers," *Trans. ASME*, vol. 64, pp. 759–768, 1942.
3. R. E. Kalman, "A new approach to linear filtering and prediction
   problems," *J. Basic Engineering*, vol. 82, pp. 35–45, 1960.
4. STMicroelectronics, *VL53L0X API user manual*, UM2039.
5. Pololu Corp., *QTR Reflectance Sensors User's Guide*.
