# Control-System Analysis — LittleSheep Sumobot

**Companion document to** [`SFG_Report.md`](SFG_Report.md).
**Status disclaimer.** The LittleSheep as built is a
**deterministic FSM + reactive sensor logic** topology. We do not claim
a closed-loop PID controller is active. This document covers:

1. What the current implementation actually does (FSM + reaction
   logic), assessed against the standard control-system rubric
   (closed loop, sensor filtering, disturbance rejection, stability).
2. What the system is **PID-ready** for (the structural skeleton is in
   place; the missing pieces are explicitly enumerated).
3. The PID design methodology that will apply *once* the missing
   pieces are added (inner-loop sensor + `pid.cpp`).

No PID gains, plant parameters, or filter coefficients in this
document are experimentally measured. Numerical guidance is restricted
to orders-of-magnitude from component datasheets and is labelled as
such.

---

## 1. Rubric Self-Assessment (honest)

| Rubric item | Implemented today? | Notes |
|---|---|---|
| Autonomous operation | ✅ Yes | The FSM runs unattended after the 5 s pre-match countdown; no operator-in-the-loop. |
| Closed-loop control concept | 🟡 Partial | The **outer supervisory loop** is closed: ToF bearing → FSM → motor command, then re-sample. The **inner continuous loop** (heading / wheel velocity) is **not** closed — no encoder, no IMU, no PID. |
| Disturbance rejection | 🟡 Partial | **Non-linear** rejection is active: EVADE pre-empts every other state on any edge event. **Linear** rejection of a constant push is not present (would require the PID integrator). |
| Sensor filtering | 🟡 Partial | Implicit filtering via (a) threshold hysteresis on the "closest-below-threshold" opponent selector, (b) calibrated edge readings normalised to 0..1000, and (c) the multi-rate cache that the FSM reads. **No** moving-average or Kalman filter yet. |
| Multi-rate / non-blocking architecture | ✅ Yes | Edge 8 ms, front ToF 30 ms, side/rear ToF 90 ms, FSM 5 ms. FSM only reads caches; sensor I/O never blocks the FSM tick. |
| Finite-state-machine stability | ✅ Yes | Seven well-defined states with deterministic transitions; EVADE pre-emption; ATTACK 10 s watchdog; STOP latched on fault. |
| Mathematical control concepts | ✅ Documented | SFG, Mason's gain formula, characteristic polynomial, Routh–Hurwitz outlook, disturbance transfer function in [`SFG_Report.md`](SFG_Report.md). |
| PID-readiness | 🟡 Architecturally yes | Deterministic 5 ms tick, signed differential `Motors::drive()`, `Movement::` calibration layer in place. **Missing:** an inner-loop sensor (encoder or IMU) and a `pid.cpp` consumer. |
| Tuned PID controller | ❌ No | Not implemented. Symbolic `Kp, Ki, Kd` are presented in the SFG report; no numerical tuned values are claimed. |
| Closed-loop PID heading / velocity control | ❌ No | Provisioned, not realised. |

This is the assessment that should drive every claim made during the
oral defense.

---

## 2. What the Current Implementation Actually Does

### 2.1 The supervisory FSM (closed at the *behaviour* layer)

States: `IDLE → CALIBRATE → COUNTDOWN → SEARCH → ATTACK → EVADE → STOP`
(implemented in
[`lib/Sumobot/strategy.cpp`](../lib/Sumobot/strategy.cpp)).

* **SEARCH** alternates deterministic left/right pivots at
  `SPEED_SEARCH` every `SWEEP_SEGMENT_MS = 600 ms`; on the next FSM
  tick that sees any ToF channel under threshold, it transitions to
  ATTACK.
* **ATTACK** maps the chosen `OpponentDir` onto a differential-drive
  command at `SPEED_ATTACK`. Inner-wheel speed = 3/5 of outer for the
  veering `FRONT_LEFT` / `FRONT_RIGHT` cases; pure pivots for
  `LEFT` / `RIGHT` / `REAR`. The 10 s watchdog reverts to SEARCH if the
  FSM stays in ATTACK without leaving.
* **EVADE** — fixed REVERSE phase (`EVADE_REVERSE_MS = 350 ms`) at
  `SPEED_TURN`, then a pivot phase (`EVADE_TURN_MS = 450 ms`) away
  from the tripped edge. If still on edge after the pivot, repeat;
  otherwise return to SEARCH.

There is no DIP-switch mode selection. Behaviour is fixed in firmware.

### 2.2 Open-loop motor command path

Strategy → `Motors::drive(leftSpeed, rightSpeed)` → `applyMotor()`
→ LEDC PWM at 5 kHz / 8-bit + IN1/IN2 polarity. The polarity contract
is verified at the bench and recorded once in
[`motors.cpp`](../lib/Sumobot/motors.cpp).

No feedback signal is read between the FSM's command and the next FSM
tick. From a control-theory standpoint this is **open-loop** at the
inner (continuous-dynamics) layer; the *only* feedback path that
exists is the slow event-driven re-sampling of the ToF ring at 30 ms,
which the FSM converts back into a new command — i.e. a coarse,
event-driven outer loop.

### 2.3 Reactive sensor cache

[`main.cpp`](../src/main.cpp) runs four cooperative timers. The FSM
tick reads `Edge::isOnEdge()` and `Edge::edgeDirection()` first in
every active state, *before* any ToF logic — this realises the
edge-priority rule. ToF reads are non-blocking from the FSM's
perspective because the cache is what the FSM consumes; the harvest
itself may briefly block up to `TOF_READ_TIMEOUT_MS = 50 ms` on a
silent sensor, which is *off the FSM-tick critical path* and therefore
acceptable.

---

## 3. PID-Readiness — What Is Already in Place

These structural decisions mean a PID can be slotted in without
re-architecting the firmware.

1. **Deterministic FSM tick `T_s = 5 ms`.** Short relative to the
   assumed motor time constant `τ_m ≈ 100 ms`, so bilinear / Tustin
   discretisation will be accurate.
2. **Signed differential drive primitive.**
   `Motors::drive(leftSpeed, rightSpeed)` accepts signed PWM in
   −255..255 and applies a single polarity convention via
   `applyMotor()`. A PID can publish its left/right pair directly to
   this primitive.
3. **Soft and hard speed caps.** `setSpeedCap()` (per-tick) and
   `setHardCap()` (set once, used as a persistent ceiling by bench
   tests) compose with `min()` and clamp every command. This is also a
   natural location to hang **anti-windup saturation feedback**.
4. **`Movement` calibration abstraction.** Sits between Strategy and
   Motors. A future PID's output can pass through `Movement` so the
   measured per-channel trim is applied to the controller's output
   without modifying `motors.cpp`.
5. **Cached, non-blocking sensor harvest.** A PID that runs on the FSM
   tick can read the most recent measurement without paying I/O
   latency.

What is **missing**:

* **An inner-loop sensor.** Encoder (closest to the SFG's `ω` node) or
  IMU (closest to `Y`). Choice is open; both are forward-compatible.
* **`pid.cpp`.** A discrete velocity-form PID with anti-wind-up and a
  low-pass on the derivative term. Not in the tree.
* **A measured plant model.** `Km` and `τm` need to come from the
  step-response test (`MT-03` in [`Test_Plan.md`](Test_Plan.md)).

---

## 4. SFG → Firmware Element Mapping

Cross-reference: [`SFG_Report.md`](SFG_Report.md) §8.

| SFG element | Firmware artefact | Status |
|---|---|---|
| Reference `R(s)` | FSM in `strategy.cpp`; `driveTowards()` maps `OpponentDir` to a left/right command | ✅ Implemented |
| Summing junction `R − H·Y` | Implicit in the planned PID; FSM today issues commands without a measured `Y` | 🟡 PID-ready |
| Controller `Gc(s)` | Not implemented. FSM issues fixed-PWM via `Motors::forward / turnLeft / turnRight / drive`. The `Movement` layer adds open-loop trim, not closed-loop control. | ❌ Theoretical |
| Driver gain `K_drv` | TB6612FNG via direct ESP32 LEDC PWM (5 kHz / 8-bit) + `digitalWrite()` on IN1/IN2/STBY | ✅ Implemented |
| Motor dynamics `Gm(s)` | Physical TT 6 V / 160 RPM L-shape gearmotor | ✅ Hardware |
| Kinematic integrator `Gr(s)` | Robot body (yaw rate → heading) | ✅ Hardware |
| Feedback path `H` | Not closed — no encoder/IMU. ToF bearing is a slow event-driven outer-loop reference, not continuous feedback. | ❌ Open |
| Disturbance `D(s)` | Opponent contact, slip; handled non-linearly by EVADE pre-emption | ✅ Supervisory only |

---

## 5. Sensor Filtering — Current and Planned

### 5.1 What is filtering today

* **Threshold hysteresis (implicit).** `ToF::detectOpponent()` picks
  the closest sensor *strictly below* threshold. Marginal readings
  that hover near threshold do not cause direction flapping because
  the selected bearing requires a clearly shorter distance than
  competing channels.
* **Calibrated edge readings.** The QTR is calibrated during the
  CALIBRATE state (`calibrateEdge()` invoked repeatedly while
  spinning); subsequent reads are normalised 0..1000 by the
  QTRSensors library.
* **Out-of-range sentinel.** `TOF_OUT_OF_RANGE = 9999 mm` is well
  above every threshold and therefore cannot falsely trigger a
  detection on a timeout.
* **Cache architecture.** The FSM reads only cached values, so a
  one-off sensor stall cannot derail an FSM tick.

### 5.2 What is *not* filtered

There is no moving average, no Kalman filter, no PWM dead-band
compensation in the current firmware. Sensor noise that defeats
the threshold hysteresis (e.g. ToF photon-shot jitter at the edge of
the threshold band) propagates straight into the FSM's bearing
decision.

### 5.3 Planned filters (forward compatible)

* **Moving-average filter on bearing.** N-tap rolling mean
  (N = 3..5) over the front ToF channels before threshold comparison.
  Cheapest upgrade; addresses ToF photon-shot jitter. Costs
  `N × sizeof(uint16_t)` of RAM per sensor.
* **Kalman filter on opponent bearing.** Six ToF measurements + a 1-D
  state with constant-plus-Brownian velocity model produce a single
  bearing estimate with covariance, which feeds `R(s)`. Natural
  successor to the moving average if bearing noise becomes the
  dominant performance limit during tuning.

Both filters act on the **reference path** (outer loop). They do not
change the inner-loop SFG.

---

## 6. Stability and Disturbance Rejection — Current and Planned

### 6.1 Current

* **FSM stability.** Each state has bounded entry/exit conditions;
  EVADE pre-empts every other active state on any edge event;
  ATTACK has a 10 s watchdog; STOP is latched on fault. The state
  graph cannot livelock under any sensor input.
* **Disturbance rejection (non-linear).** A push that drives the bot
  onto the edge triggers EVADE before mechanical disturbance can
  remove the bot from the dohyo. This is the only disturbance
  rejection mechanism active today.
* **No linear disturbance rejection.** A constant lateral push that
  *does not* reach the edge will accumulate a heading offset because
  the FSM has no integrator on a measured heading error.

### 6.2 Planned (once PID is closed)

* **Routh–Hurwitz region.** Substituting measured `Km, τm` into the
  characteristic polynomial `τm s³ + s² + K_drv Km (Kd s² + Kp s +
  Ki)` gives an admissible `(Kp, Ki, Kd)` region.
* **Margins.** Phase margin ≥ 45°, gain margin ≥ 6 dB on
  `Gc(jω) K_drv Gm(jω) Gr(jω)`.
* **Overshoot.** ≤ 15 % to a heading step; tolerable in the sumo
  context where momentary overshoot is acceptable but sustained
  ringing loses traction.
* **Anti-wind-up.** Clamp the integrator to the PWM range
  (−255..+255 in duty units) and freeze it when the PWM is saturated.
* **Derivative filter.** First-order low-pass at ~30 Hz to keep `Kd`
  from amplifying ToF measurement noise.

All numbers in §6.2 are targets, not measurements.

---

## 7. PID Tuning Methodology (planned)

The workflow is staged so each step is checkable against the SFG
model before proceeding.

1. **Inner-sensor install.** Mount wheel encoders or an IMU. Until
   then, no measurable closed-loop response exists.
2. **Open-loop step identification (MT-03).** Command a step in PWM;
   log wheel velocity (or yaw rate). Fit `Gm(s) = Km / (τm s + 1)`
   from the 63.2 % rise time and the steady-state gain. **Replaces**
   the assumed datasheet `Km`, `τm` in [`SFG_Report.md`](SFG_Report.md).
3. **Manual / Ziegler–Nichols closed-loop tuning.** With
   `Ki = Kd = 0`, raise `Kp` until sustained oscillation (`Ku`),
   measure the period `Tu`. Apply the classical ZN table for the PID
   variant; back off to the *no-overshoot* row if oscillation is
   observed under the sumo manoeuvre.
4. **Analytical verification.** Substitute measured `Km, τm` and
   tuned `Kp, Ki, Kd` into the characteristic polynomial from
   [`SFG_Report.md`](SFG_Report.md) §6; confirm pole locations via
   Routh–Hurwitz or a Bode plot. This is where the SFG report
   becomes a tool, not an exercise.
5. **Disturbance test (DR-01..DR-05).** Apply manual lateral pushes
   of various profiles; confirm the bot returns to commanded
   bearing without sustained offset.

PID controller form (planned implementation):

$$
u(t) = K_p\,e(t) + K_i \int_{0}^{t} e(\tau)\,d\tau
     + K_d\,\frac{de(t)}{dt}
$$

Discrete velocity form (to avoid integrator wind-up across state
transitions):

$$
\Delta u_k = K_p (e_k - e_{k-1})
           + K_i T_s\, e_k
           + \frac{K_d}{T_s}(e_k - 2 e_{k-1} + e_{k-2})
$$

with `T_s = FSM_UPDATE_MS = 5 ms`.

---

## 8. Open Issues Carried into the Tuning Phase

1. **Inner-loop sensor selection.** Encoder vs. IMU — encoders give
   wheel velocity (closest to `ω`); an IMU gives robot yaw rate
   (closest to `Y`). Decision pending.
2. **Anti-wind-up clamp strategy.** PWM-saturation clamping vs.
   conditional integration; both are common; choose by observed
   behaviour during ZN tuning.
3. **Derivative filter cut-off.** Initial 30 Hz target; refine once
   ToF / IMU noise spectrum is characterised.
4. **Filter choice — moving average vs. Kalman.** Defer until
   empirical bearing noise is measured during a `test_vl53` run with
   logging.
5. **Trim and pivot-by-degree constants.** `MOVE_TRIM_LEFT/RIGHT` and
   `MOVE_MS_PER_DEG_LEFT/RIGHT` in `config.h` are placeholders. They
   are measured by running `test_calibration_protocol` and copying
   the printed config block into `config.h`. This is independent of
   the PID work and should happen first.

---

## 9. References

(See also [`SFG_Report.md`](SFG_Report.md) §11.)

1. K. J. Åström & T. Hägglund, *PID Controllers: Theory, Design, and
   Tuning*, 2nd ed., ISA, 1995.
2. J. G. Ziegler & N. B. Nichols, "Optimum settings for automatic
   controllers," *Trans. ASME*, vol. 64, pp. 759–768, 1942.
3. R. E. Kalman, "A new approach to linear filtering and prediction
   problems," *J. Basic Engineering*, vol. 82, pp. 35–45, 1960.
4. STMicroelectronics, *VL53L0X API user manual*, UM2039.
5. Pololu Corp., *QTR Reflectance Sensors User's Guide*.
