# Engineering Test Plan — Heatseeker Sumobot

**Document type:** Planned engineering validation procedures for ongoing
integration and tuning.
**Status:** Forward-looking. No section below claims that the procedure
has been completed; all acceptance thresholds are *targets* against
which future test runs will be judged.
**Companion documents:** [`SFG_Report.md`](SFG_Report.md),
[`Control_System_Analysis.md`](Control_System_Analysis.md).

---

## 1. Purpose and Scope

This plan defines the engineering tests that will be carried out as the
Heatseeker moves from bench integration to competition readiness. Each
test specifies an objective, the equipment and signals involved, the
procedure, and a numeric or behavioural acceptance criterion. The plan
covers:

* Firmware unit testing
* Sensor bring-up validation
* Motor response characterisation
* Edge-detection verification
* ToF latency / cadence verification
* PID tuning workflow
* Disturbance rejection testing
* Battery discharge / runtime characterisation

Tests are grouped roughly in the order they will be executed.

---

## 2. Unit Testing Plan (firmware)

Native (host-side) tests using PlatformIO's `[env:native]` target with
hardware-dependent modules replaced by lightweight fakes.

| Test ID | Module under test | Objective | Acceptance criterion |
|---|---|---|---|
| UT-01 | `ToF::detectOpponent()` | Returns the correct `OpponentDir` enum across all single-sensor and combined-threshold inputs | 100 % of enumerated cases match expected output |
| UT-02 | `Edge::edgeDirection()` | Direction classification for every LEFT/CENTER/RIGHT/FRONT/NONE combination of below-threshold readings | All 8 combinations classified correctly |
| UT-03 | `Strategy::update()` state transitions | Every transition in the FSM table fires on the correct input (button, edge cache, ToF cache, watchdog) | Each of the 9 documented transitions exercised at least once |
| UT-04 | ATTACK watchdog timing | `Strategy::update()` returns to SEARCH after `ATTACK_WATCHDOG_MS` of stuck-in-ATTACK simulated time | Transition occurs within ±1 FSM tick of 10 s |
| UT-05 | EVADE sub-phase sequencer | REVERSE → TURN → SEARCH (or REVERSE again if still on edge) with correct durations | Phase durations within ±1 tick of `EVADE_REVERSE_MS`/`EVADE_TURN_MS` |
| UT-06 | `Motors::setSpeedCap()` | Clamps every drive primitive to the cap regardless of input magnitude | No commanded PWM exceeds the cap in any of 100 randomised inputs |

Failure handling: each failing test blocks the next stage. UT-01 through
UT-06 must be green before any of the on-hardware tests below begin.

---

## 3. Sensor Validation Checklist (on hardware)

### 3.1 VL53L0X ring (six sensors)

| Step | Action | Pass criterion |
|---|---|---|
| 3.1.1 | Power on with all sensors connected | Boot log shows `[ToF] 6 / 6 sensors online` |
| 3.1.2 | Inspect I²C scan output | Devices reported at exactly 0x30, 0x31, 0x32, 0x33, 0x34, 0x35 — no extras, no missing |
| 3.1.3 | Hold a flat target 10 cm in front of each sensor in turn | Cached `mm` field for that channel reads within ±15 % of 100 mm |
| 3.1.4 | Aim every sensor at open air (>2 m) | Cached value reads `TOF_OUT_OF_RANGE` (9999) within one cadence period |
| 3.1.5 | Disconnect one sensor and reboot | Boot log reports failure for that index, the other five remain online, robot still operates |

### 3.2 QTR-MD-03RC edge array

| Step | Action | Pass criterion |
|---|---|---|
| 3.2.1 | Run CALIBRATE state on a black mat with a white border (or printed substitute) | Calibration completes without error; subsequent edge readings span ≈ 0..1000 normalised range |
| 3.2.2 | Hold the bot stationary over black mat | `isOnEdge()` returns `false` continuously for ≥ 30 s |
| 3.2.3 | Slide each sensor over the white border in turn | `edgeDirection()` returns `LEFT`, `CENTER`, `RIGHT` respectively, within the next 8 ms poll |
| 3.2.4 | Slide entire front edge across the border | `edgeDirection()` returns `FRONT` |

### 3.3 Inputs and indicators

| Step | Action | Pass criterion |
|---|---|---|
| 3.3.1 | Toggle each DIP switch | Serial log reflects the new flag on the next FSM tick |
| 3.3.2 | Press the start button in IDLE | Transition to CALIBRATE within one tick |
| 3.3.3 | Status LED behaviour in every state | Matches the table in `strategy.cpp` (slow blink IDLE / fast blink CALIBRATE / 1 Hz COUNTDOWN / solid SEARCH / rapid ATTACK / solid EVADE / off STOP) |

---

## 4. Motor Response Tests

Objective: characterise the actual N20 + TB6612FNG combination so that
the assumed plant parameters in [`SFG_Report.md`](SFG_Report.md) §3 can
be replaced with measured values, providing the analytical foundation
for PID tuning.

### 4.1 Equipment

* Bench supply at 9 V (or fully-charged 3S pack), current-limited.
* Oscilloscope or logic analyser on PWM lines (verifies LEDC output).
* Either (a) hand tachometer / phone strobe app, or (b) a temporary
  optical / hall encoder on each wheel for the duration of the test.
* Robot lifted off the ground so wheels spin freely.

### 4.2 Tests

| Test ID | Procedure | Records | Acceptance criterion |
|---|---|---|---|
| MT-01 | Sweep duty 0 → 255 in steps of 16 each direction; record wheel RPM at each step | Duty vs. RPM curves per motor | Curves are monotonic; left/right match within ±10 % at each step |
| MT-02 | Identify motor dead-band: lowest duty that produces any rotation | Dead-band duty per motor | Documented as a known non-linearity for tuning |
| MT-03 | Apply a step from 0 to 200/255 duty; capture rise to steady state | First-order fit yields `Km` (rad/s/V) and `τm` (s) | Fitted `τm` within the assumed 30–100 ms range |
| MT-04 | Step from +200 to −200 (full reverse) | Reverse rise time and brake behaviour | No driver fault; symmetric within ±15 % |
| MT-05 | Stall test (block one wheel for ≤ 1 s) | Current draw, VM rail droop | Pack voltage stays > 9 V; TB6612FNG does not enter thermal shutdown |

Results from MT-03 update `K_m` and `τ_m` in the SFG report and in
`config.h` constants where applicable. Until MT-03 is completed,
**every PID gain remains symbolic**.

---

## 5. Edge Detection Tests

Objective: verify that the priority-edge architecture survives the
worst case — driving toward the ring at top speed.

| Test ID | Procedure | Acceptance criterion |
|---|---|---|
| ED-01 | Place robot stationary at the edge; manually trigger ATTACK state | Transition to EVADE within ≤ 20 ms of the QTR sensor reading dropping below `EDGE_THRESHOLD` (= 2.5 × `EDGE_POLL_MS`) |
| ED-02 | Drive forward at SPEED_ATTACK toward the white border on the dohyo | Robot reverses before any QTR sensor crosses the border (chassis stops with sensor bar still over black mat) |
| ED-03 | Pivot in place near the border (sweep into ring edge) | EVADE retreats away from the side that triggered; no oscillation back into the edge |
| ED-04 | Cover the front three sensors with a sheet of paper (false-positive simulation) | Robot enters EVADE within the next 8 ms poll |
| ED-05 | Repeat ED-02 with ATTACK target also visible (split decision) | Edge handling wins; ATTACK is pre-empted unconditionally |

Acceptance for ED-02 is the **single most important behavioural test in
this plan**; failure here makes the bot unsafe to run in a match.

---

## 6. ToF Latency Verification

Objective: prove that the continuous-mode ToF refactor delivers the
order-of-magnitude latency reduction it was designed for.

### 6.1 Measurement method

* Add temporary timing code that logs `micros()` immediately before and
  after each `ToF::pollFront()` / `ToF::pollSide()` call.
* Capture 1000 samples per call site over a 30 s benchtop run.
* Reduce to: median, 95th percentile, 99th percentile per call site.

### 6.2 Acceptance criteria (targets)

| Metric | Target |
|---|---|
| Median `Edge::poll()` duration | ≤ 3 ms |
| 99th percentile `Edge::poll()` duration | ≤ 5 ms |
| Median `ToF::pollFront()` duration | ≤ 2 ms (cache harvest, non-blocking) |
| 99th percentile `ToF::pollFront()` duration | ≤ 4 ms |
| Median `ToF::pollSide()` duration | ≤ 2 ms |
| Worst-case time from "edge below threshold" to "Motors::reverse() command" | ≤ 20 ms |

These targets are derived from the configured cadences in `config.h`
and the QTR RC discharge timeout (~2.5 ms). If any 99-th percentile
exceeds the corresponding target by more than 50 %, instrumentation
output is reviewed for I²C contention before code is changed.

---

## 7. PID Tuning Workflow

Cross-reference: [`Control_System_Analysis.md`](Control_System_Analysis.md)
§2.3. This section is the *test plan* view of the same workflow.

| Stage | Activity | Exit criterion |
|---|---|---|
| 7.1 Inner-sensor install | Mount wheel encoders or IMU; confirm reading propagates to firmware | Filtered measurement appears in serial log at ≥ 100 Hz |
| 7.2 Plant identification | MT-03 step response, fit `Gm(s) = Km/(τm·s + 1)` | Fit residual < 10 % of step amplitude |
| 7.3 Routh check | Substitute measured `Km, τm` into the SFG characteristic polynomial; compute the stable `Kp` range with `Ki = Kd = 0` | A non-empty stable interval exists |
| 7.4 Ziegler–Nichols P-tune | Increase `Kp` on hardware until sustained oscillation; record `Ku, Tu` | Oscillation is clean (single dominant frequency), not noise |
| 7.5 ZN table → initial gains | Compute `Kp, Ki, Kd` from the classical ZN PID row | Set in `pid.cpp` defaults |
| 7.6 Closed-loop verification | Step the heading reference; capture overshoot, settling time | Overshoot ≤ 15 %, 2 % settling ≤ 5·τm |
| 7.7 Anti-wind-up validation | Saturate PWM intentionally (oversized step) | No post-saturation overshoot beyond +5 % over the unsaturated case |
| 7.8 Bode / margin verification | Re-evaluate the analytic open-loop transfer with tuned gains | Phase margin ≥ 45°, gain margin ≥ 6 dB |

Stage 7.6 onwards is the first point in the project where the SFG
report's symbolic results become quantitative.

---

## 8. Disturbance Rejection Testing

Objective: confirm both the supervisory (edge override) and linear (PID
integrator) disturbance paths described in
[`SFG_Report.md`](SFG_Report.md) §7.

| Test ID | Disturbance applied | Method | Acceptance criterion |
|---|---|---|---|
| DR-01 | Lateral push during straight drive | Push the bot 1–2 cm sideways with a wooden dowel while it drives | Bot returns to within ±5° of original heading within 2× settling time (post-tuning) |
| DR-02 | Sustained side load | Apply constant low push for 2 s | Heading offset converges to 0 (zero steady-state error from integrator) |
| DR-03 | Impulse contact | Tap the front with a 100 g mass at low speed | No false EVADE; bot resumes ATTACK trajectory |
| DR-04 | Surface friction discontinuity | Half black mat / half rougher surface | Trajectory deviation bounded; no edge false positive |
| DR-05 | Opponent contact (foam bot stand-in) | Push back during ATTACK | Either (a) the bot wins the push, or (b) EVADE triggers if pushed near the edge |

DR-02 cannot pass before stage 7.6 of the tuning workflow.

---

## 9. Battery Discharge Testing

Objective: characterise pack runtime and confirm safe behaviour
through the LM2596 brown-out region.

### 9.1 Procedure

1. Charge 3S 18650 pack fully (~12.6 V open-circuit).
2. Place robot on blocks (wheels free) and run a *representative load
   profile*: alternate 5 s SEARCH, 5 s ATTACK, 1 s EVADE on repeat.
3. Log pack voltage (multimeter or temporary ADC tap) every 60 s until
   the pack drops to **9 V at the pack terminals**, which is the
   conservative cut-off for the 3S Li-ion chemistry.
4. Record total runtime, motor-rail voltage at cut-off, and any
   undervoltage-lockout events from the BMS.

### 9.2 Acceptance criteria

| Metric | Target |
|---|---|
| Runtime to 9 V pack | ≥ 20 minutes (sufficient for several match rounds) |
| TB6612FNG VM rail at cut-off | ≥ 7 V (driver still in-spec) |
| ESP32 5 V rail | Stays within 4.7–5.3 V throughout |
| Brown-outs / resets during the run | Zero |

### 9.3 Safety controls during this test

* Cells must be in a fire-rated container or on a non-flammable surface.
* Stop the test immediately if any cell exceeds 45 °C or the BMS trips.
* This test is **not** to be performed while the firmware is being
  modified.

---

## 10. Acceptance Criteria Summary

A single-page reference for the panel:

| Domain | Hard pass criterion |
|---|---|
| Bring-up | 6 / 6 ToF online, QTR calibrated, FSM cycles through every state |
| Edge survival | Robot reverses before any QTR sensor crosses the ring border at SPEED_ATTACK |
| Latency | Worst-case edge response ≤ 20 ms; FSM tick never blocks > 5 ms |
| Motor symmetry | Left / right RPM match within ±10 % across the duty range |
| Tuning targets | Phase margin ≥ 45°, overshoot ≤ 15 %, no sustained oscillation |
| Disturbance | Zero steady-state heading error under constant lateral push |
| Runtime | ≥ 20 minutes representative-load runtime to 9 V pack |
| Safety | No brown-outs, no thermal shutdowns, no flame events |

---

## 11. Known Risks and Mitigations

| # | Risk | Likelihood × Impact | Mitigation |
|---|---|---|---|
| R-01 | ToF cross-talk between adjacent sensors in the ring (modulated 940 nm pulses leak) | M × M | Stagger continuous-mode periods (already done in firmware: front 25 ms, side 70 ms); add foam baffles between sensors if observed |
| R-02 | I²C bus pull-up too strong with six VL53L0X breakouts in parallel | M × M | De-solder pull-ups on five of six boards; verify rise time on scope |
| R-03 | Motor stall current browns out the 5 V logic rail | M × H | 1000 µF bulk capacitor at VM (already in BOM §5); separate LM2596 for logic |
| R-04 | PWM dead-band — low-duty commands produce no motion | H × L | Quantify in MT-02; clamp commanded PWM above the measured dead-band threshold |
| R-05 | Edge sensor false positive from shadow / reflective tape | L × H | Calibrate on the actual mat being used; raise `EDGE_THRESHOLD` if the calibrated histogram shows narrow margin |
| R-06 | JTAG pins used as GPIO (40/41/42 = MTDO/MTDI/MTMS) block hardware debug | L × L | Documented; debug via USB-CDC Serial, not JTAG. Pins can be released for JTAG by repurposing SW1–3 if needed |
| R-07 | Hand-picked attack PWM saturates traction control when PID is later added | M × M | Re-tune PID with current attack PWM as upper bound; add anti-wind-up clamp |
| R-08 | 3S pack over-discharge during long tuning sessions | M × H | Battery test enforces 9 V cutoff; BMS provides hardware backstop |
| R-09 | XSHUT bring-up race if power rises slowly | L × M | 20 ms shutdown delay before reassign; retry loop on `begin()` failure (future improvement) |
| R-10 | Code changes after passing a test invalidate that result | M × M | Re-run UT-01..UT-06 on every firmware change; tag the commit hash that passed each hardware test |

Likelihood × Impact key: L = low, M = medium, H = high.

---

## 12. Test Log Template

The actual results, when collected, will be recorded in a table of
this form (one row per test instance):

| Date | Firmware SHA | Test ID | Result | Measured value(s) | Tester | Notes |
|---|---|---|---|---|---|---|

This log is intentionally **empty in this revision of the document**;
it will be populated as tests are executed.

---

## 13. References

* [`SFG_Report.md`](SFG_Report.md) — control-system context for the
  motor and tuning tests.
* [`Control_System_Analysis.md`](Control_System_Analysis.md) §2 — full
  description of the PID tuning workflow.
* [`BOM.md`](BOM.md) — components referenced in the equipment lists.
* STMicroelectronics, *VL53L0X API user manual*, UM2039, §6 (continuous
  ranging timing).
* Toshiba, *TB6612FNG datasheet*, §Electrical Characteristics (Vstall,
  thermal shutdown).
