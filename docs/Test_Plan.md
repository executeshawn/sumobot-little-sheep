# Engineering Test Plan — LittleSheep Sumobot

**Document type:** Validation procedures for the validation + tuning
phase.
**Workflow:** All tests use the PlatformIO **per-environment** build
flow — `pio run -e <env> -t upload`. **No `pio test`. No Unity.** Each
bring-up firmware is an ordinary Arduino sketch in `test/<env>/` and
exercises the real production modules from `lib/Sumobot/` via the LDF.
**Companion documents:**
[`SFG_Report.md`](SFG_Report.md),
[`Control_System_Analysis.md`](Control_System_Analysis.md),
[`../test/README.md`](../test/README.md).

---

## 1. Purpose and Scope

This plan defines the engineering tests for taking the LittleSheep
through bench validation to tournament readiness.

Scope:

* Subsystem bring-up via the PlatformIO `test_*` environments.
* Motor characterisation and calibration constant measurement.
* Edge-detection survival validation.
* ToF latency / cadence verification.
* Open-loop integration test (`test_full_system`).
* PID tuning workflow — applies only **after** an inner-loop sensor is
  installed.
* Disturbance rejection testing.
* Battery discharge characterisation on the 2S Molicel pack.

Out of scope for the current revision:

* Native / host-side unit tests with Unity — the project explicitly
  does not use `pio test`.

---

## 2. Build and Flash Workflow

All commands run from the repo root.

### 2.1 Production firmware

```bash
pio run -e esp32s3 -t upload
pio device monitor -e esp32s3        # @ 115200
```

### 2.2 Bring-up firmwares

```bash
pio run -e <env> -t upload
pio device monitor -e <env>          # @ 115200
```

with `<env>` chosen from the table below. Each env scopes its source
via `build_src_filter` so exactly one `setup()` / `loop()` is linked.

| Env | Subsystem | Motors? | Operator action |
|---|---|---|---|
| `esp32s3` | Production firmware (`src/main.cpp`) | Yes — full behaviour | Press start; supervised dohyo run. |
| `test_vl53_bringup` | VL53L0X XSHUT one-at-a-time + address re-assignment | No | Watch Serial for the bring-up sequence; expect 6/6 online and 0x30..0x35. |
| `test_vl53_verify` | Post-bring-up I²C verification | No | Confirm all six sensors respond at 0x30..0x35. |
| `test_vl53` | Live VL53L0X ring streaming | No | Wave a hand / target in front of each sensor; check mm output and cadence. |
| `test_qtr` | QTR-MD-03RC bring-up, calibration sweep, threshold check | No | Slide array over black mat and white border; send any key to advance phases. |
| `test_motor_diag` | TB6612FNG single-channel diagnostic | **Yes (one motor, PWM 90)** | **Lift wheels off the ground or block them.** Validates polarity, IN1/IN2 wiring, left/right mapping, brake, and STBY safety path. |
| `test_motor` | TB6612FNG combined motion (both channels at PWM 90/70) | **Yes (low PWM)** | **Lift wheels off the ground or block them.** Drift check. |
| `test_button` | Start button input path | No | Press button; expect rising-edge events. |
| `test_fsm` | FSM transition simulator | No (no hardware required) | Send single-char commands via Serial monitor; verify every transition. |
| `test_calibration_protocol` | Movement calibration: trim + pivot-ms-per-degree | **Yes (hard-capped PWM 90)** | Mandatory sanity check, mark a straight reference, type observations into Serial, copy the printed config block into `lib/Sumobot/config.h`. |
| `test_full_system` | All modules integrated | **Yes (hard-cap 130, 60 s autonomous timeout)** | Place bot on dohyo with ≥ 30 cm clear on every side, press start, keep main switch in reach. |

---

## 3. Bring-Up Order (recommended)

This sequence matches [`../test/README.md`](../test/README.md) and
moves the operator safely from "boards on the desk" to "supervised
dohyo run".

1. `test_button` — verifies the start-button path.
2. `test_vl53_bringup` — runs the XSHUT sequence; confirm 6 / 6.
3. `test_vl53_verify` — confirms the post-bring-up address map.
4. `test_vl53` — live opponent-detection sanity.
5. `test_qtr` — calibrates and validates the edge threshold.
6. `test_motor_diag` — single-channel polarity / wiring / safety
   check on one motor at a time.
7. `test_motor` — both channels together; drift check.
8. `test_fsm` — FSM-only regression on the actual MCU.
9. `test_calibration_protocol` — measure trim and ms-per-degree, copy
   results into `config.h`.
10. `test_full_system` — final pre-match integration check (safety
    capped).

A failure at any step blocks the next step.

---

## 4. Sensor Validation

### 4.1 VL53L0X ring — six sensors

Environment: `test_vl53_bringup`, `test_vl53_verify`, `test_vl53`.

| Step | Action | Pass criterion |
|---|---|---|
| 4.1.1 | Power on with all sensors connected | Boot log shows `6 / 6 sensors online` |
| 4.1.2 | Inspect I²C scan output | Devices reported at 0x30, 0x31, 0x32, 0x33, 0x34, 0x35 — no extras, no missing |
| 4.1.3 | Hold a flat target at 10 cm in front of each sensor | Cached `mm` reading within ±15 % of 100 mm for the channel under test |
| 4.1.4 | Aim every sensor at open air (> 2 m) | Cached value reads `TOF_OUT_OF_RANGE` (9999) within one cadence period |
| 4.1.5 | Disconnect one sensor and reboot | Boot log reports failure for that index, the other five remain online, production firmware does not fault out |

### 4.2 QTR-MD-03RC edge array

Environment: `test_qtr`.

| Step | Action | Pass criterion |
|---|---|---|
| 4.2.1 | Run the calibration phase on a black mat with a white border | Calibration completes; subsequent readings span ≈ 0..1000 |
| 4.2.2 | Hold the bot stationary over black mat | `isOnEdge()` returns `false` continuously for ≥ 30 s |
| 4.2.3 | Slide each sensor in turn over the white border | `edgeDirection()` returns `LEFT`, `CENTER`, `RIGHT` respectively within the next 8 ms poll |
| 4.2.4 | Slide entire front edge across the border | `edgeDirection()` returns `FRONT` |

### 4.3 Start button

Environment: `test_button`.

| Step | Action | Pass criterion |
|---|---|---|
| 4.3.1 | Press the button | Serial logs a `pressed` event (rising-edge debounced) |
| 4.3.2 | Hold for ≥ 1 s | No additional `pressed` events until release-then-press |

### 4.4 FSM simulator

Environment: `test_fsm`.

| Step | Action | Pass criterion |
|---|---|---|
| 4.4.1 | Send the documented single-char commands | All seven states reachable; ATTACK watchdog fires within ±1 tick of 10 s; EVADE pre-empts on simulated edge |

---

## 5. Motor Response Tests

Environment: `test_motor_diag`, `test_motor`, `test_calibration_protocol`.

Objective: characterise the actual TT + TB6612FNG combination so that
the assumed plant parameters in [`SFG_Report.md`](SFG_Report.md) §3 can
be replaced with measured values.

### 5.1 Equipment

* 2S Molicel pack (or bench supply at ~7.4 V, current-limited).
* Oscilloscope or logic analyser on the PWMA / PWMB lines (verifies
  LEDC output).
* Either a hand tachometer / phone strobe app, or a temporary optical
  / hall encoder on each wheel for the duration of the test.
* Robot lifted off the ground so wheels spin freely.

### 5.2 Tests

| Test ID | Env | Procedure | Records | Acceptance |
|---|---|---|---|---|
| MT-01 | `test_motor` | Sweep duty 0 → 255 in steps of 16 each direction; record wheel RPM at each step | Duty vs. RPM curves per motor | Curves monotonic; left/right match within ±10 % at each step |
| MT-02 | `test_motor` | Identify motor dead-band: lowest duty that produces any rotation | Dead-band duty per motor | Documented as a known non-linearity for future PID tuning |
| MT-03 | `test_motor` | Apply a step from 0 to 200/255 duty; capture rise to steady state | First-order fit yields `Km` (rad/s/V) and `τm` (s) | Fitted `τm` consistent with assumed range (50–150 ms) |
| MT-04 | `test_motor` | Step from +200 to −200 (full reverse) | Reverse rise time, brake behaviour | No driver fault; symmetric within ±15 % |
| MT-05 | `test_motor_diag` | Stall test (block one wheel for ≤ 1 s) | Current draw, VM rail droop | Pack voltage stays > 6.5 V at the cells; TB6612FNG does not enter thermal shutdown |
| MT-06 | `test_calibration_protocol` | Run the guided calibration. Measures forward drift and 360° pivot time at `SPEED_TURN`. | `MOVE_TRIM_LEFT`, `MOVE_TRIM_RIGHT`, `MOVE_MS_PER_DEG_LEFT/RIGHT` | Bot drives a straight line within ±5° over 1 m; pivot constant consistent for both sides within ±10 %. Copy printed block into `lib/Sumobot/config.h`. |

Results from MT-03 update `K_m` and `τ_m` in the SFG report. Until
MT-03 is completed, **every PID gain remains symbolic**. MT-06 must be
completed *before* the next production firmware run if the trim
defaults are still 1.00.

---

## 6. Edge Detection Tests

Environment: `esp32s3` (with operator standing by) or `test_full_system`.

Objective: verify that the priority-edge architecture survives the
worst case — driving toward the ring at top speed.

| Test ID | Procedure | Acceptance |
|---|---|---|
| ED-01 | Place robot stationary at the edge; force ATTACK via simulated ToF (or `test_fsm`) | Transition to EVADE within ≤ 20 ms of any QTR channel dropping below `EDGE_THRESHOLD` (= 2.5 × `EDGE_POLL_MS`) |
| ED-02 | Drive forward at `SPEED_ATTACK` toward the white border on the dohyo | Robot reverses before any QTR sensor crosses the border (chassis stops with sensor bar still over black mat) |
| ED-03 | Pivot in place near the border (sweep into ring edge) | EVADE retreats away from the side that triggered; no oscillation back into the edge |
| ED-04 | Cover the front three sensors with a sheet of paper (false-positive simulation) | Robot enters EVADE within the next 8 ms poll |
| ED-05 | Repeat ED-02 with a target also in front (split decision) | Edge handling wins; ATTACK is pre-empted unconditionally |

**ED-02 is the single most important behavioural test in this plan.**
Failure here makes the bot unsafe to run in a match.

---

## 7. ToF Latency Verification

Environment: `esp32s3`, instrumented build.

Objective: confirm the cached, non-blocking harvest architecture
delivers an FSM tick that never blocks on a sensor I/O round-trip.

### 7.1 Measurement method

* Add temporary timing code that logs `micros()` immediately before
  and after each `Edge::poll()`, `ToF::pollFront()`,
  `ToF::pollSide()`, and `Strategy::update()` call.
* Capture 1000 samples per call site over a 30 s benchtop run.
* Reduce to median, 95th percentile, 99th percentile per call site.

### 7.2 Acceptance targets

| Metric | Target |
|---|---|
| Median `Edge::poll()` duration | ≤ 3 ms |
| 99th percentile `Edge::poll()` duration | ≤ 5 ms |
| Median `ToF::pollFront()` duration | ≤ 2 ms (cache harvest) |
| 99th percentile `ToF::pollFront()` duration | ≤ 4 ms (or up to `TOF_READ_TIMEOUT_MS` = 50 ms on a stalled sensor — investigate if observed) |
| Median `ToF::pollSide()` duration | ≤ 2 ms |
| Median `Strategy::update()` duration | ≤ 1 ms |
| Worst-case "edge below threshold" → `Motors::reverse()` | ≤ 20 ms |

These targets are derived from the cadences in `config.h` and the QTR
RC discharge timeout (~2.5 ms). If any 99-th percentile exceeds its
target by more than 50 %, review for I²C contention before changing
code.

---

## 8. PID Tuning Workflow (planned, post inner-sensor install)

Cross-reference: [`Control_System_Analysis.md`](Control_System_Analysis.md)
§7. **Not runnable today.** Requires (a) an encoder or IMU on the
robot, (b) a `pid.cpp` module consuming its filtered measurement.

| Stage | Activity | Exit criterion |
|---|---|---|
| 8.1 Inner-sensor install | Mount wheel encoders or IMU; confirm reading propagates to firmware | Filtered measurement appears in Serial at ≥ 100 Hz |
| 8.2 Plant identification (MT-03) | Step response, fit `Gm(s) = Km / (τm s + 1)` | Fit residual < 10 % of step amplitude |
| 8.3 Routh check | Substitute measured `Km, τm` into the characteristic polynomial; compute the stable `Kp` range with `Ki = Kd = 0` | Non-empty stable interval |
| 8.4 Ziegler–Nichols P-tune | Raise `Kp` on hardware until sustained oscillation; record `Ku, Tu` | Oscillation is clean (single dominant frequency) |
| 8.5 ZN table → initial gains | Compute `Kp, Ki, Kd` from the classical ZN PID row | Set in `pid.cpp` defaults |
| 8.6 Closed-loop verification | Step the heading reference; capture overshoot, settling time | Overshoot ≤ 15 %, 2 % settling ≤ 5·τm |
| 8.7 Anti-wind-up validation | Saturate PWM intentionally (oversized step) | No post-saturation overshoot beyond +5 % over the unsaturated case |
| 8.8 Bode / margin verification | Re-evaluate the analytic open-loop transfer with tuned gains | Phase margin ≥ 45°, gain margin ≥ 6 dB |

Stage 8.6 onwards is the first point at which the SFG report's
symbolic results become quantitative.

---

## 9. Disturbance Rejection Testing

Environment: `esp32s3`, supervised.

| Test ID | Disturbance | Method | Acceptance |
|---|---|---|---|
| DR-01 | Lateral push during straight drive | Push the bot 1–2 cm sideways with a wooden dowel while it drives | (Post-PID) Bot returns to within ±5° of original heading within 2× settling time. (Pre-PID) Bot continues, no false EVADE. |
| DR-02 | Sustained side load | Apply constant low push for 2 s | (Post-PID) Heading offset converges to 0 (integrator). (Pre-PID) Expected to accumulate offset — documented limitation. |
| DR-03 | Impulse contact | Tap the front with a 100 g mass at low speed | No false EVADE; bot resumes ATTACK trajectory |
| DR-04 | Surface friction discontinuity | Half black mat / half rougher surface | Trajectory deviation bounded; no edge false positive |
| DR-05 | Opponent contact (foam-bot stand-in) | Push back during ATTACK | Either the bot wins the push, or EVADE triggers if pushed near the edge |

DR-02 cannot pass before stage 8.6 of the tuning workflow. This is the
honest gap that the validation phase will leave open until the PID is
implemented.

---

## 10. Battery Discharge Testing

Environment: `esp32s3`, robot on blocks.

Objective: characterise pack runtime and confirm safe behaviour
through the LM2596 brown-out region. Pack is a **2S Molicel
INR-18650A**, nominal 7.4 V, full-charge 8.4 V.

### 10.1 Procedure

1. Charge the pack fully (~ 8.4 V open-circuit).
2. Place robot on blocks (wheels free) and run a representative
   load profile: alternate 5 s SEARCH, 5 s ATTACK, 1 s EVADE on
   repeat.
3. Log pack voltage every 60 s until the pack drops to **6.0 V at
   the pack terminals** (3.0 V/cell — conservative cut-off for the
   INR-18650A chemistry).
4. Record total runtime, motor-rail voltage at cut-off, and any
   undervoltage-lockout events from the BMS.

### 10.2 Acceptance targets

| Metric | Target |
|---|---|
| Runtime to 6.0 V pack | ≥ 20 minutes (sufficient for several match rounds) |
| TB6612FNG VM rail at cut-off | ≥ 5 V (driver still in-spec for low-PWM commands) |
| ESP32 5 V rail | Stays within 4.7–5.3 V throughout |
| Brown-outs / resets during the run | Zero |

### 10.3 Safety controls

* Cells must be in a fire-rated container or on a non-flammable
  surface.
* Stop the test immediately if any cell exceeds 45 °C or the BMS
  trips.
* Do not modify firmware during a discharge run.

---

## 11. Acceptance Criteria Summary

A single-page panel reference:

| Domain | Hard pass criterion |
|---|---|
| Bring-up | 6 / 6 ToF online at 0x30..0x35, QTR calibrated, every FSM state reachable |
| Edge survival | Robot reverses before any QTR sensor crosses the ring border at `SPEED_ATTACK` (ED-02) |
| Latency | Worst-case edge → reverse ≤ 20 ms; FSM tick never blocks > 5 ms |
| Motor symmetry | Left / right RPM match within ±10 % across the duty range |
| Calibration | `MOVE_TRIM_*` and `MOVE_MS_PER_DEG_*` measured via `test_calibration_protocol` |
| Tuning targets (post-PID) | Phase margin ≥ 45°, overshoot ≤ 15 %, no sustained oscillation |
| Disturbance | (Post-PID) Zero steady-state heading error under constant lateral push |
| Runtime | ≥ 20 min representative-load runtime to 6.0 V pack |
| Safety | No brown-outs, no thermal shutdowns, no thermal events on the cells |

---

## 12. Risks Carried into Validation

| # | Risk | L × I | Mitigation |
|---|---|---|---|
| R-01 | ToF cross-talk between adjacent sensors in the ring (940 nm leakage) | M × M | Staggered continuous-mode periods (front 25 ms, side 70 ms); foam baffles between sensors if observed during `test_vl53` |
| R-02 | I²C bus pull-up too strong with six VL53L0X breakouts | M × M | Five of six on-board pull-ups already de-soldered; verify rise time on scope if a sensor fails to init |
| R-03 | TT-motor stall current browns out the 5 V logic rail | M × H | 1000 µF bulk capacitor at VM (BOM §5); separate LM2596 for logic |
| R-04 | PWM dead-band — low-duty commands produce no motion | H × L | Quantify in MT-02; clamp commanded PWM above the dead-band threshold when PID is later added |
| R-05 | Edge sensor false positive from shadow / reflective tape | L × H | Calibrate on the actual mat being used; raise `EDGE_THRESHOLD` if the calibrated histogram shows narrow margin |
| R-06 | JTAG pins repurposed as GPIO (39 / 40 = MTDO / MTCK) block hardware debug | L × L | Documented in [`pins.h`](../lib/Sumobot/pins.h); debug via USB-CDC Serial |
| R-07 | XSHUT bring-up race if power rises slowly | L × M | 10 ms XSHUT settle delay; manual reboot if 6/6 not seen at boot |
| R-08 | 2S pack over-discharge during long tuning sessions | M × H | 6.0 V cut-off observed during the discharge test; BMS provides hardware backstop |
| R-09 | Movement trim/pivot constants drift after a chassis change | M × L | Re-run `test_calibration_protocol` after any mechanical change |
| R-10 | Code changes after passing a test invalidate that result | M × M | Tag the commit hash that passed each hardware test in §13 |

L × I key: L = low, M = medium, H = high.

---

## 13. Test Log Template

One row per test instance, to be populated as tests are executed:

| Date | Firmware SHA | Env | Test ID | Result | Measured value(s) | Tester | Notes |
|---|---|---|---|---|---|---|---|

This log is intentionally empty in this revision.

---

## 14. References

* [`SFG_Report.md`](SFG_Report.md) — control-system context for motor
  and tuning tests.
* [`Control_System_Analysis.md`](Control_System_Analysis.md) §7 — full
  PID tuning workflow description.
* [`BOM.md`](BOM.md) — components referenced in the equipment lists.
* [`../test/README.md`](../test/README.md) — per-env operator guide,
  safety rules, and recommended order.
* STMicroelectronics, *VL53L0X API user manual*, UM2039 (continuous
  ranging timing).
* Toshiba, *TB6612FNG datasheet* (Vstall, thermal shutdown).
