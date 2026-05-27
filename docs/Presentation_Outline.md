# Presentation Outline — LittleSheep Sumobot

**Course:** CPE335 Sumobot Challenge
**Target length:** 10 minutes (≈ 60 s per slide, 14 slides)
**Honest framing rule:** Present the robot as a **deterministic FSM
with reactive sensor logic that is structurally PID-ready**. Do **not**
claim full closed-loop PID control. Every slide that touches motion
control or "closed-loop" must say so.

---

## Slide 1 — Title (≈ 15 s)

* Project name: **LittleSheep — Autonomous Sumobot**
* Course code, team members, date.
* One-line tag: "ESP32-S3 differential-drive sumobot with a 6-channel
  ToF opponent ring, a 3-channel RC edge array, and a deterministic
  FSM that is PID-ready for the next phase."
* Speaker note: *"We will walk through the hardware, the firmware
  architecture, the control-system analysis, and the current testing
  status."*

## Slide 2 — Objectives (≈ 45 s)

In priority order:

1. Build a competition-class sumobot within the size and weight
   rules.
2. Implement a deterministic supervisory FSM with edge-priority
   safety (IDLE → CALIBRATE → COUNTDOWN → SEARCH → ATTACK → EVADE →
   STOP).
3. Provide a non-blocking multi-rate sensor architecture so the FSM
   tick never waits on a sensor I/O round-trip.
4. Design and analyse a PID inner loop via SFG + Mason's gain
   formula. Implement it once an inner-loop sensor is added.
5. Document hardware, firmware, and control theory to academic
   standard.

Speaker note: *"Objective 4 is explicitly design-level. We have the
math and the structural readiness; we do not claim measured PID
gains."*

## Slide 3 — System Architecture (≈ 60 s)

Block diagram showing two cooperating layers:

* **Supervisory FSM** — `strategy.cpp`, deterministic, no DIP-switch
  mode selection.
* **Reactive sensor cache** — multi-rate non-blocking; Edge 8 ms,
  Front ToF 30 ms, Side/Rear ToF 90 ms, FSM 5 ms.

A third layer (**Inner PID loop**) is architecturally provisioned
but **not active** in this build.

Speaker note: *"The first two layers are implemented and exercised on
hardware. The inner PID is wired structurally but currently issues
fixed-PWM commands."*

## Slide 4 — Hardware Overview (≈ 45 s)

Photograph / render + bullet inventory of the **final validated
hardware**:

* ESP32-S3-DevKitC-1 N16R8 main controller (16 MB flash, 8 MB octal
  PSRAM)
* TB6612FNG dual H-bridge driver
* 6 × GY-VL53L0X opponent-detection ring (Pololu library)
* Pololu QTR-MD-03RC 3-channel edge array
* Two **TT geared motors**, 6 V / ≈ 160 RPM, L-shape
* **2S Molicel INR-18650A** pack + 2S BMS + two LM2596 buck
  converters

Cross-reference [`BOM.md`](BOM.md) for the full bill of materials.

Speaker note: *"TT motors replaced the original N20 plan during
bring-up for better push-torque. The 2S Molicel pack replaced the
earlier 3S plan for a gentler buck-down ratio and a flatter discharge
curve."*

## Slide 5 — ESP32-S3 Pin Mapping + Migration Story (≈ 60 s)

Compact pin table — emphasise that the mapping was validated against
ESP32-S3 N16R8 octal-PSRAM restrictions:

* Motors A (left):  PWMA=4, AIN1=5, AIN2=6
* Motors B (right): PWMB=7, **BIN1=47, BIN2=21**, STBY=18
* I²C: SDA=8, SCL=9
* VL53L0X XSHUT: FL=10, FC=11, FR=12, L=13, R=14, **RR=2**
* QTR edge: LEFT=38, CENTER=39, RIGHT=40
* Start button: 1 (INPUT_PULLUP)

**BIN1 / BIN2 migration story (talking point):**

* Iteration 1 — **16 / 17** → conflicted with the octal-PSRAM bus.
* Iteration 2 — **41 / 42** → unreliable direction-latching on this
  S3 N16R8 build.
* Iteration 3 — **47 / 21** → stable. **This is the locked map.**

VL53L0X RR also moved off GPIO 15 to GPIO 2 to keep all
timing-sensitive digital outputs away from the PSRAM bus.

## Slide 6 — Finite-State Machine (≈ 75 s)

State diagram:

```
IDLE ── btn ──► CALIBRATE ── 4s ──► COUNTDOWN ── 5s ──► SEARCH
                                                          │  ▲
                                              opponent ───┘  │  edge → EVADE → SEARCH
                                                          ▼  │
                                                        ATTACK
                                                          │
                                                 watchdog 10 s
                                                          ▼
                                                        SEARCH
```

Talking points:

* IDLE: motors stopped.
* CALIBRATE: slow spin so the QTR sees both mat and ring; calls
  `Edge::calibrateEdge()` repeatedly.
* COUNTDOWN: mandatory 5 s pre-match delay.
* SEARCH: deterministic alternating sweep at `SPEED_SEARCH = 140`
  every 600 ms.
* ATTACK: ToF bearing → differential-drive command at `SPEED_ATTACK
  = 230`. Inner wheel = 3/5 outer for veering bearings; pure pivots
  for side / rear bearings.
* EVADE: REVERSE 350 ms then pivot 450 ms away from the tripped
  edge.
* STOP: STBY pulled LOW (motors coast); latched on fault.

## Slide 7 — Sensor Architecture (≈ 60 s)

* **Priority hierarchy:** edge > front ToF > side / rear ToF. The FSM
  checks `Edge::isOnEdge()` *before* any ToF logic in SEARCH,
  ATTACK, and EVADE.
* **Multi-rate polling**, configured in `config.h`:
  * Edge — every **8 ms** (FAST, top priority)
  * Front ToF — every **30 ms** (MEDIUM)
  * Side / rear ToF — every **90 ms** (SLOW)
* **FSM tick** runs every **5 ms** on cached data — never blocks on
  sensor I/O.
* VL53L0X uses **continuous-ranging mode**; XSHUT one-at-a-time
  bring-up to 0x30..0x35.
* QTR is **RC discharge timing** mode; calibrated 0..1000.

## Slide 8 — Block Diagram and Signal Flow Graph (≈ 90 s)

Insert the block diagram and SFG from
[`SFG_Report.md`](SFG_Report.md) §4.

* One forward path:
  `P₁ = Gc(s)·K_drv·Gm(s)·Gr(s)`
* One loop:
  `L₁ = −P₁`
* Mason's gain formula:
  `T(s) = P₁·Δ₁ / Δ = G(s) / (1 + G(s))`
* Closed-form denominator:
  `τm·s³ + s² + K_drv·Km·(Kd·s² + Kp·s + Ki)`

Speaker note: *"Numerical parameters are datasheet-derived
assumptions. PID gains are symbolic. The SFG describes the target
control architecture; the firmware mapping table in the SFG report
discloses which elements are implemented and which remain
theoretical."*

## Slide 9 — Disturbance Rejection (≈ 45 s)

Two complementary mechanisms:

1. **Non-linear, active today.** The supervisory layer treats edge
   detection as a priority interrupt and forces EVADE regardless of
   inner-loop state.
2. **Linear, planned post-PID.** The PID integrator combined with the
   kinematic integrator drives `Y(jω)/D(jω) → 0` at low frequency,
   rejecting a sustained opponent push. **Not active today**, because
   the inner loop is not closed.

This dual approach is the project's main design hypothesis for
surviving opponent contact.

## Slide 10 — Rubric Self-Assessment (≈ 60 s) **(honesty slide)**

| Rubric | Status |
|---|---|
| Autonomous operation | ✅ |
| Multi-rate non-blocking architecture | ✅ |
| FSM stability | ✅ |
| Mathematical control concepts (SFG + Mason) | ✅ documented |
| Sensor filtering | 🟡 implicit only — moving-avg / Kalman planned |
| Disturbance rejection | 🟡 non-linear (EVADE) only; linear awaits PID |
| Closed-loop control | 🟡 outer-loop only; inner loop not closed |
| Tuned PID | ❌ symbolic only |

Cross-reference: [`Control_System_Analysis.md`](Control_System_Analysis.md)
§1.

Speaker note: *"This slide is the honest summary. We have the
structural skeleton and the math; we do not have a tuned PID."*

## Slide 11 — Test Infrastructure (≈ 60 s)

PlatformIO multi-env workflow — **no `pio test`, no Unity**. Each
bring-up firmware is an Arduino sketch built via
`pio run -e <env> -t upload`.

Environments:

* Production: `esp32s3`
* Subsystem bring-up: `test_vl53_bringup`, `test_vl53_verify`,
  `test_vl53`, `test_qtr`, `test_motor_diag`, `test_motor`,
  `test_button`, `test_fsm`
* Calibration: `test_calibration_protocol` (prints trim and
  ms-per-degree constants into Serial; operator copies the block
  into `config.h`)
* Integration: `test_full_system` (hard-cap PWM 130, 60 s autonomous
  timeout)

Speaker note: *"`test_calibration_protocol` is the bridge between
'wired up' and 'drives a straight line'. It is the next test we
run."*

## Slide 12 — What Is PID-Ready (≈ 45 s)

The firmware is structurally ready to receive a PID. What is already
in place:

* Deterministic 5 ms FSM tick.
* Signed differential drive primitive — `Motors::drive(left, right)`.
* Soft + hard speed caps that compose with `min()` — natural
  saturation-feedback hook.
* `Movement` calibration layer — open-loop trim and pivot-by-degree
  abstraction.
* Cached, non-blocking sensor harvest.

What is missing:

* Inner-loop sensor (encoder or IMU).
* `pid.cpp` discrete velocity-form controller.
* Measured plant parameters (`Km`, `τm`) from a step-response test.

## Slide 13 — Validation Phase Status (≈ 60 s)

What is validated:

* ✅ All six ToF sensors stream at 0x30..0x35.
* ✅ QTR edge sensor calibrates and reports direction correctly.
* ✅ Both TB6612FNG channels exercise forward / reverse / brake /
  coast at PWM 90 with verified polarity.
* ✅ FSM transitions exercised on hardware via `test_fsm` and on the
  bench in `test_full_system`.

What is pending validation:

* 🟡 `test_calibration_protocol` run on the final chassis to populate
  the trim and pivot constants.
* 🟡 ED-02 — edge survival at `SPEED_ATTACK`.
* 🟡 Battery discharge runtime to 6.0 V on the 2S Molicel pack.

What is not implemented (and is honestly disclosed):

* ❌ PID gains are not tuned. No closed-loop heading control.
* ❌ No encoder, no IMU.
* ❌ No real-match validation against an opposing bot beyond a
  bench target.

## Slide 14 — Future Work + Conclusion (≈ 30 s)

Future work, ranked by control-system impact:

1. Add wheel encoders or an IMU to physically close the inner loop.
2. Run the PID tuning workflow
   ([`Control_System_Analysis.md`](Control_System_Analysis.md) §7).
3. Add a moving-average or Kalman filter on the bearing reference.
4. Telemetry over Wi-Fi for live tuning sessions.

Conclusion: *"The LittleSheep presents a complete and defensible
design through to the control-law analysis. The next milestone is
bench measurement of the plant parameters and the open-loop
calibration constants, followed by closing the inner loop with the
added sensor."*

---

## Appendix — Speaking-time budget

| Slide | Topic | Target (s) |
|---|---|---:|
| 1 | Title | 15 |
| 2 | Objectives | 45 |
| 3 | Architecture | 60 |
| 4 | Hardware | 45 |
| 5 | Pin map + migration | 60 |
| 6 | FSM | 75 |
| 7 | Sensor architecture | 60 |
| 8 | SFG + Mason | 90 |
| 9 | Disturbance rejection | 45 |
| 10 | Rubric self-assessment | 60 |
| 11 | Test infrastructure | 60 |
| 12 | PID-ready | 45 |
| 13 | Validation status | 60 |
| 14 | Future + conclusion | 30 |
| | **Total** | **750 s ≈ 12 min 30 s** |

Trim slides 5, 7, and 11 to ~30 s each if the venue enforces a strict
10-minute cap.

---

## Q&A preparation cues

* *"Have you tuned the PID?"* — "No. The report deliberately presents
  symbolic gains. The SFG and Mason derivation give us the analytic
  starting point for Ziegler–Nichols tuning once we add an encoder
  or IMU. We do not claim a tuned controller."
* *"Why no encoder?"* — "Scope decision for this revision. The SFG
  documents exactly which feedback path is missing, and it is the
  first item in the future-work list."
* *"Then how is the robot 'closed-loop'?"* — "The outer supervisory
  loop is closed at an event-driven level: ToF bearing → FSM →
  motor command → re-sample. The inner continuous loop is **not**
  closed; that is the PID phase."
* *"Why six ToF sensors?"* — "Six × 60° ring gives directional
  opponent bearing without an I²C multiplexer; the XSHUT one-at-a-time
  bring-up reassigns each sensor to 0x30..0x35."
* *"What protects the robot from driving off the ring during a slow
  ToF read?"* — "The edge sensor is polled every 8 ms in a separate
  fast loop and checked first by the FSM in every active state,
  *before* any ToF logic. EVADE pre-empts unconditionally."
* *"Why did BIN1/BIN2 move three times?"* — "ESP32-S3 N16R8 reserves
  GPIO 26–37 for its flash and octal-PSRAM buses, and we observed
  that 15–17 are PSRAM-bus neighbours that cause interference. The
  first map (16 / 17) hit that interference; the second (41 / 42)
  latched intermittently; the final (47 / 21) is stable and locked."
