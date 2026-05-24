# Heatseeker Sumobot — Documentation Index

**Course:** CPE335 Sumobot Challenge
**Platform:** ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM), Arduino
framework on PlatformIO.
**Repository purpose:** Firmware, hardware design, and academic
documentation for an autonomous differential-drive sumobot built around
a 6-channel VL53L0X opponent-detection ring, a Pololu QTR-MD-03RC edge
array, and a TB6612FNG-driven pair of N20 gear motors.

---

## 1. Project Overview

The Heatseeker is an autonomous sumo robot with three architectural
layers:

* **Supervisory** — a finite-state machine that selects match behaviour
  (IDLE → CALIBRATE → COUNTDOWN → SEARCH → ATTACK → EVADE → STOP).
* **Outer loop** — a six-sensor Time-of-Flight ring generates the
  control reference (opponent bearing); a three-channel reflectance
  array issues priority edge overrides.
* **Inner loop** — a PID heading/velocity controller commanding the
  TB6612FNG motor driver; provisioned in firmware, awaiting empirical
  tuning.

**Current status:** *Firmware architecture and control-system framework
implemented; currently under integration, tuning, and validation.* No
PID gains, plant parameters, or match results in this repository are
experimentally measured at the time of writing.

---

## 2. Documentation Map

All design documentation lives under `docs/`. Each file is
self-contained but cross-linked to the others.

| File | Purpose | Major contents |
|---|---|---|
| [`README.md`](README.md) | This index. Entry point for evaluators. | Project overview · doc map · firmware summary · implementation status · reading order |
| [`SFG_Report.md`](SFG_Report.md) | Theoretical control-system model of the sumobot derived from the firmware architecture. | Modelling assumptions · signal flow graph · branch inventory · Mason's gain formula derivation · closed-loop and disturbance transfer functions · SFG↔firmware mapping |
| [`Control_System_Analysis.md`](Control_System_Analysis.md) | PID design, sensor filtering discussion, and SFG↔code mapping. | Theoretical PID tuning methodology · stability and anti-oscillation goals · continuous ToF / cached polling / priority edge logic · planned Kalman / moving-average upgrades · firmware element-by-element mapping |
| [`BOM.md`](BOM.md) | Professional bill of materials. | Itemised parts (compute, sensing, actuation, power, passives, wiring, chassis) · quantities and estimated PHP/USD pricing · per-section subtotals · component justification |
| [`Presentation_Outline.md`](Presentation_Outline.md) | 10-minute oral-defense deck structure. | 14-slide outline with speaker notes · per-slide time budget · honesty framing on Slide 13 · Q&A preparation cues |
| [`Test_Plan.md`](Test_Plan.md) | Planned engineering validation procedures. | Unit tests · sensor bring-up checklist · motor characterisation (MT-01..MT-05) · edge tests (ED-01..ED-05) · ToF latency targets · PID-tuning workflow · disturbance tests (DR-01..DR-05) · battery discharge methodology · acceptance criteria · risk register |
| [`Glossary.md`](Glossary.md) | Quick-reference definitions for oral defense. | 18 terms (PID, FSM, Mason's gain formula, transfer function, disturbance rejection, sensor fusion, continuous ranging, closed/open loop, PWM, latency, polling, non-blocking, RC reflectance, edge detection, stability, overshoot, settling time) |

---

## 3. Firmware Architecture Summary

* **Finite-State Machine** — Seven states implemented in
  [`src/strategy.cpp`](../src/strategy.cpp). EVADE pre-empts every
  other state on any edge event. An ATTACK watchdog forces a return to
  SEARCH after 10 s of stuck pursuit. DIP switches select aggressive
  vs defensive, sweep vs spin search, and torque vs fast speed profile.

* **Continuous VL53L0X sensing** — All six sensors are brought up via
  the XSHUT one-at-a-time sequence and reassigned to unique I²C
  addresses 0x30..0x35 ([`src/sensors_tof.cpp`](../src/sensors_tof.cpp)).
  Each then runs in **continuous-ranging mode** with the
  `VL53L0X_SENSE_HIGH_SPEED` preset, at 25 ms intermeasurement period
  for the front three sensors and 70 ms for the side/rear three. The
  firmware harvests completed measurements non-blockingly via
  `isRangeComplete()`; the FSM only ever reads cached values.

* **QTR edge detection** — Pololu QTR-MD-03RC in RC (digital
  discharge-timing) mode on three channels
  ([`src/sensors_edge.cpp`](../src/sensors_edge.cpp)). Calibrated at
  boot during the CALIBRATE state via a slow rotation that sweeps the
  array over both the black mat and the white border. White detected =
  calibrated reading below `EDGE_THRESHOLD = 250 / 1000`.

* **TB6612FNG motor control** — Two N20 motors driven directly from
  [`src/motors.cpp`](../src/motors.cpp) using ESP32 LEDC PWM at 5 kHz /
  8-bit on channels 0 and 1, with `digitalWrite()` on the IN1/IN2/STBY
  direction pins. No external motor library is required. The wrapper
  provides `forward / reverse / turnLeft / turnRight / drive / brake /
  stop` and a global speed cap used by torque mode.

* **Asynchronous polling strategy** — Four independent timers in
  [`src/main.cpp`](../src/main.cpp):

  | Loop | Period | Purpose |
  |---|---:|---|
  | `Edge::poll()` | **8 ms** | FAST, top priority |
  | `ToF::pollFront()` | 30 ms | MEDIUM, engagement |
  | `ToF::pollSide()` | 90 ms | SLOW, flank awareness |
  | `Strategy::update()` | 5 ms | FSM tick, reads caches only |

* **Safety mechanisms** —
  * Edge sensor polled fastest and checked first in every active FSM
    state, so the robot reverses before it can drive off the ring.
  * ATTACK watchdog reverts to SEARCH if no transition occurs for 10 s.
  * `Strategy::setFault()` latches a STOP transition if zero ToF
    sensors come online.
  * STOP state pulls TB6612FNG STBY low (motor coast, no holding
    current).
  * 5 s mandatory pre-match countdown after CALIBRATE before any
    motion is permitted.
  * Hardware-side: 10 kΩ pulldown on STBY, 470–1000 µF bulk capacitor
    across VM/GND, star-ground topology (see `pins.h` HARDWARE NOTES).

---

## 4. Repository Structure

```
sumobot-heatseeker/
├── platformio.ini                  PlatformIO target, lib_deps, PSRAM config
├── src/
│   ├── main.cpp                    Setup, I²C scan, multi-rate poll loop
│   ├── pins.h                      All GPIO constants + ESP32-S3 cautions
│   ├── config.h                    Thresholds, speeds, poll rates, timing
│   ├── motors.h / motors.cpp       TB6612FNG via LEDC PWM
│   ├── sensors_tof.h / .cpp        6× VL53L0X, continuous mode, front/side
│   ├── sensors_edge.h / .cpp       QTR-MD-03RC RC mode, cached poll
│   └── strategy.h / strategy.cpp   FSM, DIP logic, ATTACK watchdog
└── docs/
    ├── README.md                   (this file)
    ├── SFG_Report.md
    ├── Control_System_Analysis.md
    ├── BOM.md
    ├── Presentation_Outline.md
    ├── Test_Plan.md
    └── Glossary.md
```

---

## 5. Current Implementation Status

| Area | Status |
|---|---|
| ESP32-S3 GPIO architecture (validated against PSRAM / strapping / USB / JTAG restrictions) | ✅ Implemented |
| TB6612FNG motor driver wrapper (LEDC PWM, speed cap, brake/coast) | ✅ Implemented |
| VL53L0X XSHUT bring-up + address reassignment (0x30..0x35) | ✅ Implemented |
| VL53L0X continuous-ranging mode with cached non-blocking harvest | ✅ Implemented |
| QTR-MD-03RC edge sensing (RC mode, calibration, cached poll) | ✅ Implemented |
| Multi-rate asynchronous polling (fast edge / medium front / slow side / FSM tick) | ✅ Implemented |
| Finite-state machine (7 states + transitions + ATTACK watchdog) | ✅ Implemented |
| DIP-switch behaviour selection (aggressive / spin-search / torque) | ✅ Implemented |
| Status LED indication per state | ✅ Implemented |
| Signal flow graph + Mason's gain formula derivation (documentation) | ✅ Implemented |
| Bench bring-up of motors and sensors | 🟡 In Progress |
| Motor characterisation (`Km`, `τm` identification — MT-03) | 🟡 In Progress |
| PID controller code module (`pid.cpp`) | ❌ Planned |
| PID empirical tuning (Ziegler–Nichols on hardware) | ❌ Planned |
| Inner-loop sensor (wheel encoder or IMU) installation | ❌ Planned |
| Closed-loop heading control physically closed | ❌ Planned |
| Disturbance rejection validation (DR-01..DR-05) | ❌ Planned |
| ToF latency verification on hardware (§6 of Test Plan) | ❌ Planned |
| Battery discharge / runtime characterisation | ❌ Planned |
| Moving-average / Kalman filter on opponent bearing | ❌ Planned |
| Wi-Fi telemetry for live tuning sessions | ❌ Planned |
| Tournament validation against real opposing bots | ❌ Planned |

Legend: ✅ Implemented · 🟡 In Progress · ❌ Planned.

---

## 6. Academic Integrity Note

This documentation has been prepared to support an honest engineering
defense at the design and integration stage of the project. The
following constraints have been observed throughout:

* **No fabricated tuning data.** PID gains (`Kp, Ki, Kd`) appear as
  symbols in every document until experimental tuning is carried out.
  No numerical "tuned values" are presented.
* **No fabricated match results.** No claims are made about wins,
  losses, opponent comparisons, or competition outcomes. The robot has
  not yet competed.
* **Theoretical control models are clearly marked as assumed / derived.**
  Plant parameters (`Km`, `τm`, `K_drv`) used in the SFG report are
  explicitly labelled as datasheet-derived assumptions that will be
  replaced by measurements taken during the Test Plan execution.
* **Implementation status is reported honestly.** The table in §5
  above, the firmware-mapping tables in
  [`SFG_Report.md`](SFG_Report.md) §8 and
  [`Control_System_Analysis.md`](Control_System_Analysis.md) §4.1, and
  Slide 13 of [`Presentation_Outline.md`](Presentation_Outline.md)
  each separately disclose which blocks of the target architecture are
  active today and which are still planned.
* **Empirical testing remains ongoing.** The test log template at the
  end of [`Test_Plan.md`](Test_Plan.md) §12 is intentionally empty in
  this revision; it will be populated as tests are executed and
  signed off.

---

## 7. Suggested Reading Order for Evaluators

The documents are written to be read in this order; each builds on the
previous one's context without strictly requiring it.

1. **[`README.md`](README.md)** — *(this file)* understand the
   project's scope, status, and document map.
2. **[`Presentation_Outline.md`](Presentation_Outline.md)** — get the
   ten-minute narrative arc and the team's framing of what is and is
   not claimed.
3. **[`SFG_Report.md`](SFG_Report.md)** — see the theoretical control
   model that the rest of the documentation rests on.
4. **[`Control_System_Analysis.md`](Control_System_Analysis.md)** —
   companion to the SFG: PID design intent, sensor-filtering rationale,
   and code-to-theory mapping.
5. **[`Test_Plan.md`](Test_Plan.md)** — the bridge from theory to
   measurement; defines what success will look like in the validation
   phase.
6. **[`BOM.md`](BOM.md)** — hardware reference, useful for sanity
   checking the design against cost and component-choice questions.
7. **[`Glossary.md`](Glossary.md)** — quick-reference for any term
   above that needs a one-paragraph refresher.
