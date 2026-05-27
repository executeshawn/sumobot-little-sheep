# LittleSheep Sumobot — Documentation Index

**Course:** CPE335 Sumobot Challenge
**Platform:** ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM), Arduino
framework on PlatformIO.
**Status:** Hardware bring-up complete; motor and sensor subsystems
validated. Project is entering the validation + tuning phase.

This repository holds the firmware, hardware design notes, and academic
documentation for an autonomous differential-drive sumo robot built
around a 6-channel VL53L0X opponent-detection ring, a Pololu
QTR-MD-03RC edge array, two TT geared motors driven by a TB6612FNG, and
an ESP32-S3 supervisory controller. The robot is **deterministic**:
behaviour is fixed in firmware (no DIP-switch mode select, no status
LED).

---

## 1. Project Overview

The LittleSheep is organised in two cooperating layers:

* **Supervisory FSM** — `IDLE → CALIBRATE → COUNTDOWN → SEARCH → ATTACK
  → EVADE → STOP`. The state machine reads cached sensor data and
  issues fixed-PWM motor commands. EVADE pre-empts every other active
  state on any edge event. ATTACK has a 10 s watchdog that reverts to
  SEARCH if no transition happens.
* **Reactive sensor layer** — Multi-rate non-blocking cache.
  `Edge::poll()` runs every 8 ms, `ToF::pollFront()` every 30 ms,
  `ToF::pollSide()` every 90 ms, `Strategy::update()` every 5 ms. The
  FSM only ever reads the cache, so no FSM tick is blocked on a sensor
  I/O round-trip.

The architecture is intentionally **open-loop reactive**: there is no
PID inner loop and no encoder/IMU feedback in the current build. The
SFG and control-system documents describe both the realised reactive
controller *and* the closed-loop PID architecture that the hardware is
"PID-ready" to receive once an inner-loop sensor is added. See
[`SFG_Report.md`](SFG_Report.md) and
[`Control_System_Analysis.md`](Control_System_Analysis.md) for an
honest treatment of what is implemented vs. what remains theoretical.

---

## 2. Documentation Map

| File | Purpose |
|---|---|
| [`README.md`](README.md) | This index. Entry point for evaluators. |
| [`BOM.md`](BOM.md) | Finalised bill of materials — validated components only, with per-member cost split. |
| [`SFG_Report.md`](SFG_Report.md) | Block diagram, signal flow graph, and Mason's gain derivation for the target PID architecture, with explicit mapping to the current reactive implementation. |
| [`Control_System_Analysis.md`](Control_System_Analysis.md) | Honest control-system analysis: what is implemented (deterministic FSM + reactive sensor logic), what is "PID-ready", and what remains theoretical. |
| [`Test_Plan.md`](Test_Plan.md) | Validation procedures, structured around the PlatformIO bring-up firmware environments. |
| [`Presentation_Outline.md`](Presentation_Outline.md) | 10-minute oral-defense deck structure with speaker notes. |
| [`Glossary.md`](Glossary.md) | Quick-reference definitions for oral defense. |

---

## 3. Hardware Architecture (final, validated)

### 3.1 Core components

| Subsystem | Part | Notes |
|---|---|---|
| MCU | ESP32-S3 N16R8 (DevKitC-1) | 16 MB flash, 8 MB **octal** PSRAM (`qio_opi`). PSRAM bus blocks GPIO 26–37 and constrains 15/16/17. |
| Motor driver | TB6612FNG dual H-bridge | 1.2 A continuous / 3.2 A peak per channel; 10 kΩ pulldown on STBY. |
| Motors | TT geared motors, 6 V, ≈ 160 RPM, L-shape (yellow gearbox) | Replaces the original N20 plan. Higher torque, lower top speed, easier mechanical mount. |
| Opponent ring | 6 × GY-VL53L0X breakouts (Pololu VL53L0X driver) | Continuous-ranging mode. Addresses re-assigned to 0x30..0x35 via XSHUT bring-up. |
| Edge sensor | Pololu QTR-MD-03RC (3-channel) | RC discharge-timing mode; calibrated 0..1000. |
| Battery | 2S Molicel INR-18650A pack (2 × 18650, holder, 2S BMS) | ~7.4 V nominal, 8.4 V full. |
| Buck converters | 2 × LM2596 (adjustable) | Pack → ≈ 6 V to TB6612FNG VM; pack → 5 V to ESP32. |
| Inputs | Tactile start button on GPIO 1, INPUT_PULLUP | No DIP switches in this revision. |

### 3.2 Final GPIO map (locked)

| Function | GPIO | Notes |
|---|---:|---|
| Motor PWMA (left) | 4 | LEDC ch 0, 5 kHz / 8-bit |
| Motor AIN1 (left dir 1) | 5 | |
| Motor AIN2 (left dir 2) | 6 | |
| Motor PWMB (right) | 7 | LEDC ch 1, 5 kHz / 8-bit |
| Motor **BIN1** (right dir 1) | **47** | Migration: 16/17 → 41/42 → **47/21** |
| Motor **BIN2** (right dir 2) | **21** | See migration note below |
| Motor STBY | 18 | 10 kΩ pulldown to GND |
| I²C SDA | 8 | shared by all 6 VL53L0X |
| I²C SCL | 9 | shared by all 6 VL53L0X |
| XSHUT FL / FC / FR | 10 / 11 / 12 | front-left / center / right |
| XSHUT L / R / RR | 13 / 14 / **2** | RR moved off GPIO 15 to keep it away from the PSRAM bus |
| QTR LEFT / CENTER / RIGHT | **38 / 39 / 40** | RC mode |
| Start button | 1 | INPUT_PULLUP, press = LOW |

#### 3.2.1 BIN1 / BIN2 migration history

The right-motor direction pins moved through three iterations during
bring-up. The history is recorded in
[`lib/Sumobot/pins.h`](../lib/Sumobot/pins.h):

| Iteration | Pins | Outcome |
|---|---|---|
| 1 | 16 / 17 | Interfered with the ESP32-S3 octal PSRAM subsystem (the N16R8's `qio_opi` configuration treats GPIO 15–17 as timing-sensitive PSRAM neighbours). |
| 2 | 41 / 42 | Functional but unreliable as direction outputs on this S3 N16R8 build (intermittent latching). |
| 3 (final) | **47 / 21** | Stable. **This is the locked production map.** |

The same iteration removed GPIO 15 from the XSHUT block: VL53L0X RR
moved from GPIO 15 to GPIO 2 to keep all noisy / timing-sensitive
digital outputs away from the PSRAM bus.

### 3.3 VL53L0X address map (post-bring-up)

| Index | Position | I²C address | XSHUT GPIO |
|---:|---|---:|---:|
| 0 | Front-Left  (FL) | 0x30 | 10 |
| 1 | Front-Centre (FC) | 0x31 | 11 |
| 2 | Front-Right (FR) | 0x32 | 12 |
| 3 | Left  side  (L)  | 0x33 | 13 |
| 4 | Right side  (R)  | 0x34 | 14 |
| 5 | Rear        (RR) | 0x35 |  2 |

---

## 4. Firmware Architecture (final, modular)

The firmware was refactored from a flat `src/` tree into a modular
embedded architecture during the bring-up phase. The production
entry-point now contains only the main loop; every subsystem lives in
the shared `Sumobot` library that PlatformIO's LDF auto-discovers for
every build environment.

```
sumobot-little-sheep/
├── platformio.ini                    Multi-env build config
├── src/
│   └── main.cpp                      Multi-rate poll loop (production firmware entry)
├── lib/Sumobot/                      Shared modules (LDF-discovered)
│   ├── pins.h                        Final GPIO map + ESP32-S3 cautions + migration log
│   ├── config.h                      Thresholds, speed profiles, cadences, timing
│   ├── motors.h / .cpp               TB6612FNG via LEDC PWM; soft + hard speed caps
│   ├── movement.h / .cpp             Open-loop trim + pivot-by-degree (calibration layer)
│   ├── sensors_tof.h / .cpp          6× VL53L0X continuous ranging; non-blocking cache harvest
│   ├── sensors_edge.h / .cpp         QTR-MD-03RC RC mode; cached poll
│   ├── strategy.h / .cpp             Deterministic FSM (7 states, EVADE pre-emption, ATTACK watchdog)
│   └── library.json                  Declares Pololu driver deps for the LDF
├── test/                             Bring-up firmware sketches (see §6)
│   ├── test_motor_diag/   test_motor/
│   ├── test_qtr/
│   ├── test_vl53/   test_vl53_bringup/   test_vl53_verify/
│   ├── test_button/   test_fsm/
│   ├── test_calibration_protocol/
│   └── test_full_system/
└── docs/                             (this directory)
```

### 4.1 Module responsibilities

* **`main.cpp`** — Wires the subsystems together, runs the four-timer
  multi-rate poll loop, latches a fault if zero ToF sensors come
  online.
* **`motors`** — TB6612FNG control via LEDC PWM (5 kHz / 8-bit).
  Authoritative polarity contract (forward = IN1 LOW, IN2 HIGH).
  Exposes `forward / reverse / turnLeft / turnRight / drive / brake /
  stop` plus a **soft cap** (`setSpeedCap`) re-applied each FSM tick and
  a **hard cap** (`setHardCap`) used by bench tests as a persistent
  ceiling.
* **`movement`** — Open-loop calibration abstraction layer over
  `Motors::`. Adds per-channel PWM **trim** to compensate motor
  mismatch, and an empirical **pivot-by-degree** helper based on
  bench-measured ms-per-degree constants. Strategy still calls
  `Motors::` directly today; `Movement::` is wired in for the upcoming
  trim/pivot characterisation pass.
* **`sensors_tof`** — Full XSHUT one-at-a-time bring-up, address
  re-assignment to 0x30..0x35, continuous-ranging start at 25 ms
  (front) / 70 ms (side) inter-measurement period with a 20 ms timing
  budget. `pollFront()` / `pollSide()` harvest into a per-sensor cache
  with a 50 ms read timeout; FSM only ever reads the cache.
* **`sensors_edge`** — QTR-MD-03RC in RC mode, three channels.
  Spin-in-place calibration via repeated `calibrateEdge()` calls during
  the CALIBRATE state. `poll()` updates a cache; `isOnEdge()` and
  `edgeDirection()` are non-blocking queries.
* **`strategy`** — Deterministic FSM. No DIP-switch mode selection.
  EVADE has top priority. STOP pulls STBY low (motor coast, no holding
  current).
* **`config.h` / `pins.h`** — All thresholds, cadences, speed profiles,
  and GPIO assignments. Single source of truth.

### 4.2 Multi-rate poll loop (in [`src/main.cpp`](../src/main.cpp))

| Loop | Period | Purpose |
|---|---:|---|
| `Edge::poll()` | 8 ms | FAST, top priority — RC discharge timing |
| `ToF::pollFront()` | 30 ms | MEDIUM — engagement-critical bearing |
| `ToF::pollSide()` | 90 ms | SLOW — flank/rear awareness |
| `Strategy::update()` | 5 ms | FSM tick — reads caches only |

### 4.3 Calibration protocol framework

The `test_calibration_protocol` environment is a guided bench
procedure that measures and prints the two open-loop calibration
constants used by the `Movement` layer:

1. `MOVE_TRIM_LEFT` / `MOVE_TRIM_RIGHT` — per-channel PWM scaler to
   straighten forward drift (clamped to [0.50, 1.00]; trim can only
   *slow* a motor, never accelerate it).
2. `MOVE_MS_PER_DEG_LEFT` / `MOVE_MS_PER_DEG_RIGHT` — empirical
   pivot-by-degree constants, measured by commanding a 360° pivot at
   `SPEED_TURN` and timing it.

The operator runs the protocol, types observations into Serial, and
copies the printed config block into `lib/Sumobot/config.h`.

### 4.4 Safety mechanisms (present in production firmware)

* `Strategy::setFault()` latches a STOP transition if zero ToF sensors
  come online at boot.
* STOP state pulls TB6612FNG STBY LOW (high-Z bridge — motors coast,
  no holding current).
* EVADE pre-empts every other active state on any edge event, so the
  bot reverses before it can drive off the dohyo at SPEED_ATTACK.
* ATTACK watchdog reverts to SEARCH after 10 s of stuck pursuit.
* Mandatory 5 s COUNTDOWN after CALIBRATE before any motion is
  permitted (rule-compliant pre-match delay).
* `test_full_system` enforces `Motors::setHardCap(130)` (≈ 51 % full
  PWM) and a 60 s autonomous time limit.

---

## 5. Build environments (PlatformIO)

All firmware is built with `pio run -e <env> -t upload`. There is **no
`pio test`** and no Unity framework — each bring-up sketch is an
ordinary Arduino sketch (`setup()` / `loop()`) selected by a per-env
`build_src_filter`.

| Env | Purpose |
|---|---|
| `esp32s3` | Production firmware (`src/main.cpp`). Default env. |
| `test_motor_diag` | Single-channel TB6612FNG diagnostic. Forward / stop / reverse / brake / coast on one motor, then the other; validates polarity, IN1/IN2 wiring, and STBY safety path. |
| `test_motor` | Both channels together at low PWM (90/70). Drift check. |
| `test_calibration_protocol` | Guided procedure that measures and prints trim and ms-per-degree constants for `Movement`. Hard-capped at PWM 90. |
| `test_full_system` | Full integration test on the dohyo. Hard-capped at PWM 130; 60 s autonomous timeout. |
| `test_qtr` | QTR-MD-03RC bring-up, calibration sweep, threshold validation. |
| `test_vl53` | Live VL53L0X ring streaming (post-bring-up). |
| `test_vl53_bringup` | XSHUT one-at-a-time + address reassignment sequence. |
| `test_vl53_verify` | Post-bring-up verification — confirms all six respond at 0x30..0x35. |
| `test_button` | Start-button input path. |
| `test_fsm` | FSM transition simulator — no hardware needed, single-char Serial commands. |

Run the production firmware:

```
pio run -e esp32s3 -t upload
pio device monitor -e esp32s3
```

Run a bring-up:

```
pio run -e test_motor_diag -t upload
pio device monitor -e test_motor_diag
```

See [`../test/README.md`](../test/README.md) for the full per-env
operator guide and recommended bring-up order.

---

## 6. Current Implementation Status

| Area | Status |
|---|---|
| ESP32-S3 final GPIO map (validated, PSRAM-clean) | ✅ Locked |
| TB6612FNG motor driver wrapper (LEDC, soft + hard caps, brake/coast) | ✅ Implemented |
| TT geared motor polarity verified at the bench (both channels) | ✅ Implemented |
| VL53L0X XSHUT bring-up + readdress to 0x30..0x35 | ✅ Implemented |
| VL53L0X continuous ranging + cached harvest | ✅ Implemented |
| QTR-MD-03RC RC-mode calibration + cached poll | ✅ Implemented |
| Multi-rate asynchronous poll loop (8 / 30 / 90 / 5 ms) | ✅ Implemented |
| Deterministic FSM (7 states, EVADE pre-emption, ATTACK watchdog) | ✅ Implemented |
| Movement calibration layer (open-loop trim, pivot-by-degree) | ✅ Implemented (constants pending bench measurement) |
| Bring-up firmware suite (10 envs) | ✅ Implemented |
| Calibration protocol firmware | ✅ Implemented |
| `test_full_system` safety-capped integration | ✅ Implemented |
| Bench measurement of `MOVE_TRIM_*` and `MOVE_MS_PER_DEG_*` | 🟡 Pending |
| Edge survival validation at SPEED_ATTACK (ED-02 in Test Plan) | 🟡 Pending |
| Match-load runtime characterisation (2S pack) | 🟡 Pending |
| PID controller module (`pid.cpp`) | ❌ Not implemented |
| Inner-loop sensor (encoder or IMU) | ❌ Not installed |
| Closed-loop PID heading / velocity control | ❌ Architecturally provisioned only |
| Moving-average / Kalman filter on bearing | ❌ Theoretical only |

Legend: ✅ Implemented · 🟡 In Progress · ❌ Not implemented.

---

## 7. Academic Integrity Statement

* The robot operates today on a **deterministic FSM + reactive sensor
  logic** topology. We do **not** claim full closed-loop PID control.
* No fabricated PID gains, plant parameters, or match results appear
  anywhere in this documentation. Symbolic `Kp, Ki, Kd` are used in the
  theoretical SFG analysis.
* `MOVE_TRIM_*` and `MOVE_MS_PER_DEG_*` are explicitly labelled as
  placeholders until measured at the bench via
  `test_calibration_protocol`.
* `Km`, `τm`, and `K_drv` in the SFG report are datasheet-derived
  assumptions, not measurements.
* The implementation-vs-theory mapping table in
  [`Control_System_Analysis.md`](Control_System_Analysis.md) §4
  separately discloses each block of the target architecture as
  implemented, PID-ready, or theoretical.

---

## 8. Suggested Reading Order

1. **[`README.md`](README.md)** — this index.
2. **[`BOM.md`](BOM.md)** — what the system is physically made of.
3. **[`SFG_Report.md`](SFG_Report.md)** — block diagram, signal flow
   graph, and Mason's gain derivation.
4. **[`Control_System_Analysis.md`](Control_System_Analysis.md)** —
   honest mapping between theory and the realised firmware.
5. **[`Test_Plan.md`](Test_Plan.md)** — validation procedures keyed to
   the PlatformIO environments in [`../test/`](../test/).
6. **[`Presentation_Outline.md`](Presentation_Outline.md)** — defense
   narrative.
7. **[`Glossary.md`](Glossary.md)** — quick reference.
