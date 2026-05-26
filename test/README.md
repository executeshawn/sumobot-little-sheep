# Hardware Bring-Up Tests

This directory contains the PlatformIO test framework for the Heatseeker
sumobot. Each sub-directory is an **independent firmware image** that
initialises only the subsystem under test, prints detailed Serial
diagnostics at **115200 baud**, and is safe to flash and run at the
bench.

These are plain Arduino sketches — `setup()` / `loop()` with serial
diagnostics, nothing else. No test framework, no Unity assertions, no
custom runner, no Python scripts. `pio test -f test_xxx` is used purely
as a build-and-upload selector. PlatformIO automatically excludes
`src/main.cpp` from each test build so the test's own `setup()` /
`loop()` becomes the entry point, while every other module under
`src/` is linked in so each test exercises the real production code.

---

## Tests in this directory

| Test dir | Subsystem | Motors enabled? | Operator action required |
|---|---|---|---|
| [`test_vl53`](test_vl53/test_main.cpp) | 6 × VL53L0X ring | No | Wave a hand / target in front of each sensor |
| [`test_qtr`](test_qtr/test_main.cpp) | QTR-MD-03RC edge array | No | Slide array over black mat and white border; send any key to advance phases |
| [`test_motor_diag`](test_motor_diag/test_main.cpp) | TB6612FNG single-channel diagnostic (one motor at a time) | **Yes (one motor active, low PWM)** | **Lift wheels off the ground / block them** |
| [`test_motor`](test_motor/test_main.cpp) | TB6612FNG combined motion (both channels, drift check) | **Yes (low PWM)** | **Lift wheels off the ground / block them** |
| [`test_button`](test_button/test_main.cpp) | Start button (only remaining UI input) | No | Press button |
| [`test_fsm`](test_fsm/test_main.cpp) | FSM transition simulator | No (no hardware needed at all) | Send single-char commands via Serial monitor |
| [`test_full_system`](test_full_system/test_main.cpp) | Everything integrated | **Yes (hard-capped + 60 s safety timeout)** | Place bot on dohyo with clear space, press start |

---

## Running a test

```bash
pio test -e esp32s3 -f test_vl53
pio test -e esp32s3 -f test_qtr
pio test -e esp32s3 -f test_motor_diag
pio test -e esp32s3 -f test_motor
pio test -e esp32s3 -f test_button
pio test -e esp32s3 -f test_fsm
pio test -e esp32s3 -f test_full_system
```

PlatformIO will compile, upload, and open a serial monitor at 115200
baud. Because these sketches never emit Unity `PASS`/`FAIL` markers,
the test runner will eventually report a timeout — that result is
**harmless and expected**; the diagnostic Serial output streamed during
the run is the deliverable. To keep watching after the runner exits,
open a second monitor in any terminal:

```bash
pio device monitor -b 115200
```

To return to the production firmware afterwards:

```bash
pio run -e esp32s3 --target upload
```

---

## Safety rules

* **`test_motor_diag`** — *Always* lift the wheels off the ground or
  block them. Runs ONE motor at a time at PWM 90/255 through forward,
  stop, reverse, brake, coast — then repeats on the opposite channel —
  then drops STBY mid-drive to validate the coast safety path. Use this
  before `test_motor` to isolate any single-channel wiring fault
  (swapped IN1/IN2, dead PWM line, mis-mapped left/right).

* **`test_motor`** — *Always* lift the wheels off the ground or block
  them. Runs both channels together (forward / reverse / left turn /
  right turn / full stop) at PWM 90/70 out of 255 so any straight-line
  drift between the two motors shows up clearly.

* **`test_full_system`** — Place the robot on the dohyo (or equivalent
  marked surface) with at least 30 cm clear on every side. The test
  enforces `Motors::setHardCap(130)` (≈ 51 % full PWM) and a 60 s
  autonomous time limit, after which a STOP fault is latched. Keep the
  main power switch in reach.

* **All other tests** — Do not enable the motors. No actuation occurs.

---

## Suggested bring-up order

Run the tests in this sequence the first time the bot is assembled or
after any hardware change:

1. **`test_button`** — verifies the start-button input path end-to-end.
2. **`test_vl53`** — confirms the I²C bus and the XSHUT bring-up
   sequence (the riskiest part of the sensor layer).
3. **`test_qtr`** — calibrates and validates the edge threshold, which
   is the safety-critical sensor.
4. **`test_motor_diag`** — single-channel motor diagnostic. Validates
   PWM, direction polarity, left/right mapping, brake, and STBY safety
   on ONE motor at a time. Run this first whenever motor behaviour is
   in question.
5. **`test_motor`** — combined-motion test; both channels run together
   so any drift between the two motors is visible.
6. **`test_fsm`** — sanity-checks the state diagram on the actual MCU
   with no hardware dependencies (useful regression test after any
   `strategy.cpp` edit).
7. **`test_full_system`** — only after every previous test passes.
   This is the final pre-match integration check.

This ordering matches the sensor-bring-up portion of
[`../docs/Test_Plan.md`](../docs/Test_Plan.md) §3–§5 and gives the
operator a safe path from "boards on the desk" to "supervised dohyo
run."

---

## How `test_full_system` enforces its speed cap

The motors module exposes a soft cap (`setSpeedCap`) for runtime
adjustment and a separate **hard cap** (`setHardCap`) used by the
bench tests to enforce a persistent safety ceiling that the FSM cannot
raise:

```cpp
Motors::setHardCap(130);
```

`Motors::drive()` clamps to `min(softCap, hardCap)` on every call, so
`test_full_system` cannot be tricked into running faster than 130/255
no matter what the FSM commands. The production firmware leaves the
hard cap at its default 255 (no extra limit), so this has no effect on
match performance.
