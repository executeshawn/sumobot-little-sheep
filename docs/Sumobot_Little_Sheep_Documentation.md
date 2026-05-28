# Sumobot Little Sheep Documentation

**Course:** Feedback and Control Systems
**Project:** Sumobot Little Sheep — autonomous mini sumo robot

---

## 1. Introduction

Sumobot Little Sheep is an autonomous robot built for the sumo
challenge. It is designed to find an opponent inside the ring, push
it out, and avoid driving off the edge by itself.

The robot uses:

* **ESP32-S3** as the main controller (it runs the brain of the
  robot)
* **6 × VL53L0X** time-of-flight sensors for opponent detection
* **QTR-MD-03RC** reflectance sensors to detect the white edge of
  the ring
* **TB6612FNG** motor driver to control two TT geared motors

The robot is fully autonomous. Once the start button is pressed, it
runs on its own without remote control.

---

## 2. Block Diagram

This is how the parts connect together:

```
   [ Battery (2S Molicel) ]
            │
            ▼
   [ LM2596 Buck Converter ]
            │
            ▼
   ┌──────────────────────────────┐
   │     ESP32-S3 Controller      │
   │  (reads sensors, decides,    │
   │   sends motor commands)      │
   └──────────────────────────────┘
       ▲          ▲          │
       │          │          ▼
[ VL53 Sensors ]  │   [ TB6612FNG Driver ]
 (6 ToF ring)     │          │
                  │          ▼
            [ QTR Edge ]  [ TT Motors ]
            (3 sensors)    (left + right)

         [ Start Button ] → ESP32-S3
```

Short explanation of each block:

* **Battery + Buck Converter** — gives clean power to both motors
  and the controller.
* **ESP32-S3** — the brain. Reads all sensors and decides what to
  do.
* **VL53 Sensors (6 pcs)** — look outward to find the opponent.
* **QTR Edge Sensors** — look down to see the white ring border so
  the robot won’t fall out.
* **Start Button** — tells the robot to begin the match.
* **TB6612FNG Driver** — receives signals from the ESP32 and powers
  the motors.
* **TT Motors** — turn the wheels left and right.

---

## 3. Signal Flow Graph (SFG)

The robot can be drawn as a simple signal flow graph showing how
information moves from sensors to motors and back.

```
   R(s) ──►(Σ)── E(s) ──► Gc(s) ──► Gm(s) ──► C(s)
            ▲                                   │
            │                                   │
            └─────────── H(s) ◄─────────────────┘
```

### Variables used

| Symbol  | Meaning                                                  |
|---------|----------------------------------------------------------|
| `R(s)`  | Reference / target direction (where the robot should face) |
| `E(s)`  | Error signal (difference between target and actual) |
| `Gc(s)` | Controller gain — the ESP32 decision logic |
| `Gm(s)` | Motor gain — TB6612FNG driver + TT motor response |
| `H(s)`  | Feedback path — sensor reading that goes back to the controller |
| `C(s)`  | Output — actual robot movement / heading |

### Reading the graph

The sensors give the robot a “target direction”. The controller
compares this with what the robot is currently doing (`H(s)`), gets
an error `E(s)`, then sends a command through `Gc(s)` and `Gm(s)`
to produce movement `C(s)`. The sensors read again, and the loop
keeps repeating.

**a) Input node**
`R(s)` — the desired direction, generated from the VL53 sensors
finding the opponent.

**b) Output node**
`C(s)` — the actual heading of the robot after the motors move.

**c) Forward path**
`R(s) → E(s) → Gc(s) → Gm(s) → C(s)`
Forward path gain: `P₁ = Gc(s) · Gm(s)`

**d) Feedback loop**
The output `C(s)` is sensed by `H(s)` and fed back to the summing
node. This makes the loop:
`Gc(s) → Gm(s) → H(s)` back to the input side.
Loop gain: `L₁ = − Gc(s) · Gm(s) · H(s)`

**e) Self-loop**
None. No block feeds directly back into itself.

**f) Path gain (overall)**
The combined effect of `Gc(s) · Gm(s)` going forward, with `H(s)`
acting as the feedback that closes the loop.

**g) Non-touching loops**
Only one loop exists, so there are no non-touching loops.

### Feedback behavior (simple)

* If the robot is **off-target**, `E(s)` is large → motors push
  harder.
* If the robot is **on-target**, `E(s)` becomes small → motors
  ease off.
* The feedback `H(s)` keeps correcting the robot until `R(s)` and
  `C(s)` match.

---

## 4. Mason’s Gain Derivation (simple form)

Mason’s gain formula:

```
T(s) = Σ (Pk · Δk) / Δ
```

For Little Sheep there is only **one forward path** and **one
loop**, so the derivation is short.

### Step 1 — Identify the forward path

```
P₁ = Gc(s) · Gm(s)
```

### Step 2 — Identify the loop gain

```
L₁ = − Gc(s) · Gm(s) · H(s)
```

### Step 3 — Compute Δ (graph determinant)

```
Δ = 1 − (sum of loop gains)
Δ = 1 − ( − Gc(s) · Gm(s) · H(s) )
Δ = 1 + Gc(s) · Gm(s) · H(s)
```

### Step 4 — Compute Δ₁ (cofactor)

The loop touches the forward path, so:

```
Δ₁ = 1
```

### Step 5 — Final transfer function

```
T(s) = ( P₁ · Δ₁ ) / Δ

T(s) = Gc(s) · Gm(s) / ( 1 + Gc(s) · Gm(s) · H(s) )
```

### What each part means in the robot

* **`Gc(s)`** — the ESP32 controller logic (state machine
  decisions in `strategy.cpp`).
* **`Gm(s)`** — the motor driver + TT motor response (TB6612FNG +
  motors).
* **`H(s)`** — the sensor feedback (VL53 ToF readings come back to
  the controller as new input).
* **`T(s)`** — the overall robot response: how the target
  direction `R(s)` becomes the actual heading `C(s)`.

### Edge correction note

The QTR edge sensors are **not** part of this linear loop. They
act as a safety override: when an edge is detected, the controller
jumps directly to the EVADE state and ignores the normal loop
until the robot is safe again.

---

## 5. Relation of SFG to the ESP32 Code

The signal flow graph matches the firmware modules like this:

* **`sensors_tof.cpp`** — reads the 6 VL53 sensors and tells the
  robot where the opponent is.
* **`sensors_edge.cpp`** — reads the QTR sensors to check if the
  robot is near the ring edge.
* **`strategy.cpp`** — the decision-maker. It runs the finite
  state machine: IDLE → CALIBRATE → COUNTDOWN → SEARCH → ATTACK →
  EVADE → STOP.
* **`motors.cpp`** — sends PWM signals through the TB6612FNG to
  spin the motors.
* **`movement.cpp`** — applies small adjustments like drift trim
  and timed pivots before sending commands to `motors.cpp`.

So the SFG’s “sensor → controller → driver → output” path lines up
directly with the actual files in the project.

---

## 6. PID and Tuning Explanation

Right now, the robot is **PID-ready but not running a full PID
loop**. It does not have an encoder or IMU yet, so we don’t do
real closed-loop heading control. Instead, we use simpler tuning
techniques:

* **PWM speed control** — the motor speed is set by changing the
  duty cycle (0–255). Different states use different fixed speeds
  (attack speed, search speed, turn speed).
* **Drift calibration** — both motors don’t spin at exactly the
  same speed, so we apply a small trim value to slow down the
  faster side and make the robot drive straight.
* **Movement tuning** — pivot turns are timed (ms per degree).
  This is measured by hand on the bench.
* **Edge recovery behavior** — when the QTR sensor sees the white
  edge, the robot reverses for a short time and turns away. This
  acts like a safety reaction.
* **Future PID capability** — the firmware is structured so a
  proper PID can be added later once we install an encoder or
  IMU. The motor functions already accept signed speeds, and the
  loop runs at a fixed 5 ms tick — both are perfect for PID.

We are being honest: there is **no real PID running yet**. The
control system today is a deterministic state machine plus
reactive sensor logic.

---

## 7. Sensor Filtering Technique

Instead of doing math filtering, we use **multi-rate polling**
and **caching** to keep readings stable and responsive.

* **Different update intervals** for different sensors:

  | Sensor          | Update Rate | Why            |
  |-----------------|-------------|----------------|
  | QTR (edge)      | every 8 ms  | safety-first   |
  | VL53 front      | every 30 ms | engagement     |
  | VL53 side/rear  | every 90 ms | flank awareness|
  | FSM (decision)  | every 5 ms  | quick reaction |

* **Noise reduction** — the controller only uses cached values,
  not raw I/O. If a sensor is slow or unstable, the previous
  value is still available, so the robot does not freeze.

* **Stable readings** — by separating sensor polling from
  decision-making, the robot reacts quickly to the edge while
  still tracking the opponent smoothly.

* The VL53 sensors give the “where is the enemy” signal, while
  the QTR sensors give the “don’t fall off” signal. Edge always
  has higher priority than opponent.

---

## 8. Technical Presentation Notes

Short bullet answers for the defense.

**Why ESP32-S3 was used**

* Fast, dual-core, with built-in PWM and Wi-Fi.
* Plenty of GPIO pins for all six ToF sensors and three edge
  sensors.
* Can be expanded later (Wi-Fi telemetry, PID tuning, etc.).

**Why VL53L0X sensors were chosen**

* Small, light, and accurate up to about 1 meter.
* Six of them give a full ring of detection without needing a
  spinning sensor or IR mux.
* Easy to address using their XSHUT pin.

**Why TT motors replaced N20 motors**

* TT motors have higher torque, which is what matters most in
  sumo (push power, not top speed).
* They are easier to mount on the chassis.
* The N20s were too weak for the actual robot weight.

**Purpose of the TB6612FNG**

* Acts as the bridge between the ESP32’s low-current signals and
  the actual motor power.
* Allows forward, reverse, brake, and coast.
* Has a standby pin (STBY) that can quickly cut motor power for
  safety.

**Purpose of edge sensors**

* Detect the white border of the ring.
* Trigger an immediate EVADE so the robot does not lose by
  driving off the edge.
* Highest priority in the entire system.

**How the FSM works (simple)**

* IDLE — robot is off, waiting for the button.
* CALIBRATE — slowly spins to learn the edge.
* COUNTDOWN — required 5-second wait before starting.
* SEARCH — looks for the opponent by sweeping.
* ATTACK — drives toward the opponent.
* EVADE — backs away from the edge.
* STOP — safe shutdown.

**Why deterministic timing matters**

* The robot must react fast and predictably.
* Using fixed cycle times (5 ms / 8 ms / 30 ms / 90 ms) avoids
  random delays.
* This keeps the robot safe and consistent during a match.

---

## 9. Conclusion

Sumobot Little Sheep is a complete and working autonomous sumo
robot. It uses an ESP32-S3 to read ToF and edge sensors, then
controls two TT motors through a TB6612FNG driver. The system is
based on a deterministic finite state machine and reactive sensor
logic.

Although a full PID controller is not yet implemented, the
hardware and firmware are designed in a way that a PID loop can
easily be added in the future. For now, the project demonstrates
the most important concepts of feedback and control systems:
sensing, decision-making, actuation, and safe correction.
