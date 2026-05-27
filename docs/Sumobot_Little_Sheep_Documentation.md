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
   R(s) ──► (Σ) ──► Controller ──► Motor Driver ──► Motors ──► Y(s)
              ▲                                                  │
              └────────────── Feedback ──────────────────────────┘
```

### Reading the graph

The sensors give the robot a “target direction”. The controller
compares this with what the robot is doing now, then sends a
command to the motors. The motors move the robot, and the sensors
read again. This loop keeps repeating.

**a) Input node (R)**
The desired direction the robot should face. It comes from the
VL53 sensors finding the opponent.

**b) Output node (Y)**
The actual heading of the robot — where it is really facing.

**c) Forward path**
Sensor reading → ESP32 decision → motor driver → motors → robot
movement.

**d) Feedback loop**
The robot keeps reading the sensors again after moving. This
becomes the feedback that updates the next decision.

**e) Self-loop**
None in this simple version. The robot does not feed any value
directly back into itself.

**f) Path gain**
The “gain” of the path is just the combined effect of:
controller logic × motor driver × motor response.

**g) Non-touching loops**
There is only one feedback loop, so there are no non-touching
loops.

---

## 4. Mason’s Gain Derivation (simple form)

Mason’s gain formula gives the overall transfer of the system:

```
T(s) = (sum of forward paths × cofactors) / Δ
```

For Little Sheep, there is:

* **One forward path** — sensor → controller → driver → motor →
  output.
* **One loop** — the negative feedback from the output back to the
  controller.

So the equation reduces to the basic closed-loop form:

```
T(s) = G(s) / (1 + G(s))
```

Where `G(s)` is the combined gain of the controller, driver, and
motors.

Relating this to the robot:

* **Sensor input** — VL53 ToF sensors give the opponent direction.
* **ESP32 processing** — `strategy.cpp` decides the next action.
* **Motor response** — TB6612FNG + TT motors produce the actual
  movement.
* **Edge correction** — QTR sensors act as a safety override.
  Whenever the edge is detected, the system jumps to the EVADE
  state, no matter what the inner loop is doing.

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
