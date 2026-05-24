# Presentation Outline — Heatseeker Sumobot

**Course:** CPE335 Sumobot Challenge
**Target length:** 10 minutes (≈ 60 seconds per slide, 14 slides)
**Honest framing rule:** The robot is presented as a *system under
iterative testing and tuning*, not as a finished, fully-validated
competition unit. Every slide that touches PID, performance, or match
behaviour must say so.

---

## Slide 1 — Title (≈ 15 s)

* Project name: **Heatseeker — Autonomous Sumobot**
* Course code, team members, date
* One-line tag: "ESP32-S3 differential-drive sumobot with ToF opponent
  ring, RC edge array, and a PID motion controller currently in tuning."
* Speaker note: *"We will walk through the design, the control-system
  analysis, and the current testing status."*

## Slide 2 — Objectives (≈ 45 s)

Bullet objectives, in priority order:

1. Build a competition-class sumobot within the size/weight rules.
2. Implement a robust supervisory FSM (IDLE → CALIBRATE → COUNTDOWN →
   SEARCH → ATTACK → EVADE → STOP).
3. Provide a sensor architecture that prioritises **edge survival** over
   opponent pursuit.
4. Design (not yet measure) a PID inner loop and analyse it via SFG /
   Mason's gain formula.
5. Document hardware, firmware, and control theory to academic standard.

Speaker note: *"Objective 4 is explicitly design-level; tuning data is
collected in the next phase."*

## Slide 3 — System Architecture (≈ 60 s)

Block diagram (insert image) showing three layers:

* **Supervisory** — FSM in `strategy.cpp`.
* **Outer loop** — ToF ring drives the reference; QTR edge issues
  pre-emptive overrides.
* **Inner loop** — PID + TB6612FNG + N20 motors (architecturally
  provisioned; gains untuned).

Speaker note: *"The outer two layers are implemented and exercised on
hardware. The inner PID is wired in but currently issues fixed-PWM
commands."*

## Slide 4 — Hardware Overview (≈ 45 s)

Photograph / render + bullet inventory:

* ESP32-S3 N16R8 main controller
* TB6612FNG dual H-bridge driver
* 6 × VL53L0X opponent-detection ring
* QTR-MD-03RC 3-channel edge array
* Two N20 6 V / 1000 RPM gear motors
* 3S 18650 pack with separate LM2596 buck rails for motor and logic

Cross-reference to `docs/BOM.md` for full bill of materials.

## Slide 5 — ESP32-S3 Pin Mapping (≈ 45 s)

Compact pin table; emphasise that the mapping was **validated against
ESP32-S3 N16R8 restrictions**:

* Motors: GPIO 4–7, 16–18
* I²C bus: GPIO 8, 9
* VL53L0X XSHUT: GPIO 10–15
* QTR edge: GPIO 38, 39, 40 (no PSRAM-bus conflicts)
* DIP switches: GPIO 41, 42, 47
* Start button / LED: GPIO 1, 2

Speaker note: *"GPIO 33–37 are reserved by octal PSRAM and are not used.
GPIO 0 and 48 are deliberately avoided due to boot strapping and
on-board RGB LED conflicts."*

## Slide 6 — Finite-State Machine (≈ 75 s)

State diagram (insert mermaid render):

```
IDLE ── btn ──► CALIBRATE ── done ──► COUNTDOWN ── 5 s ──► SEARCH
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

* IDLE = motors off, slow LED blink.
* CALIBRATE spins slowly so the QTR sees both mat and ring.
* COUNTDOWN enforces the mandatory 5 s start delay.
* SEARCH alternates between **sweep** and **spin** modes (DIP-selected).
* ATTACK uses ToF bearing to steer; 10 s watchdog forces re-search.
* EVADE has top priority and pre-empts ATTACK.

## Slide 7 — PID Concept (≈ 60 s)

* Continuous control law: `u(t) = Kp·e + Ki·∫e dt + Kd·de/dt`.
* Role of each term in the sumo context:
  * **P** — primary corrective effort proportional to bearing error.
  * **I** — eliminates steady-state offset against constant push.
  * **D** — damps oscillation during fast direction changes.
* **Honest disclaimer slide footer:** *"Numerical gains will be obtained
  experimentally during the tuning phase; no fabricated values are
  presented in this report."*

## Slide 8 — SFG and Transfer Function (≈ 90 s)

Insert the SFG (see [`SFG_Report.md`](SFG_Report.md) §4.2).

* One forward path:
  `P₁ = Gc(s)·K_drv·Gm(s)·Gr(s)`
* One loop:
  `L₁ = −P₁`
* Mason's formula:
  `T(s) = P₁·Δ₁ / Δ = G / (1 + G)`
* Closed-form denominator:
  `τm·s³ + s² + K_drv·Km·(Kd·s² + Kp·s + Ki)`

Speaker note: *"All parameters are symbolic. The SFG describes the
target control architecture; the document explicitly states which
elements are present in firmware today and which arrive with the PID
tuning phase."*

## Slide 9 — Sensor Fusion Strategy (≈ 60 s)

* **Priority hierarchy:** edge > front ToF > side/rear ToF.
* The FSM checks `Edge::isOnEdge()` *before* any ToF logic in
  `SEARCH`, `ATTACK`, `EVADE` states.
* Multi-rate polling, configured in `config.h`:
  * Edge — every **8 ms**
  * Front ToF — every **30 ms**
  * Side / rear ToF — every **90 ms**
* FSM tick runs every 5 ms on **cached** sensor data (non-blocking).

## Slide 10 — Edge Detection Logic (≈ 45 s)

* QTR-MD-03RC in RC (digital discharge-timing) mode.
* White ring → low calibrated reading; threshold = 250 / 1000.
* Direction classification: `LEFT`, `CENTER`, `RIGHT`, `FRONT`.
* EVADE = (1) reverse, (2) pivot away from the side that tripped,
  (3) re-check, then return to SEARCH.

## Slide 11 — VL53L0X Continuous Ranging (≈ 45 s)

* Six sensors brought up sequentially via XSHUT (0x29 default →
  reassigned to 0x30–0x35).
* Continuous-ranging mode started at boot (front 25 ms, side 70 ms
  period).
* `pollFront()` / `pollSide()` non-blockingly harvest the latest
  completed result into a cache; the FSM only reads the cache.
* Replaces the previous blocking single-shot read (~30 ms × 6 ≈ 200 ms
  per cycle) — see SFG §3 timing notes.

## Slide 12 — Disturbance Rejection (≈ 45 s)

Two complementary mechanisms:

1. **Linear (planned, post-tuning):** the integrator in the PID + the
   kinematic integrator drive `Y(jω)/D(jω) → 0` at low frequency,
   rejecting sustained pushes.
2. **Non-linear (active now):** the supervisory layer treats edge
   detection as a *priority interrupt*, pre-empting motor commands
   regardless of inner-loop state.

This dual approach is the project's main design hypothesis for surviving
opponent contact.

## Slide 13 — Testing Status & Limitations (≈ 75 s) **(honesty slide)**

Bullet inventory of what is and is not validated:

* ✅ Firmware compiles and exercises every state on bench power.
* ✅ Sensor bring-up (6 ToF + 3 QTR) confirmed via I²C scan and live
  Serial logging.
* ✅ FSM transitions verified by trigger inputs.
* 🟡 Motor PWM exercised on bench; trajectory not yet characterised
  under load.
* ❌ PID gains **not yet experimentally tuned**; current motor commands
  are fixed PWM values.
* ❌ No real-match validation; no opposing-bot bench tests beyond a
  cardboard target.
* ❌ No encoder / IMU — the SFG's inner feedback loop is therefore not
  physically closed today.

Speaker note: *"This slide is here intentionally — the system is honest
about what has and has not been demonstrated."*

## Slide 14 — Future Improvements & Conclusion (≈ 30 s)

Future work, ranked by impact on the SFG model:

1. Add wheel encoders or an IMU to physically close the inner heading
   loop assumed in the SFG.
2. Run the PID-tuning workflow (Section 5 of
   `Control_System_Analysis.md`): manual ZN, then verification against
   the analytic transfer function.
3. Move to a moving-average or Kalman filter on the ToF bearing.
4. Telemetry over Wi-Fi for live PID-tuning sessions.

Conclusion: *"The Heatseeker presents a complete and defensible design
through to the control-law analysis. The next milestone is empirical
identification of `Km` and `τm` on the actual hardware, followed by PID
tuning against the transfer function derived in this report."*

---

## Appendix — Speaking-time budget

| Slide | Topic | Target time (s) |
|---|---|---:|
| 1 | Title | 15 |
| 2 | Objectives | 45 |
| 3 | Architecture | 60 |
| 4 | Hardware | 45 |
| 5 | Pin map | 45 |
| 6 | FSM | 75 |
| 7 | PID concept | 60 |
| 8 | SFG | 90 |
| 9 | Sensor fusion | 60 |
| 10 | Edge logic | 45 |
| 11 | ToF continuous | 45 |
| 12 | Disturbance rejection | 45 |
| 13 | Testing status | 75 |
| 14 | Future & conclusion | 30 |
| | **Total** | **735 s ≈ 12 min 15 s** |

Trim slides 5 and 10 to ~30 s each (or merge slides 9-11) if the venue
enforces a strict 10-minute cap.

---

## Q&A preparation cues

* *"Have you tuned the PID?"* — "No; the report deliberately presents
  symbolic gains. The SFG and Mason's-formula derivation give us the
  analytic starting point for ZN tuning once we add an encoder or IMU."
* *"Why no encoder?"* — "Scope decision for revision 0.3. The SFG
  documents exactly which feedback path is missing, and it is the first
  item in the future-work list."
* *"Why so many ToF sensors?"* — "Six × 60° ring gives bearing without
  an I²C multiplexer; address reassignment is handled by the XSHUT
  bring-up sequence."
* *"What protects the robot from driving off the ring during a slow
  ToF read?"* — "The edge sensor is polled at 8 ms in a separate fast
  loop and checked first by the FSM, *before* any ToF logic."
