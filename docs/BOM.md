# Bill of Materials — Heatseeker Sumobot

**Project:** CPE335 Sumobot Challenge — *Heatseeker*
**Revision:** 0.3 (3-DIP-switch hardware revision, ToF-continuous firmware)
**Prepared for:** Academic submission alongside the SFG report and presentation.

## Notes on pricing

Prices are **estimates from common Philippine electronics suppliers**
(Lazada, Shopee, e-Gizmo, DECO, CD-R King) at the time of writing. They
are intended for budgeting and grading purposes only and will vary with
supplier, stock, and shipping. Convert to USD at the prevailing rate
(≈ ₱56 / USD) if required.

Quantities reflect the **assembled-robot count** plus a minimal spares
allowance where appropriate.

---

## 1. Compute and control electronics

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 1.1 | ESP32-S3-DevKitC-1 N16R8 (16 MB flash, 8 MB octal PSRAM) | 1 | 750 | 750 | Main MCU; runs FSM, sensor polling, motor commands |
| 1.2 | TB6612FNG dual H-bridge breakout (SparkFun-style) | 1 | 200 | 200 | Drives both N20 motors; logic at 3.3 V, motor rail at VM |
| 1.3 | Tactile push-button (12 mm, momentary) | 1 | 20 | 20 | Match start button (5 s countdown trigger) |
| 1.4 | 3-position DIP switch (SPST through-hole) | 1 | 50 | 50 | Behaviour-mode selection (aggressive / spin-search / torque) |
| 1.5 | 5 mm LED + 220 Ω resistor | 1 | 10 | 10 | Status indicator |
| **Subtotal** | | | | **1,030** | |

## 2. Sensing

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 2.1 | VL53L0X Time-of-Flight breakout (GY-VL53L0X) | 6 | 180 | 1,080 | Opponent detection ring (FL, FC, FR, L, R, RR) |
| 2.2 | Pololu QTR-MD-03RC reflectance array | 1 | 800 | 800 | White-ring edge detection (3-channel RC mode) |
| **Subtotal** | | | | **1,880** | |

> If the six VL53L0X breakouts have on-board I²C pull-ups, plan to
> de-solder them on five of the six boards (keep one pair) to avoid an
> over-strong total pull-up. No cost item — design note only.

## 3. Actuation

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 3.1 | N20 micro-metal-gear motor, 6 V / ≈ 1000 RPM no-load | 2 | 350 | 700 | Left and right drive |
| 3.2 | Sumo wheel set (rubber tyre + hub, ≈ 32 mm) | 1 pair | 500 | 500 | Tyre traction |
| **Subtotal** | | | | **1,200** | |

## 4. Power

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 4.1 | 3S 18650 Li-ion pack (3× cells + holder + 3S BMS) | 1 | 800 | 800 | Primary motor + logic supply (≈ 11.1 V nominal) |
| 4.2 | LM2596 adjustable buck converter module | 2 | 100 | 200 | (a) 11.1 V → 7–9 V to TB6612FNG VM; (b) 11.1 V → 5 V to ESP32 expansion board |
| 4.3 | 9 V battery + clip (alternative MCU supply) | 1 | 100 | 100 | Optional separate supply for the ESP32 expansion board barrel jack |
| 4.4 | XT60 connector pair (in-line) | 1 | 100 | 100 | Pack-to-chassis disconnect |
| 4.5 | Toggle switch (rated ≥ 5 A) | 1 | 50 | 50 | Main power isolation |
| **Subtotal** | | | | **1,250** | |

## 5. Passive components and protection

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 5.1 | Electrolytic capacitor 1000 µF / 25 V | 1 | 30 | 30 | Bulk decoupling across TB6612FNG VM/GND (motor inrush absorbance) |
| 5.2 | Electrolytic capacitor 100 µF / 16 V | 2 | 10 | 20 | Decoupling at LM2596 outputs |
| 5.3 | Ceramic capacitor 0.1 µF | 10 | 2 | 20 | High-frequency decoupling at MCU / sensor VCC pins |
| 5.4 | Resistor 10 kΩ ¼ W (pulldowns, pull-ups) | 10 | 1 | 10 | STBY pulldown + miscellaneous |
| 5.5 | Resistor 220 Ω ¼ W (LED) | 5 | 1 | 5 | LED current limit |
| 5.6 | Schottky diode (1N5819 or similar) | 2 | 10 | 20 | Reverse-polarity / freewheel protection |
| **Subtotal** | | | | **105** | |

## 6. Wiring and assembly

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 6.1 | 22 AWG silicone hookup wire (red / black, ≈ 1 m each) | 1 set | 150 | 150 | Power runs |
| 6.2 | Dupont jumper kit (M-M, M-F, F-F, 20 cm) | 1 | 250 | 250 | Sensor and signal wiring |
| 6.3 | Perfboard 7×9 cm (or small custom PCB) | 1 | 100 | 100 | Sensor breakout / pulldown / cap board |
| 6.4 | Heat-shrink assortment | 1 | 80 | 80 | Splice insulation |
| 6.5 | Rosin-core solder + flux (small qty) | 1 | 100 | 100 | Joints |
| **Subtotal** | | | | **680** | |

## 7. Chassis and mechanical

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 7.1 | Chassis (aluminium plate ≤ 20×20 cm, or 3D-printed body) | 1 | 1,000 | 1,000 | Structural frame within sumo size class |
| 7.2 | M3 brass standoffs + screws + nuts (assorted) | 1 set | 150 | 150 | Mounting of boards and sensor brackets |
| 7.3 | Plough / scoop (1 mm steel or aluminium) | 1 | 200 | 200 | Front contact surface |
| **Subtotal** | | | | **1,350** | |

---

## Grand total (estimated)

| Section | Subtotal (₱) |
|---|---:|
| 1. Compute & control electronics | 1,030 |
| 2. Sensing | 1,880 |
| 3. Actuation | 1,200 |
| 4. Power | 1,250 |
| 5. Passives & protection | 105 |
| 6. Wiring & assembly | 680 |
| 7. Chassis & mechanical | 1,350 |
| **Total** | **₱ 7,495** |
| ≈ USD (₱56/USD) | **≈ USD 134** |

A contingency of ~10 % (≈ ₱ 750) is recommended for breakage and spare
sensors during integration testing.

---

## Component justification (short form)

* **ESP32-S3 N16R8** — Selected over the classic ESP32 for the larger
  octal PSRAM (room for future image / lookup-table features) and dual
  LEDC groups; 240 MHz cores are comfortably faster than the firmware
  requires.
* **TB6612FNG** — 1.2 A continuous / 3.2 A peak per channel matches the
  N20 stall envelope with a healthy margin; logic-level decoupled from
  motor rail.
* **6× VL53L0X** — Ring coverage at 6 × 60° resolves opponent bearing
  without the cost of dedicated IR pairs; XSHUT address-reassignment
  removes the need for an I²C multiplexer.
* **QTR-MD-03RC** — Pololu RC array provides reliable
  black-mat / white-ring discrimination with on-chip emitters; three
  channels are sufficient for L / C / R edge classification.
* **N20 6V / 1000 RPM** — Common, well-characterised sumobot motor with
  enough torque at the chosen gear ratio for the weight class.
* **3S 18650 + LM2596** — Standard student-build power topology; the
  ≈ 11.1 V pack with separate bucks keeps logic and motor rails
  independent, addressing the SFG's noise-rejection assumption that the
  controller and driver share a clean reference voltage.

---

## Items deliberately **not** in the BOM (and why)

| Item | Reason |
|---|---|
| Rotary encoders or IMU | Not present in the current revision. Adding either is on the future-improvements list because the SFG's inner velocity / heading feedback path requires one of them to be physically closed. |
| PCF8574 I²C expander | Considered for DIP-switch consolidation; dropped after the switch count was reduced from 6 to 3, which frees enough GPIOs without an expander. |
| Bluetooth / Wi-Fi telemetry hardware | The ESP32-S3 includes both natively; no additional hardware required. Whether to enable telemetry during tuning is a software decision. |
