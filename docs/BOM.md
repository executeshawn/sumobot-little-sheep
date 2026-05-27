# Bill of Materials — LittleSheep Sumobot (Finalised)

**Project:** CPE335 Sumobot Challenge — *LittleSheep*
**Revision:** 1.0 (post bring-up; final validated build)
**Prepared for:** Academic submission alongside the SFG report and oral
defense.

This BOM lists **only the components that survived bring-up and are
present on the final assembled robot**. Prototype-phase parts that were
swapped out (N20 motors, 3S 18650 pack, status LED, DIP switches,
temporary jumper wiring, etc.) are deliberately excluded. They are
listed under §8 *Items removed during bring-up* for traceability only;
they are not in the total cost.

## Notes on pricing

Prices are **estimates from common Philippine electronics suppliers**
(Lazada, Shopee, e-Gizmo, DECO, Circuit Rocks) at the time of
preparation. They are intended for budgeting and grading purposes only
and will vary with supplier, stock, and shipping. Convert to USD at the
prevailing rate (≈ ₱56 / USD) if required.

Quantities reflect the **assembled-robot count**; spares budgets are
called out separately in §7.

---

## 1. Compute and Control Electronics

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 1.1 | ESP32-S3-DevKitC-1 **N16R8** (16 MB flash, 8 MB octal PSRAM) | 1 | 850 | 850 | Main MCU; runs FSM, sensor polling, motor commands |
| 1.2 | TB6612FNG dual H-bridge breakout | 1 | 200 | 200 | Drives both TT motors; logic 3.3 V, motor rail VM |
| 1.3 | Tactile push-button (12 mm, momentary) | 1 | 20 | 20 | Match start button (5 s countdown trigger) |
| **Subtotal** | | | | **1,070** | |

*No status LED. No DIP switches. Behaviour is deterministic and fixed
in firmware; mode-selection hardware was removed during the refactor.*

## 2. Sensing

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 2.1 | GY-VL53L0X Time-of-Flight breakout | 6 | 180 | 1,080 | Opponent-detection ring (FL, FC, FR, L, R, RR) |
| 2.2 | Pololu QTR-MD-03RC reflectance array | 1 | 850 | 850 | White-ring edge detection, 3-channel RC mode |
| **Subtotal** | | | | **1,930** | |

> Five of the six VL53L0X breakouts have one of the on-board I²C
> pull-ups de-soldered so the parallel pull-up strength on the shared
> bus stays within the VL53L0X bus-loading spec. No cost item — design
> note only.

## 3. Actuation

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 3.1 | **TT geared motor, 6 V / ≈ 160 RPM, L-shape (yellow gearbox)** | 2 | 200 | 400 | Left and right drive |
| 3.2 | Wheel set compatible with TT motor shaft (rubber tyre + plastic hub) | 1 pair | 300 | 300 | Tyre traction |
| **Subtotal** | | | | **700** | |

The TT geared motors replace the original N20 plan after bench testing:
the higher gear reduction gives better push-torque for the sumo task,
and the L-shape body simplifies chassis mounting at the cost of a lower
top speed. The original N20 motors are not on the assembled robot and
are not in this BOM.

## 4. Power

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 4.1 | **Molicel INR-18650A** cell, 3.7 V nominal | 2 | 350 | 700 | 2S pack — primary motor + logic supply |
| 4.2 | 2S 18650 holder | 1 | 80 | 80 | Mechanical pack housing |
| 4.3 | 2S Li-ion BMS (protection + balance) | 1 | 120 | 120 | Over-discharge / short-circuit protection |
| 4.4 | LM2596 adjustable buck converter module | 2 | 100 | 200 | (a) pack → ≈ 6 V to TB6612FNG VM; (b) pack → 5 V to ESP32 |
| 4.5 | XT60 connector pair (in-line) | 1 | 100 | 100 | Pack-to-chassis disconnect |
| 4.6 | Toggle switch (rated ≥ 5 A) | 1 | 50 | 50 | Main power isolation |
| **Subtotal** | | | | **1,250** | |

The 2S Molicel pack (≈ 7.4 V nominal, 8.4 V full charge) replaces the
earlier 3S plan. The buck-down ratio to 6 V at the TB6612FNG VM rail is
gentler, the cell count is smaller, and the genuine Molicel
INR-18650A's flat discharge curve gives a near-constant motor rail
through most of the match.

## 5. Passive Components and Protection

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 5.1 | Electrolytic capacitor 1000 µF / 25 V | 1 | 30 | 30 | Bulk decoupling across TB6612FNG VM/GND (motor inrush) |
| 5.2 | Electrolytic capacitor 100 µF / 16 V | 2 | 10 | 20 | LM2596 output decoupling |
| 5.3 | Ceramic capacitor 0.1 µF | 10 | 2 | 20 | High-frequency decoupling at MCU / sensor VCC |
| 5.4 | Resistor 10 kΩ ¼ W | 5 | 1 | 5 | STBY pulldown + miscellaneous |
| 5.5 | Schottky diode (1N5819 or similar) | 1 | 10 | 10 | Reverse-polarity / freewheel protection |
| **Subtotal** | | | | **85** | |

## 6. Wiring and Mechanical

| # | Item | Qty | Unit (₱) | Subtotal (₱) | Function |
|---|---|---:|---:|---:|---|
| 6.1 | 22 AWG silicone hookup wire (red / black, ~1 m each) | 1 set | 150 | 150 | Power runs (pack, VM, 5 V, GND) |
| 6.2 | Dupont jumper kit (M-M, M-F, F-F, 20 cm) | 1 | 200 | 200 | Permanent sensor / signal wiring |
| 6.3 | Perfboard 7×9 cm | 1 | 100 | 100 | Sensor breakout / cap board |
| 6.4 | Heat-shrink assortment | 1 | 80 | 80 | Splice insulation |
| 6.5 | Rosin-core solder + flux | 1 | 100 | 100 | Joints |
| 6.6 | Chassis (3 mm acrylic plate / 3D-printed base, ≤ 20×20 cm) | 1 | 600 | 600 | Structural frame within sumo size class |
| 6.7 | M3 brass standoffs + screws + nuts (assorted) | 1 set | 150 | 150 | Mounting of boards and sensor brackets |
| 6.8 | Plough / scoop (1 mm steel or aluminium sheet) | 1 | 200 | 200 | Front contact surface |
| **Subtotal** | | | | **1,580** | |

Temporary breadboard-phase wiring (test leads, alligator clips,
breadboards) is excluded; only the wiring on the final assembled
chassis is in this section.

---

## 7. Grand Total and Per-Member Contribution

| Section | Subtotal (₱) |
|---|---:|
| 1. Compute & control electronics | 1,070 |
| 2. Sensing | 1,930 |
| 3. Actuation | 700 |
| 4. Power | 1,250 |
| 5. Passives & protection | 85 |
| 6. Wiring & mechanical | 1,580 |
| **Validated system total** | **₱ 6,615** |
| ≈ USD (₱56/USD) | **≈ USD 118** |

Recommended **10 % contingency** for breakage / spare sensors during
the validation phase: **₱ 660** (already incurred sensors are NOT in
this contingency — they were consumed during bring-up and are not part
of the assembled-system total).

### 7.1 Per-member contribution

Equal split of the validated system total:

| Team size | Per-member share (₱) | Per-member share (USD) |
|---:|---:|---:|
| 3 | 2,205 | ≈ 39 |
| 4 | 1,654 | ≈ 30 |
| 5 | 1,323 | ≈ 24 |
| 6 | 1,103 | ≈ 20 |

If contributions are unequal, record the actual split in the project
submission cover sheet. The same table with the 10 % contingency
included:

| Team size | Per-member share incl. contingency (₱) |
|---:|---:|
| 3 | 2,425 |
| 4 | 1,819 |
| 5 | 1,455 |
| 6 | 1,213 |

---

## 8. Items Removed During Bring-Up (excluded from total)

These parts were on the original prototype BOM but are **not** on the
assembled robot. They are listed here for traceability — and to make
clear what is *not* claimed in the final cost.

| Removed item | Reason |
|---|---|
| N20 micro-metal-gear motors (6 V / 1000 RPM) | Replaced by TT geared 6 V / 160 RPM motors for higher push-torque. |
| 3S 18650 pack (3 × cells) | Replaced by a 2S Molicel INR-18650A pack. Lower buck-down ratio, fewer cells, flat discharge curve. |
| 9 V battery + clip | Removed; the 2S pack + LM2596 5 V rail powers the ESP32 directly. |
| 3-position DIP switch + 5 mm status LED | Behaviour is deterministic and fixed in firmware — no mode-select hardware is needed. |
| Temporary breadboard wiring / test leads | Consumables of the bench phase; not part of the assembled robot. |
| Pololu rotary encoders / IMU | Never installed. Listed as "not in the BOM" to make the open-loop nature of the current build explicit. |

---

## 9. Component Justification (short form)

* **ESP32-S3 N16R8** — Chosen for its dual LEDC groups (clean motor PWM
  + non-blocking sensor I/O on the same MCU), Wi-Fi for future
  telemetry, and the 8 MB octal PSRAM (head-room for future filters /
  lookup tables). The octal PSRAM bus constrains the GPIO map; the
  final pin assignments are validated against this constraint and
  recorded in [`pins.h`](../lib/Sumobot/pins.h) with the migration
  history.
* **TB6612FNG** — 1.2 A continuous / 3.2 A peak per channel matches
  the TT-motor stall envelope with comfortable margin; logic-level
  decoupled from the motor rail.
* **6 × GY-VL53L0X** — Ring coverage at 6 × 60° gives directional
  opponent detection without the cost of a dedicated IR-pair tower;
  XSHUT one-at-a-time bring-up to 0x30..0x35 removes the need for an
  I²C multiplexer. Driven by the Pololu VL53L0X library.
* **Pololu QTR-MD-03RC** — Reflectance array in RC mode gives reliable
  black-mat / white-ring discrimination with on-board emitters; three
  channels are sufficient for LEFT / CENTER / RIGHT classification.
* **TT geared 6 V / 160 RPM** — Higher torque than the N20 at the
  weight class, simpler chassis mount, lower top speed (which is fine
  for the sumo task because acceleration and push matter more than
  top-end velocity in a 3 m dohyo).
* **2S Molicel INR-18650A + LM2596** — Genuine Molicel INR-18650A cells
  give a high continuous discharge rating and a flat voltage curve, so
  the motor rail stays close to nominal through most of the match. The
  2S configuration drops the buck-down ratio to ≈ 6 V VM, easing the
  LM2596's switching duty.
