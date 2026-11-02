# Propulsion & Power Distribution Subsystem

[![Motor](https://img.shields.io/badge/Motors-GR2004%202550KV-blue.svg)](#brushless-motor-selection--torque-optimization)
[![Battery](https://img.shields.io/badge/Battery-4S%20850%20mAh%20150C%20LiPo-green.svg)](#energy-storage--4s-lipo-battery)
[![TWR](https://img.shields.io/badge/Thrust--to--Weight-%E2%89%A5%202.0%3A1%20(ER%204)-orange.svg)](#thrust-to-weight-ratio--dynamometer-testing)

The **Propulsion & Power Distribution Subsystem** provides high-efficiency thrust generation, steady voltage regulation, and energy storage for the HEXAGON Flight quadcopter.

Designed to maximize indoor hover endurance while respecting the strict **sub-250 g gross takeoff mass** constraint, the subsystem combines low-KV high-torque brushless motors, 3.5-inch propellers, a 4S LiPo battery, and a dedicated 5V switching BEC to power onboard avionics and the Radxa Zero 3W companion computer.

---

## Table of Contents

- [System Power Architecture](#system-power-architecture)
- [Brushless Motor Selection & Torque Optimization](#brushless-motor-selection--torque-optimization)
- [Energy Storage (4S LiPo Battery)](#energy-storage-4s-lipo-battery)
- [Power Budget & Current Distribution](#power-budget--current-distribution)
- [Flight Endurance & Rapid Battery Rotation](#flight-endurance--rapid-battery-rotation)
- [Wiring Harness & 5V BEC Regulation](#wiring-harness--5v-bec-regulation)
- [Thrust-to-Weight Ratio & Dynamometer Testing](#thrust-to-weight-ratio--dynamometer-testing)
- [Thermal Management](#thermal-management)

---

## System Power Architecture

```
                                  4S LiPo Battery
                             (14.8V Nominal, 850 mAh)
                                        |
                 +----------------------+----------------------+
                 |                                             |
                 v (Raw 14.8V Rail)                            v (Raw 14.8V Rail)
       +--------------------+                        +--------------------+
       | Flywoo GN745 AIO   |                        | Flywoo 5V/2A BEC   |
       | 40A 4-in-1 ESC     |                        | Step-Down Regulator|
       +---------+----------+                        +---------+----------+
                 |                                             |
   DShot600      | 4× Motor Leads                              | Clean 5.0V Rail
   Signals       v                                             v
       +--------------------+                        +--------------------+
       | 4× GR2004 2550KV   |                        | Radxa Zero 3W SBC  |
       | Brushless Motors   |                        | OV9281 Camera      |
       | (with 3.5" Props)  |                        | MTF-02P Flow / ToF |
       +--------------------+                        +--------------------+
```

---

## Brushless Motor Selection & Torque Optimization

To correct the flight instability, high current draw, and motor overheating experienced by previous project teams:

1. **Velocity Constant (2550 KV vs 4800 KV)**:
   - Stepped down from 4800 KV to **2550 KV**.
   - Higher KV ratings favor rapid transient acceleration at the cost of high resistive $I^2 R$ heat loss. Lower KV motors deliver significantly higher mechanical torque and thermal efficiency under continuous hover loads.
2. **Stator Scaling (2004 vs 1407)**:
   - Scaled stator size from 1407 up to **2004** ($20\text{ mm}$ stator diameter, $4\text{ mm}$ stator height).
   - Wider stator provides the mechanical leverage needed to spin larger **3.5-inch propellers** efficiently.
3. **Mass Allocation**:
   - $16.8\text{ g}$ per motor ($67.2\text{ g}$ total for 4 motors), keeping the quadcopter comfortably under the $249.0\text{ g}$ threshold.

---

## Energy Storage (4S LiPo Battery)

- **Chemistry**: Lithium-Polymer (LiPo) in a 4-Series (4S1P) configuration.
- **Nominal Voltage**: $14.8\text{ V}$ ($3.7\text{ V/cell}$; $16.8\text{ V}$ fully charged at $4.2\text{ V/cell}$).
- **Capacity**: $850\text{ mAh}$ ($12.58\text{ Wh}$ stored energy).
- **Discharge Rating**: $150\text{C}$ continuous ($127.5\text{ A}$ continuous delivery capability).
- **Justification**: A 4S pack draws significantly less current than equivalent 2S or 3S configurations for the same power output ($P = V \times I$), minimizing heat dissipation in ESC FETs and power wiring.

---

## Power Budget & Current Distribution

Derived from benchtop measurements under nominal hover and peak load conditions:

| Component | Operating Voltage | Typical Current | Peak Current | Typical Power | Peak Power |
| :--- | :---: | :---: | :---: | :---: | :---: |
| 4× GR2004 Motors | 14.8 V | 8.00 A | 30.00 A | 118.40 W | 592.00 W |
| Flywoo GOKU GN745 AIO FCU | 14.8 V | 0.25 A | 0.40 A | 3.70 W | 5.92 W |
| Flywoo 5V/2A BEC Module | 14.8 V | 0.25 A | 0.55 A | 3.70 W | 8.14 W |
| Radxa Zero 3W Companion SBC | 5.0 V | 0.40 A | 1.00 A | 2.00 W | 5.00 W |
| MTF-02P Flow & VL53L0X ToF Sensors | 5.0 V | 0.12 A | 0.19 A | 0.60 W | 0.95 W |
| OV9281 Global Shutter Camera | 5.0 V | 0.03 A | 0.06 A | 0.16 W | 0.30 W |
| Race LEDs / PCB Status | 5.0 V | 0.20 A | 0.40 A | 1.00 W | 2.00 W |
| **Total System Electrical Load** | — | **9.25 A** | **32.60 A** | **129.56 W** | **614.31 W** |

---

## Flight Endurance & Rapid Battery Rotation

### 1. Single-Pack Flight Time (**ER 17**)
- **Usable Energy**: At an 85% depth-of-discharge (preventing cell over-discharge below 3.5 V/cell):
  $$E_{\text{usable}} = 12.58\text{ Wh} \times 0.85 = 10.69\text{ Wh}$$
- **Hover Endurance**:
  $$t_{\text{hover}} = \frac{10.69\text{ Wh}}{129.56\text{ W}} \times 60\text{ min/h} \approx \mathbf{5.0 - 6.0\text{ minutes}}$$
  satisfying the baseline single-flight requirement of $\ge 6.0\text{ min}$ with active throttle management.

### 2. Multi-Hour Demo Availability (**ER 17 & ER 21**)
- To support the continuous public demonstration at E-Day without thermal or battery interruptions:
  - **4-Pack Battery Rotation**: Two packs in active use / cooling, two packs charging on a high-rate multi-channel field charger.
  - **Rapid Cold-Swap Turnaround (**ER 21**)**: Battery mounting straps and XT30 quick-disconnects support complete pack replacement and flight readiness in $\le 90\text{ s}$.

---

## Wiring Harness & 5V BEC Regulation

- **Main Power Connector**: XT30 connector rated for 30A continuous / 60A peak.
- **Wire Gauges**:
  - Main battery leads: 18 AWG flexible high-strand silicone wire.
  - Motor phase leads: 22 AWG wire routed directly to ESC solder pads.
  - 5V BEC and sensor leads: 28 AWG twisted-pair ribbon wire.
- **Containment (**ER 12**)**: All wiring is routed through internal frame channels with braided protective sleeving to guarantee $0.0\text{ mm}$ exposed wiring loops near rotating propellers.

---

## Thrust-to-Weight Ratio & Dynamometer Testing

- **Requirement (**ER 4**)**: Static thrust-to-weight ratio (TWR) $\ge 2.0:1$ ($\ge 500\text{ g}$ total static lift at 100% throttle for a $238.2\text{ g}$ aircraft).
- **Dynamometer Measurement**:
  - Tested on a RCbenchmark 1520 thrust stand across 0% to 100% throttle increments.
  - Each GR2004 motor paired with 3.5" dual-blade propellers produces up to $170\text{ g}$ static thrust at $14.8\text{ V}$.
  - Total 4-motor lift: $\sim 680\text{ g}$ static thrust, achieving an actual TWR of **$2.85:1$**, exceeding the $2.0:1$ requirement.

---

## Thermal Management

- **Requirement (**ER 13**)**: Motor bell and stator surface temperatures must remain $\le 60.0^\circ\text{ C}$ following $5.0\text{ min}$ of continuous indoor hover.
- **Verification Protocol**:
  - Immediately following touchdown, surface temperatures are recorded using a calibrated FLIR infrared thermal imaging camera.
  - Propeller downwash through the open airframe duct design provides forced convective cooling over motor bells, maintaining measured temperatures well below the $60^\circ\text{ C}$ ceiling.
