# Chassis & Enclosure Subsystem

[![Material](https://img.shields.io/badge/Airframe%20Material-PETG--CF%20(Carbon%20Fiber)-blue.svg)](#airframe-design--structural-specifications)
[![Mass Target](https://img.shields.io/badge/Airframe%20Mass-%E2%89%A4%2025.0%20g-brightgreen.svg)](#mass--center-of-gravity-budget)
[![Safety Enclosure](https://img.shields.io/badge/Enclosure-6x6x6%20ft%20%7C%20Mesh%20%E2%89%A4%201%22-orange.svg)](#drone-course--safety-enclosure)

The **Chassis & Enclosure Subsystem** encompasses the mechanical structural design of the quadcopter airframe, protective propeller ducts, modular landing gear, and the physical netted flight course enclosure.

The airframe is designed to achieve high structural stiffness and vibrational damping within strict weight constraints, ensuring compliance with the **sub-250 g gross takeoff mass** regulatory limit while protecting spectators and drone components during autonomous flight demonstrations at the University of Kentucky Engineering Day (E-Day).

---

## Table of Contents

- [Airframe Design & Structural Specifications](#airframe-design--structural-specifications)
- [Mass & Center of Gravity Budget](#mass--center-of-gravity-budget)
- [Propeller Duct Clearances & Safety](#propeller-duct-clearances--safety)
- [Drone Course & Safety Enclosure](#drone-course--safety-enclosure)
- [3D-Printed Target Gates](#3d-printed-target-gates)
- [Fabrication & 3D-Printing Parameters](#fabrication--3d-printing-parameters)
- [Testing & Empirical Verification](#testing--empirical-verification)

---

## Airframe Design & Structural Specifications

```
                       [Front]
                     +---------+
         Motor 1 (CW) \       / Motor 2 (CCW)
             ( )=======\=====/=======( )
             |  Duct   |     |  Duct   |
             ( )=======|     |=======( )
                       | SBC |
                       | FCU |
             ( )=======| BAT |=======( )
             |  Duct   |     |  Duct   |
             ( )=======/=====\=======( )
        Motor 4 (CCW) /       \ Motor 3 (CW)
                     +---------+
                        [Rear]
```

### 1. Monolithic Unibody Construction
- The primary airframe is 3D-printed as a single unibody piece without requiring assembly adhesives or support material.
- **Material Selection**: **PETG-CF** (15% chopped carbon-fiber filled PETG).
  - Selected over standard PLA for superior Young’s Modulus (bending stiffness) and enhanced vibrational damping, mitigating high-frequency motor harmonics that interfere with the flight controller's IMU.
- **Electronics Mounting**:
  - $20\times 20\text{ mm}$ M2 mounting pattern for the Flywoo GOKU GN745 AIO flight controller.
  - Dedicated standoffs for the Radxa Zero 3W companion computer.
  - Underside mounting tray for the downward-facing MTF-02P optical flow and VL53L0X ToF rangefinder sensors.

### 2. Modular Landing Gear
- Detachable landing struts connect at the four motor extremities rather than extending from the frame center.
- Provides a wide, stable base during touchdown, preventing tipping during autonomous vertical descents (descent $v_z \le 0.25\text{ m/s}$, **ER 6**).

---

## Mass & Center of Gravity Budget

Strict mass control is mandatory to satisfy **ER 8** ($\le 249.0\text{ g}$ GTOM):

| Component | Budgeted Mass (g) | Actual Mass (g) |
| :--- | :---: | :---: |
| Bare PETG-CF Airframe & Duct Rings | 18.0 | 18.2 |
| Modular Landing Gear Struts (x4) | 4.0 | 3.8 |
| Fasteners, Standoffs, and Battery Strap | 3.0 | 3.0 |
| **Total Structural Allocation** | **25.0 g** | **25.0 g** |

### Center of Gravity Alignment (**ER 5**)
- The physical Center of Gravity (CoG) must align within $\le \pm 3.0\text{ mm}$ of the geometric center of thrust across both X and Y propulsion axes.
- Battery mounting straps allow fine-tuning the longitudinal balance along the X-axis to eliminate pitch trim biases.

---

## Propeller Duct Clearances & Safety

To allow the drone to navigate safely around obstacles and spectators:

1. **Integrated Structural Ducts**:
   - The propeller guards surround all four 3.5-inch propellers, preventing spinning blades from contacting gates, netting, or personnel.
2. **Tip Clearance Specification (**ER 11**)**:
   - Maintains a radial clearance of $\ge 2.0\text{ mm}$ between the spinning propeller blade tips and the inner duct wall under a static lateral side-load of $5.0\text{ N}$.
3. **Internal Wire Routing (**ER 12**)**:
   - All motor phase leads, battery power cables, and sensor ribbons are enclosed within molded frame channels, ensuring $0.0\text{ mm}$ exposed wiring loops.

---

## Drone Course & Safety Enclosure

Demonstrations are conducted exclusively inside a purpose-built safety enclosure (**ER 24**):

```
+-----------------------------------------------------------+
|                   6.0 ft x 6.0 ft PVC CAGE                |
|                                                           |
|       +-------------------------------------------+       |
|       | Overhead Suspension Bar                   |       |
|       |                                           |       |
|       |   [Circle]       [Triangle]     [Square]  |       |
|       |     ( )             /\            [ ]     |       |
|       |                                           |       |
|       +-------------------------------------------+       |
|                                                           |
|   Floor: High-contrast grid surface for optical flow      |
|   Perimeter: Commercial mesh netting (<= 1.0 inch aperture) |
+-----------------------------------------------------------+
```

- **Perimeter Dimensions**: $6.0\text{ ft} \times 6.0\text{ ft} \times 6.0\text{ ft}$ ($1.83\text{ m} \times 1.83\text{ m} \times 1.83\text{ m}$) constructed from rigid PVC pipe.
- **Safety Netting**: High-strength polyethylene mesh with aperture spacing $\le 1.0\text{ in}$ ($20\text{ mm}$) enclosing all four sides and ceiling, completely mitigating flyaway hazards.

---

## 3D-Printed Target Gates

The three mission targets (Circle, Triangle, Square) are suspended from an overhead crossbar:

- **Aperture Clearance (**ER 16**)**: Internal opening size is designed $\ge 25\%$ larger than the drone envelope ($11.25\text{ in} = 0.28575\text{ m}$ inner width), guaranteeing $\ge 5.0\text{ cm}$ clearance between duct edges and gate borders during traversal.
- **Retroreflective Bordering**: Lined with $1.0\text{ in}$ 3M Scotchlite retroreflective tape to bounce concentrated illumination from the drone's forward LED into the OV9281 monochrome camera.
- **Fiducial Tag Anchors**: Surveyed AprilTag markers affixed to gate borders and enclosure corners provide absolute global coordinate fixes to eliminate dead-reckoning drift.

---

## Fabrication & 3D-Printing Parameters

Recommended slicer settings for Bambu Lab / Prusa 3D printers:

| Parameter | Recommended Setting | Justification |
| :--- | :--- | :--- |
| **Material** | Bambu / Prusament PETG-CF | High stiffness and impact resistance |
| **Nozzle Diameter** | 0.4 mm Hardened Steel | Required for abrasive carbon-fiber filaments |
| **Nozzle Temperature** | 255°C – 265°C | Optimal interlayer bonding |
| **Bed Temperature** | 75°C – 80°C | Prevents bed warping on unibody base |
| **Wall Perimeters** | 4 walls (1.6 mm total) | Structural rigidity on duct rings |
| **Infill Density** | 25% Gyroid | Isotropic strength and vibration damping |
| **Layer Height** | 0.16 mm | High geometric accuracy on motor mounts |

---

## Testing & Empirical Verification

- **Free-Fall Drop Test (**ER 2**)**: The fully integrated airframe is dropped from a height of $1.0\text{ m}$ onto foam padding. Success criteria: zero structural fractures, zero cracked duct rings, and zero loosened motor fasteners.
- **Static Deflection Test (**ER 11**)**: Apply $5.0\text{ N}$ lateral force to outer duct rings with a digital force gauge; measure blade clearance with feeler gauges to verify $\ge 2.0\text{ mm}$ clearance.
- **Plumb-Line Balance Test (**ER 5**)**: Suspend the flight-ready quadcopter from a dual-axis pivot; verify CoG within $\pm 3.0\text{ mm}$ of geometric center.
