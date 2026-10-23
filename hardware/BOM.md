# Jensen Hand — Bill of Materials

> **Design-stage BOM.** The hand has not been built. Quantities were counted by
> fully expanding the sub-assemblies of
> [`assembly/Jensen_Hand_Assembly.step`](assembly/Jensen_Hand_Assembly.step), so
> they say what the CAD contains, not what has been purchased, fitted or
> tested. No supplier, price or part-number validation has been done. Part
> descriptions follow the names used in the CAD library.

## 1. Actuators

| Item | Qty (CAD) | Role in CAD | Notes |
|---|---:|---|---|
| TowerPro SG90 micro servo | 5 | Four finger servos (in a row on the palm servo plates, at the finger pitch) and one at the thumb | Nominal: about 1.8 kg·cm at 4.8 V, about 0.65 A stall, plastic gears. Clone parts vary; verify against your supplier. |
| MG996R servo | 1 | Output horn (`Arm 1`) attaches to the palm base, so it is positioned as the wrist rotator | Nominal: about 10 kg·cm, about 2.5 A stall, metal gears. |
| 5 V vibration motor module | 1 | Haptic feedback (GPIO 14) | Not an actuator for the hand. |
| Ball-jointed tie rod (`Tie Rod v3`, with `Ball` ends) | 10 | Rigid push–pull links in the servo-to-finger transmission | Reference model: `components/Tie_Rod.step`. Length, thread and material are not recorded in the CAD. |

## 2. Electronics

| Item | Qty (CAD) | Used by firmware | Notes |
|---|---:|---|---|
| ESP32 DevKit (30-pin) | 1 | Yes | `components/ESP32_DevBoard.stp` |
| ESP32 expansion board | 1 | Yes | `components/ESP32_Expansion_Board.step` |
| PCA9685 16-channel PWM driver | 1 | Yes | I2C 0x40; `components/PCA9685_PWM_Driver.stp` |
| DC–DC step-down converter, 5–40 V in, 9 A | 1 | Yes | Servo and logic rail; `components/Buck_Converter_5-40V_9A.stp` |
| 2S Li-ion pack, 7.4 V, 1500 mAh | 1 | Yes | Battery sense on GPIO 34 |
| Nextion 2.4" HMI display | 1 | Yes | `components/Nextion_2.4in_Display.step` |
| Voice Recognition V3 module | 1 | Reserved (UART1, GPIO 18/5) | Optional |
| LM393 comparator module | 5 (3 + 2 mirrored) | 3 contact inputs (GPIO 25/19/23) | CAD has 2 more than the firmware reads |
| FSR400 force-sensitive resistor | 4 | 3 (thumb, index, middle; GPIO 35/32/33) | CAD has 1 more than the firmware reads |
| Green LED, 5 mm | 3 | 1 status LED (GPIO 13) | `components/LED_Green_5mm.stp` |
| 12 mm metal momentary push button | 1 | Yes (GPIO 4) | Mode button |
| USB Type-C jack socket | 1 | — | Charging / programming |
| 2× USB 3.0 jack socket | 1 | — | Role not documented in the CAD |
| DC connector 5.5 mm, IP67, female | 1 | — | Alternative power input |

## 3. Required by the firmware but not modeled in the CAD

| Item | Qty | Notes |
|---|---:|---|
| sEMG front end (electrodes, amplifier, conditioning) | 2 sites (up to 4) | Flexor on GPIO 36, extensor on GPIO 39, ADC1 only. No part is specified. The repository provides no electrode isolation or IEC 60601 compliance; see `current_and_emg_expansion.md`. |
| Battery divider resistors | 1 pair | Reference 100 kΩ and 22 kΩ; measure the fitted values and correct `kBatteryDividerRatio`. |
| Bulk capacitor on the servo rail | 1 | At least 1000 µF. |
| Current-sense channels | 0–7 | Optional, per servo; needs an external ADC on the reference board. |
| Wiring, connectors, solder | — | Not specified. |

## 4. Structural parts

How these are made (3D print, laser cut, machined) is **not specified** in the
repository. Only the parts marked "Yes" have an STL; the others exist only
inside the STEP assembly and must be exported from it.

| Part | Qty | STL in repo |
|---|---:|---|
| Forearm shell (`Foreamr_V1.2`, four revolve-cut bodies) | 1 | Yes (`stl/Forearm_V1.2-Cut-Revolve1…4.STL`) |
| Wrist flange (`Flanec`) | 1 | Yes (`stl/Flanec.STL`) |
| Wrist flange 2 (`Flanec 2`) | 1 | Yes (`stl/Flanec_2.STL`) |
| Palm upper base (`Base-Palm_Upor_V1.2`) | 1 | Yes (`stl/Base-Palm_Upor_V1.2.STL`) |
| Palm base (`Base-Palm_V1.2`) | 1 | No |
| Hand palm base (`Hand_PalmBase_V1.2`) | 1 | No |
| Palm servo plates (`Hand_PalmServoPlate`, `…2`, `…3`) | 1 each | No |
| Hand plates (`Hand_Plate1`, `Hand_Plate2`) | 1 each | No |
| Finger MCP base plate, left and right | 5 each (4 fingers + thumb) | No |
| Finger proximal phalanx plate | 4 | No |
| Finger middle phalanx plate | 4 | No |
| Finger distal phalanx plate | 4 | No |
| Finger proximal link | 4 | No |
| Finger tendon link | 4 | No |
| Finger plate (`Part2^Finger_Plate_V1.2`) | 4 | No |
| Thumb CMC base plate | 1 | No |
| Thumb proximal phalanx plate | 1 | No |
| Thumb distal phalanx plate | 1 | No |
| Thumb proximal link | 1 | No |
| Thumb tendon link | 1 | No |
| Generic spacer (`Spacer`) | 10 | No (dimensions not recorded in the CAD names) |
| Bushing (`Bushing 1`) | 2 | No |
| Passive cover shell (third party, Ivo Maric) | optional | `third_party/` (STEP); see `NOTICE.md` |

## 5. Fasteners

| Item | Qty |
|---|---:|
| M3 brass heat-set insert, 5 mm | 64 |
| M3 brass heat-set insert, 3 mm | 5 |
| M3 brass heat-set insert, 4 mm | 4 |
| DIN 912 M3×6 socket head screw | 57 |
| DIN 912 M3×12 socket head screw | 10 |
| ISO 7045 M3×10 pan head cross-recess screw | 14 |
| ISO 7045 M3×20 pan head cross-recess screw | 5 |
| ISO 7045 M3×6 pan head cross-recess screw | 1 |
| DIN 7049 ST2.2×6.5 tapping screw | 34 |
| DIN 7049 ST2.2×9.5 tapping screw | 30 |
| DIN 985 M3 nyloc nut | 5 |
| DIN 125 M3 washer | 2 |
| M3 hex spacer, female–female, 12 mm | 3 |
| M3 hex spacer, female–female, 29.5 mm | 3 |

## CAD vs firmware discrepancies

Nothing has been built, so none of these has been resolved by experiment.
Decide each one before fitting parts.

| Topic | CAD | Firmware / docs | Consequence |
|---|---|---|---|
| Finger servos | SG90 | Written for MG996R-class servos | Torque is much lower; stall current is about 0.65 A, not 2.5 A, so the 2.2 A `current.trip` default and the supply sizing do not apply as written. |
| Servo count | 6 (5 SG90 + 1 MG996R) | 7 PCA9685 channels (thumb flexion, thumb opposition, 4 fingers, wrist) | The CAD has no dedicated thumb-opposition servo, while the firmware drives one (`thumb_rot`, channel 1). The firmware tolerates its absence (`digitsCalibrated()` does not require it), but it is enabled by default and the pinch, tripod and key poses assume it moves; without it the thumb will not oppose. |
| Thumb servo | 1 SG90 at the thumb | Flexion and opposition on separate channels | Whether the CAD servo drives flexion or opposition is not recorded. |
| Wrist | MG996R positioned as the wrist rotator | Wrist is optional, SG90 or MG996R | Consistent, but the firmware does not require it. |
| FSR400 | 4 | 3 read (thumb, index, middle) | One sensor has no input. |
| LM393 modules | 5 | 3 contact inputs | Two modules have no input. |
| Transmission | Rigid ball-jointed tie rods, no springs | Docs say "tendon" and describe stopping "before the tendon goes tight" | The calibration advice about slack tendons does not literally apply to a rigid linkage; the endpoint guard still does. |
| Force targets | SG90 torque class | `verification/README.md` gates 45 N power grasp | Probably not reachable with SG90 finger servos; unverified either way. |
