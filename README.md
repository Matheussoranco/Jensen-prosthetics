# Jensen Hand

**An open-source, servo-actuated, tendon-driven anthropomorphic prosthetic hand**

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](LICENSE)

---

## Abstract

The Jensen Hand is an open-hardware transradial/partial-hand prosthetic
platform built around a servo-actuated, tendon-driven finger architecture.
Five independently linked digits (four fingers plus an opposable thumb with
a dedicated CMC base) are actuated through artificial tendons routed from
wrist/palm-mounted servos, reproducing the general force-transmission
strategy of the human extrinsic musculature rather than placing an actuator
at every joint. The platform combines MG996R/SG90 servos driven by a
PCA9685 PWM expander, an ESP32 microcontroller, FSR-based grip force
sensing, and a Nextion touch display for status/configuration, powered from
a 2S Li-ion pack through a buck-regulated supply. This repository releases
the complete mechanical design (STEP/STL) of the hand and forearm shell
under an open, non-commercial license, the reference component models used
to validate fit and clearance during development, and the ESP32 control
firmware: myoelectric proportional control over a seven-pattern grasp
library, closed-loop grip-force regulation with slip response, and a guided
calibration procedure.

## 1. Introduction and Motivation

Commercial myoelectric and body-powered prosthetic hands remain
expensive and largely closed-source, which limits their availability,
repairability, and the ability of clinicians, researchers, and amputees
themselves to iterate on the design. Since the mid-2010s, a body of
open-source prosthetics work — e-NABLE's community-maintained device
library [4], Zuniga et al.'s Cyborg Beast [2], exiii's HACKberry [3], and
Open Bionics' Ada Hand / Hero Arm lineage [5] — has demonstrated that
3D-printable, off-the-shelf-actuator hands can deliver useful grasping
function at a small fraction of commercial cost. The Jensen Hand continues
in that tradition: it is a from-scratch mechanical and electromechanical
design, informed by the design conventions established in that body of
work and in the broader anthropomorphic-hand literature [1], and released
so that it can be reproduced, studied, and modified by others.

This repository contains the CAD and exported interchange geometry for the
hand and forearm assembly, the bill of materials, and the ESP32 control
firmware under [`firmware/`](firmware/) — see
[Section 6.1](#61-firmware) and [`firmware/README.md`](firmware/README.md).

## 2. Related Work

| Project | Approach | Relevance to this design |
|---|---|---|
| Belter, Segil, Dollar & Weir, *Mechanical design and performance specifications of anthropomorphic prosthetic hands: a review*, J. Rehabil. Res. Dev. 50(5), 2013 [1] | Comparative survey of commercial myoelectric hands (Vincent, i-Limb, bebionic, Michelangelo) | Baseline design trade-offs (actuator count vs. weight vs. grasp force) referenced when sizing the servo/tendon layout |
| Zuniga et al., *Cyborg Beast: a low-cost 3D-printed prosthetic hand for children with upper-limb differences*, BMC Res. Notes 8:10, 2015 [2] | Passive, body-powered, cable-driven digits with elastic return | Informed the tendon-return approach used in the finger links |
| exiii Inc. / Mission ARM Japan, **HACKberry**, 2015 [3] | Open-source, sub-$300 myoelectric hand with full CAD + firmware release | Reference point for scoping a fully open BOM around commodity electronics |
| e-NABLE (Enabling the Future) [4] | Distributed volunteer network, open device library | General open-hardware distribution and licensing precedent |
| Open Bionics, **Ada Hand** (derived from The Open Hand Project's Dextrus) and **Hero Arm** [5] | 3D-printed, servo/tendon actuated, CC-licensed | Closest architectural analogue: servo-in-palm/forearm, tendon-routed fingers |
| Custom FSR-based grip sensing, IEEE BioRob 2018 [6] | Fingertip force-sensitive resistors for grasp/slip feedback | Basis for the FSR400 grip-sensing approach adopted here |

The passive cover shell in `third_party/` is a direct reuse of a published
design by Ivo Maric (2017) rather than related work in the academic sense —
see [Attribution](#8-license-and-attribution) below.

## 3. System Overview

```
                 ┌────────────────────┐
   2S Li-ion  →  │  Buck Converter     │ →  5V rail  →  ESP32 / PCA9685 / logic
   7.4V pack     │  (5–40V in)         │
                 └────────────────────┘
                            │
                            ▼
   ESP32  ──I2C──▶ PCA9685 (16-ch PWM) ──▶ MG996R × N (finger/thumb tendons)
     │                                └──▶ SG90 × N (auxiliary joints)
     │
     ├──ADC──▶ FSR400 × N (fingertip grip force) ── via LM393 conditioning
     ├──UART──▶ Nextion 2.4" HMI (status / configuration display)
     ├──GPIO──▶ Vibration motor (haptic feedback)
     └──GPIO──▶ Push button (mode select), status LED
```

Mechanically, the forearm shell houses the battery, buck converter, ESP32 +
expansion board, and PCA9685; the palm carries the servo bank and routes
tendon lines through the MCP/CMC base plates into each digit's phalanx
links.

## 4. Mechanical Design

The hand is modeled as a fully parametric SolidWorks assembly (native
source retained locally; see [Section 7](#7-repository-structure) for why
it is not included in this repository) and exported to open STEP and STL
formats for public release.

**Digit architecture.** Each finger is built from a proximal, middle, and
distal phalanx plate connected through a tendon link and a proximal link,
mounted to a shared MCP base plate (left/right variants for the four
fingers). The thumb uses an analogous but distinct proximal/distal
phalanx-plate pair mounted to a dedicated CMC base plate, giving it an
independent axis of opposition rather than a mirrored finger geometry.
Tendon routing pulls each distal segment through its link chain, with
segment geometry left compliant enough to under-actuate to some extent
per digit — consistent with the underactuated, tendon-driven pattern
documented for low-cost prosthetic hands [1].

**Palm and forearm.** The palm is split across a base plate and two/three
servo mounting plates (`Hand_PalmBase`, `Hand_PalmServoPlate`,
`Hand_PalmServoPlate2/3`) that carry the MG996R bank and route tendons
through to the digits. The forearm shell (`Foreamr_V1.2`, exported as four
STL bodies from its cut/revolve features) houses the electronics and
battery and mounts to the palm assembly.

**Fasteners.** The design uses M3 hardware throughout: DIN 912 and DIN 7991
socket-head screws in multiple lengths, DIN 125 washers, DIN 985 nyloc
nuts, M3 brass heat-set inserts (3–8 mm), and M3 hex spacers (12 mm and
29.5 mm) for standoffs between plates.

## 5. Actuation

| Actuator | Role | Notes |
|---|---|---|
| MG996R (metal-gear, ~10 kg·cm) | Primary finger/thumb tendon pull | Mounted in the palm/servo-plate bank |
| SG90 (micro servo) | Auxiliary/secondary joint actuation | Lower load-bearing joints |
| PCA9685 | 16-channel, 12-bit I2C PWM driver | Offloads servo timing from the ESP32; single I2C bus |

Driving servos through a PCA9685 rather than directly from ESP32 GPIO
timers frees the microcontroller from software PWM and allows all channels
to be updated over I2C with consistent timing — the same rationale used in
comparable open hand-control stacks.

## 6. Electronics, Control, and Sensing

- **MCU:** ESP32 (Wi-Fi/BLE-capable), on an expansion/breakout board for
  I2C, ADC, and UART fan-out.
- **Power:** 2S 7.4 V Li-ion pack → step-down buck converter (5–40 V input
  range) → regulated logic rail; USB-C jack for charging/programming; IP67
  DC barrel jack as an alternative power input.
- **Grip sensing:** FSR400 force-sensitive resistors at the fingertips,
  conditioned through an LM393 comparator stage for threshold/contact
  detection, in line with FSR-based grasp/slip-sensing approaches reported
  in the prosthetics literature [6].
- **Feedback / HMI:** Nextion 2.4" touch display for status and
  configuration; a vibration motor for haptic feedback; a status LED and a
  momentary push button for local mode control.

### 6.1 Firmware

The control stack lives in [`firmware/`](firmware/) and is documented in
detail in [`firmware/README.md`](firmware/README.md). In outline:

A single scalar **aperture** command (0 = open, 1 = closed) drives the hand.
Two surface-EMG electrodes produce it — the flexor closes, the extensor
opens — with activation integrated into aperture over time, so holding a
contraction keeps closing and relaxing holds position. Which digits move for
a given aperture, and how far, is what distinguishes one grasp pattern from
another: in a power grasp all four fingers sweep the full range; in a key
grip they stay curled and only the thumb moves; when pointing, the index
never leaves full extension. Seven patterns ship, and contracting both
electrodes together ("co-contraction") cycles them without reaching for the
button.

Fingertip FSRs close a second loop around that. Once the grip reaches its
force target the hand stops closing rather than stalling its servos against
an object it cannot crush, and a sudden drop in fingertip force — an object
starting to slip — makes it tighten a step. The force setpoint is scaled per
pattern, because the setpoint that is comfortable wrapped around a bottle
will crush an egg held between two fingertips.

Architecturally the firmware is split so that the entire control policy —
drive law, force loop, mode transitions, calibration sequencing — lives in
`firmware/lib/jensen_core/`, which contains no Arduino code and compiles on
any host. That is what allows the behaviour of the prosthesis to be
exercised against synthetic inputs in an 89-case test suite without a hand
attached; `firmware/src/` only samples ADCs and writes servo pulses. The
velocity-integration approach is the one idea taken from exiii's HACKberry
[3]; no HACKberry code is used (see [`NOTICE.md`](NOTICE.md)).

## 7. Repository Structure

```
Jensen-prosthetics/
├── README.md
├── LICENSE                 CC BY-NC-SA 4.0
├── NOTICE.md                third-party attributions (read before reuse)
├── firmware/                ESP32 control stack — see firmware/README.md
│   ├── platformio.ini
│   ├── src/                 ESP32 application (peripherals only)
│   ├── lib/jensen_core/     control policy — no Arduino, host-testable
│   ├── test/native/         89 host tests for the control core
│   ├── tools/               test runner, syntax check, telemetry monitor
│   └── docs/                Nextion HMI component specification
├── hardware/
│   ├── assembly/
│   │   └── Jensen_Hand_Assembly.step   full hand assembly, open STEP
│   ├── components/          off-the-shelf reference models (metadata-cleaned)
│   │   ├── ESP32_DevBoard.stp
│   │   ├── ESP32_Expansion_Board.step
│   │   ├── PCA9685_PWM_Driver.stp
│   │   ├── Buck_Converter_5-40V_9A.stp
│   │   ├── LED_Green_5mm.stp
│   │   ├── Nextion_2.4in_Display.step
│   │   ├── Tie_Rod.step
│   │   └── MG996R_Servo/    servo body + horn/arm/bushing/shaft references
│   └── stl/                 printable geometry (forearm shell, palm parts)
├── media/renders/           reference renders and component photos
└── third_party/
    └── passive_prosthetic_hand_cover_by_ivo_maric.step   see NOTICE.md
```

Native SolidWorks source files (`.SLDPRT` / `.SLDASM`) are **not** included
in this repository. Recent SolidWorks parts are stored in a compressed
container that does not expose a standard, tool-readable metadata stream —
attempts to verify and strip embedded authoring metadata (username,
workstation, edit history) with open tooling were inconclusive, so rather
than publish files that might still carry that information, only the open,
fully-inspectable STEP and STL exports are published here. STEP alone is
sufficient to reproduce, re-derive, or re-parameterize the full assembly in
any CAD package. Native files may be provided on request once independently
scrubbed.

## 8. License and Attribution

This repository is licensed under **Creative Commons
Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**
— see [`LICENSE`](LICENSE). In short: you may use, share, and adapt this
work for non-commercial purposes, with attribution, and any derivative must
be shared under the same license.

Full third-party attributions are in [`NOTICE.md`](NOTICE.md). In summary:

- `third_party/passive_prosthetic_hand_cover_by_ivo_maric.step` is
  **Ivo Maric's** "Passive Prosthetic Hand" design (2017), used with credit
  under its original Credit–Remix–Noncommercial terms:
  <https://www.myminifactory.com/object/3d-print-passive-prosthetic-hand-41934>
- Reference component models under `hardware/components/` represent
  commercial off-the-shelf parts, sourced from public CAD libraries for
  fit-checking; they are not original design work of this project.
- Design approach and conventions draw on Open Bionics, exiii's HACKberry,
  e-NABLE, and the academic literature cited below.

## 9. Future Work

- **Bench validation of the firmware.** The control core is covered by an
  89-case host test suite, but the ESP32 build has not yet been run against
  physical hardware; servo endpoints, the FSR force fit, and the battery
  divider ratio all need measuring on a real build.
- Motor current sensing, so a stalled servo can be detected directly rather
  than inferred from fingertip load and elapsed time.
- Pattern-recognition control across more than two electrode sites, to
  replace co-contraction as the grip-change gesture.
- Independent verification and release of native SolidWorks source once
  personal metadata can be conclusively stripped or the files are
  re-authored in a clean template.
- Quantified grasp-force and cycle-life testing against the benchmarks
  discussed in [1] and [6].

## 10. Disclaimer

This is a research/hobbyist hardware project, **not a certified medical
device**. It has not undergone clinical testing or regulatory review. Do
not use as a substitute for a properly fitted, clinically supervised
prosthesis without appropriate professional oversight.

## References

1. Belter, J.T., Segil, J.L., Dollar, A.M., Weir, R.F. "Mechanical design
   and performance specifications of anthropomorphic prosthetic hands: a
   review." *J. Rehabil. Res. Dev.* 50(5), 2013, 599–618.
   <https://www.rehab.research.va.gov/jour/2013/505/belter505.html>
2. Zuniga, J., Katsavelis, D., Peck, J., et al. "Cyborg beast: a low-cost
   3D-printed prosthetic hand for children with upper-limb differences."
   *BMC Res. Notes* 8, 10 (2015).
   <https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4304188/>
3. exiii Inc. / Mission ARM Japan. **HACKberry** — open-source 3D-printed
   myoelectric prosthetic arm, 2015. <https://www.exiii-hackberry.com/>
4. e-NABLE (Enabling the Future) — open-source community device library.
   <https://en.wikipedia.org/wiki/E-NABLE>
5. Open Bionics. **Ada Hand** (derived from The Open Hand Project's
   Dextrus) and **Hero Arm**. <https://github.com/Open-Bionics>
6. "Custom Force Sensor and Sensory Feedback System to Enable Grip Control
   of a Robotic Prosthetic Hand." *2018 7th IEEE Int. Conf. on Biomedical
   Robotics and Biomechatronics (BioRob)*.
   <https://ieeexplore.ieee.org/document/8487948/>
7. Maric, I. "Passive Prosthetic Hand," 2017 (cover shell design, used
   under credit — see [NOTICE.md](NOTICE.md)).
   <https://www.myminifactory.com/object/3d-print-passive-prosthetic-hand-41934>
