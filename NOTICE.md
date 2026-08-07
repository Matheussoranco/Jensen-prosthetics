# Third-Party Notices

This project is released under CC BY-NC-SA 4.0 (see `LICENSE`). The notices
below cover material in this repository that originates from other authors
and is included with attribution, as required by their own licenses or as a
courtesy.

## Passive prosthetic hand cover — Ivo Maric

File: [`third_party/passive_prosthetic_hand_cover_by_ivo_maric.step`](third_party/passive_prosthetic_hand_cover_by_ivo_maric.step)

Design by **Ivo Maric**, originally published 2017-11-07 as "Passive
Prosthetic Hand" on MyMiniFactory (also mirrored on GrabCAD):
<https://www.myminifactory.com/object/3d-print-passive-prosthetic-hand-41934>

Licensed by the original author under MyMiniFactory's **Credit – Remix –
Noncommercial** terms: attribution is required, remixing/modification is
permitted, and commercial use is not. This file is included unmodified aside
from removing the original author's local file path from the STEP header;
authorship metadata is otherwise preserved. Any redistribution of this file
or derivatives of it must retain credit to Ivo Maric and a link to the
source above, and must not be sold or used commercially.

## Off-the-shelf component reference models

The files under `hardware/components/` (ESP32, PCA9685 PWM driver, MG996R
servo, buck converter, LED, Nextion display, tie rod, etc.) are CAD
representations of commercially available, off-the-shelf electronic and
mechanical parts, sourced from public CAD libraries (GrabCAD and equivalent
community repositories) for mechanical fit-checking during design. They
represent standard components, not original design work by this project,
and are included purely as engineering references for anyone reproducing
the assembly. Personal authoring metadata (contributor names, timestamps,
local file paths) originally embedded in these files has been removed;
no attribution is legally required for their inclusion, but credit goes
to the respective GrabCAD community contributors who originally modeled
them.

## Firmware — relationship to exiii's HACKberry

The firmware under `firmware/` was written after reading the published
HACKberry Arduino sketches, and the control approach it uses — condition a
myoelectric signal, convert it to a speed, integrate that speed into a hand
position — comes from that work.

**No HACKberry source code is present in this repository.** exiii releases the
HACKberry Arduino sketch under the **GNU General Public License version 3**
(their 3D and circuit data is separately under CC BY-NC-SA 4.0). The firmware
here is an independent implementation written from scratch for different
hardware — ESP32 + PCA9685 rather than ATmega32u4 with direct servo pins,
seven actuated digits rather than three, closed-loop grip force rather than
open loop, and a different architecture throughout — so no GPL-licensed code
is copied, adapted, or linked, and no GPL obligation attaches to this
repository.

Several comments in `firmware/lib/jensen_core/` and a section of
`firmware/README.md` describe HACKberry's published control behaviour by way
of comparison, including one specific defect in its speed mapping. Those
references are engineering commentary on publicly documented behaviour, not
reproduced code.

- HACKberry project and Open Source Project Policy:
  <https://www.exiii-hackberry.com/> · <https://github.com/mission-arm/HACKberry>

## Firmware — third-party libraries

The firmware depends on one external Arduino library, resolved at build time
by PlatformIO and **not** vendored into this repository:

- **Adafruit PWM Servo Driver Library** — BSD licensed, © Adafruit Industries.
  <https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library>

The declaration-only headers under `firmware/test/hostcheck/stubs/` mirror the
*signatures* of the Arduino core, `Wire`, `Preferences`, `esp_task_wdt` and the
Adafruit driver so the application sources can be type-checked without the
Xtensa toolchain. They contain no implementation and are not derived from
those projects' source.

## Design inspiration

This project's mechanical and control approach was informed by, and owes a
conceptual debt to, several open-source prosthetics projects and academic
work. See the **Related Work** and **References** sections of `README.md`
for full citations, including Open Bionics (Ada Hand / Hero Arm), exiii
Inc.'s HACKberry, and the e-NABLE community.
