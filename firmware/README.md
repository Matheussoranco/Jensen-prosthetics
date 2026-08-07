# Jensen Hand — Firmware

ESP32 control stack for the Jensen Hand: myoelectric proportional control,
seven grasp patterns, FSR-based grip force regulation, and a guided
calibration procedure.

> **Not a medical device.** This firmware has not undergone clinical testing
> or regulatory review. See [Safety](#safety) before putting it on a person.

---

## 1. What it does

A single scalar **aperture** command (0 = open, 1 = closed) drives the whole
hand. Two surface-EMG electrodes produce that command: the flexor closes, the
extensor opens, and their activation is integrated into aperture over time, so
holding a contraction keeps closing and relaxing holds position. Which digits
move for a given aperture, and how far, is what distinguishes one grasp
pattern from another.

Fingertip force sensors close a second loop around that: once the grip reaches
its force target the hand stops closing rather than stalling its servos
against the object, and a sudden drop in fingertip force — an object starting
to slip — makes it tighten a step.

```
 EMG counts ─▶ despike ─▶ envelope ─▶ calibrated normalise ─▶ activation 0..1
                                                                    │
                                                     ┌──────────────┘
                                                     ▼
                                       velocity law (+ co-contraction gesture)
                                                     │
                                                     ▼
                                        ∫ dt ──▶ aperture 0..1 ◀── force limit
                                                     │                  ▲
                                                     ▼                  │
                                         grasp pattern → per-digit closure
                                                     │                  │
                                                     ▼                  │
                                    slew limit → endpoint map → pulse µs │
                                                                        │
 FSR counts ─▶ newtons ─▶ contact / slip ─▶ grip-force controller ──────┘
```

## 2. Layout

```
firmware/
├── platformio.ini            build configuration (esp32dev + native test env)
├── src/                      ESP32 application — hardware only
│   ├── main.cpp              setup/loop; samples inputs, applies outputs
│   ├── board_config.h        pin map, I2C addresses, loop rates
│   ├── hal/                  peripheral drivers
│   │   ├── servo_bus.*       PCA9685, rail + /OE safe torque off
│   │   ├── analog_in.*       ADC oversampling, battery divider
│   │   ├── button.*          click / double / long / very-long gestures
│   │   ├── feedback.*        status LED and haptic patterns
│   │   └── config_store.*    dual-slot CRC-protected NVS storage
│   └── ui/
│       ├── nextion.*         2.4" HMI protocol
│       └── serial_cli.*      console: calibration, tuning, telemetry
├── lib/jensen_core/          the control policy — no Arduino, host-testable
│   └── jensen/
│       ├── util.h            clamping and mapping helpers
│       ├── filter.h          EMA, median, envelope, rate limit, Schmitt
│       ├── emg.*             electrode conditioning + the velocity law
│       ├── digit.*           per-digit slew limiting and endpoint mapping
│       ├── grip.*            grasp pattern library and selector
│       ├── force.*           FSR conditioning and the grip-force loop
│       ├── modes.*           operating-mode state machine
│       ├── button_gestures.* click / double / long / very-long recognition
│       ├── safety.*          battery, load guard, fault aggregation
│       ├── calibration.*     guided electrode + bench servo calibration
│       ├── hand_config.*     the persisted configuration struct
│       ├── hand_state.h      inputs / outputs / telemetry
│       └── hand_controller.* the policy that ties it together
├── test/
│   ├── native/               89 host tests for the control core
│   └── hostcheck/stubs/      declaration-only Arduino API, for syntax checks
├── tools/
│   ├── run_core_tests.py     build + run the core tests without PlatformIO
│   ├── check_app_syntax.py   type-check src/ without the Xtensa toolchain
│   └── jensen_monitor.py     live telemetry viewer / CSV logger
└── docs/
    └── nextion_hmi.md        component IDs the display driver expects
```

The split between `src/` and `lib/jensen_core/` is the important one. Nothing
in `jensen_core` touches a peripheral, so the entire behaviour of the hand —
the drive law, the force loop, mode transitions, calibration sequencing — runs
on a development machine against synthetic inputs. `src/` only samples ADCs
and writes servo pulses.

## 3. Building

```bash
cd firmware

pio run -e esp32dev              # build
pio run -e esp32dev -t upload    # flash
pio device monitor               # serial console (115200 8N1)
```

Dependencies resolve automatically (`Adafruit PWM Servo Driver`). Set
handedness at build time in `platformio.ini` via `JENSEN_DEFAULT_SIDE_RIGHT`;
this only seeds the factory defaults, and a stored calibration overrides it.

### Tests

```bash
pio test -e native                        # with PlatformIO
python tools/run_core_tests.py            # without it
python tools/check_app_syntax.py          # type-check src/ with no toolchain
```

`run_core_tests.py` finds g++, clang++ or the MSVC Build Tools on its own.
`check_app_syntax.py` compiles the Arduino sources against declaration-only
stubs — it catches signature and include errors in seconds, but only
`pio run -e esp32dev` produces a flashable image or verifies the real library
APIs.

## 4. Wiring

All analogue inputs are on **ADC1**. ADC2 shares hardware with the Wi-Fi radio
and stops returning readings once the radio is enabled, which would silently
kill myoelectric control the first time someone adds a BLE feature.

| GPIO | Direction | Function |
|-----:|:----------|:---------|
| 21 | I2C SDA | PCA9685 @ 0x40 |
| 22 | I2C SCL | PCA9685 |
| 27 | out | PCA9685 `/OE` — low enables outputs |
| 26 | out | Servo rail enable (buck / high-side MOSFET) |
| 16 / 17 | UART2 | Nextion 2.4" HMI (RX / TX), 9600 baud |
| 18 / 5 | UART1 | Voice Recognition V3 (RX / TX), reserved |
| 4 | in, pull-up | Mode button, active low |
| 13 | out | Status LED (green 5 mm) |
| 14 | out | Vibration motor (LEDC PWM) |
| 2 | out | On-board LED — control-loop heartbeat |
| 25 / 19 / 23 | in, pull-up | LM393 contact comparators (thumb / index / middle) |
| 36 | ADC1_CH0 | EMG flexor |
| 39 | ADC1_CH3 | EMG extensor |
| 35 / 32 / 33 | ADC1 | FSR400 (thumb / index / middle) |
| 34 | ADC1_CH6 | Battery sense |

GPIO 0, 12 and 15 are strapping pins and are left unconnected. GPIO 2 is a
strapping pin but is only driven after boot.

**PCA9685 channels:** 0 thumb flex · 1 thumb opposition · 2 index · 3 middle ·
4 ring · 5 pinky · 6 wrist (optional).

**Battery divider.** The default assumes R1 = 100 kΩ (pack to node) and
R2 = 22 kΩ (node to ground), putting 8.4 V at ~1.51 V. Measure the resistors
you actually fitted and correct `kBatteryDividerRatio` in `board_config.h` —
a wrong ratio is the difference between "charged" and "park the hand".

**Servo power.** MG996R stall current is ~2.5 A each. Do not power the servo
rail from the ESP32 board's regulator; it comes from the buck converter, and
ground must be common. Bulk capacitance across the servo rail (≥ 1000 µF)
keeps rail sag from resetting the ESP32 when several digits move at once.

## 5. Bringing up a new hand

A freshly built hand has **placeholder servo endpoints**. They are set to
1200–1800 µs, which is roughly the middle third of an MG996R's travel: enough
to verify wiring and direction, far enough from the mechanical stops that a
mis-wired channel pulls a tendon instead of breaking a printed link. They are
not usable endpoints. Establishing the real ones is the first job.

### 5.1 Servo endpoints — once per digit, per hand

With the tendons fitted and the hand supported so nothing is loaded:

```
jensen> cal digit index
calibrating index — servo channel 2
  jog <us>     move by a signed number of microseconds (e.g. jog -25)
  pulse <us>   go to an absolute pulse width
  open         mark this position as fully open
  closed       mark this position as fully closed
  save         store both endpoints and leave
  cancel       leave without changing anything

cal> jog -50
1150 us
cal> jog -50
1100 us
...
cal> open
open = 980 us
cal> jog 400
1380 us
...
cal> closed
closed = 2050 us
cal> save
index: open 980 us, closed 2050 us, guards 940–2090 us
applied — use 'save' to persist
```

Jog in small steps and **stop before the tendon goes tight**. The guard band
is placed 40 µs outside whatever you mark, and the control loop can never
command a pulse outside it afterwards.

Repeat for `thumb`, `thumb_rot`, `index`, `middle`, `ring`, `pinky`, and
`wrist` if one is fitted. Then `save` at the top level to persist.

Direction does not need a flag: if a servo runs backwards, mark the physically
open position as `open` and the physically closed one as `closed`, in whatever
order the pulse widths come out. Everything downstream reads the ordering.

### 5.2 Fingertip sensors

With the hand touching nothing:

```
jensen> cal fsr
fingertip sensors tared (hand must be touching nothing)
jensen> save
```

FSR zero drifts with temperature and with how much the printed fingertip pad
has taken a set, so this is worth re-running occasionally.

To get meaningful newtons rather than a repeatable arbitrary number, press a
fingertip against a kitchen scale until it reads a known load, note the raw
count from `telem`, and set `spanCounts` / `referenceNewtons` accordingly. The
grip-force loop regulates fine without this — it just means the numbers on the
display are approximate.

### 5.3 Electrodes

`cal emg`, or hold the button for 1.2 s. The sequence is guided by haptic cues
and, if a display is fitted, by on-screen prompts:

| Step | Duration | Prompt |
|:-----|:---------|:-------|
| settle | 1 s | relax your arm |
| capture | 3 s | hold still — measuring baseline and noise |
| settle | 1 s | ready to close |
| capture | 3 s | close hard — measuring flexor maximum |
| settle | 1 s | ready to open |
| capture | 3 s | open hard — measuring extensor maximum |

The result is **rejected** if a contraction is not clearly distinguishable
from rest (it must clear the noise floor by at least 120 counts and a factor
of 2.5). That is the usual symptom of an electrode that has come loose, and
accepting it would amplify noise straight into servo motion. A successful
calibration is saved to NVS immediately.

## 6. Using the hand

### Button — the only control the wearer always has

| Gesture | Action |
|:--------|:-------|
| click | next grasp pattern |
| double click | hold / release (freeze the hand where it is) |
| long press, 1.2 s | start electrode calibration |
| very long press, 4 s | park the hand (limp) / wake it up |

Holding past 1.2 s starts calibration and buzzes; carrying on to 4 s escalates
to parking, which aborts that calibration. The escalation is deliberate — a
long hold on the only reachable button has to end with the hand going limp.

### Co-contraction

Contracting both electrodes hard together for 0.2 s changes grasp pattern
without reaching for the button. Motion is frozen while the gesture is held,
so the hand does not lurch toward whichever channel is stronger, and the
gesture will not repeat until both channels relax again.

### Grasp patterns

| Name | Description |
|:-----|:------------|
| `rest` | Relaxed neutral posture; ignores aperture. The fallback on a fault. |
| `power` | Cylindrical whole-hand wrap. The default. |
| `pinch` | Thumb-to-index tip pinch; ulnar digits held clear. |
| `tripod` | Thumb, index and middle. |
| `key` | Thumb adducted against the side of a closed index. |
| `point` | Index pinned at full extension; the rest close. |
| `hook` | Four-finger carrying hook, thumb parked open. |

`grips` lists them with their force scaling; `grip enable <name> on|off` takes
unwanted ones out of the cycle. Most users settle on two or three, and cycling
through all seven on every co-contraction makes the hand tedious to operate.

### Status LED

| Pattern | Meaning |
|:--------|:--------|
| solid | active |
| slow breathe | idle — relaxed, outputs released |
| slow blink | holding |
| fast blink | calibrating (or bench mode) |
| double blink | warning-level fault; the hand still works |
| SOS | severe fault or parked; servos are unpowered |

## 7. Serial console

115200 8N1. `help` lists everything; the essentials:

```
status                     one-shot state summary
telem [on|off] [hz]        stream telemetry (tools/jensen_monitor.py parses it)
config                     print the active configuration and every tuning key
grips                      list grasp patterns

grip [next|prev|<name>]    select a grasp pattern
grip enable <name> on|off  add or remove a pattern from the cycle
hold [on|off]              freeze the hand
park [on|off]              safe torque off / wake up
open | close               direct aperture command
aperture <0..1>            direct aperture command
wrist <0..1>               wrist rotator, 0.5 is neutral

cal emg | cal fsr | cal digit <name> | cal abort

set <key> <value>          change a tuning value
side left|right            reload factory defaults for a handedness
save | load | defaults     persist / restore / reset
erase                      wipe stored configuration
reboot
```

Direct aperture commands are revoked the moment real muscle activity appears —
the wearer always outranks the console.

### Tuning keys

| Key | Default | Effect |
|:----|--------:|:-------|
| `emg.attack` | 0.030 s | envelope rise; lower feels quicker, noisier |
| `emg.release` | 0.150 s | envelope fall; lower feels twitchier |
| `emg.deadband` | 0.08 | activation ignored as noise |
| `emg.gamma` | 1.35 | >1 gives finer control at low effort |
| `drive.mode` | `dual` | `dual` (two electrodes) or `single` |
| `drive.close_speed` | 2.2 /s | full travel per second at full activation |
| `drive.open_speed` | 2.8 /s | |
| `drive.single_open` | 0.12 | single-channel: below this the hand opens |
| `drive.single_hold` | 0.32 | single-channel: below this it holds |
| `drive.co_level` | 0.55 | co-contraction threshold |
| `force.target` | 12 N | grip force setpoint |
| `force.gain` | 0.05 | integral gain of the force loop |
| `force.slip_step` | 0.05 | closure added on a detected slip |
| `speed.<digit>` | 2.2 /s | per-digit slew limit |
| `scale.<grip>` | varies | per-grip force scaling (pinch is 0.45) |
| `battery.warn` | 3.45 V/cell | |
| `battery.critical` | 3.20 V/cell | park threshold |
| `idle.timeout` | 20 s | inactivity before relaxing |
| `haptics` | on | |

Changes take effect immediately and are lost on reboot unless you `save`.

### Live telemetry

```bash
python tools/jensen_monitor.py COM7            # live dashboard
python tools/jensen_monitor.py COM7 --csv run.csv
```

The firmware emits one `T,key=value,...` line per sample; the monitor renders
it and can log it for later analysis.

## 8. Single-electrode operation

With one electrode, `set drive.mode single`. The law then mirrors HACKberry's:
relaxed opens, light effort holds, firm effort closes proportionally.

One difference is deliberate. HACKberry maps speed with
`map(v, 40, 100, 0, 6)` and evaluates it for inputs below 40 as well, so the
region between its "zero" threshold (30) and 40 produces a small *negative*
speed — the hand creeps open inside what reads like a deadband. Here the
deadband is exactly a deadband, and there is a regression test
(`drive_single_channel_deadband_has_no_creep`) that says so.

## 9. Safety

The design assumptions worth knowing before relying on this:

- **Two independent ways to remove torque.** `/OE` cuts the PWM outputs in
  microseconds; the rail enable drops servo power entirely. Safe torque off
  asserts both, because they fail differently — the second also covers a
  PCA9685 that has stopped responding on I2C while still emitting its last
  frame.
- **Severe faults park the hand**: critical battery, servo bus lost, no valid
  servo calibration. Warning faults (battery low, uncalibrated electrodes,
  sustained load) leave it working and tell the user.
- **A user-parked hand stays parked.** A fault appearing and clearing does not
  silently re-energise servos the wearer switched off.
- **The hand cannot be re-energised into a severe fault.**
- **No `delay()` anywhere.** A hand that stops reading its electrodes to
  redraw a display stutters under the user's control.
- **Watchdog.** A wedged control loop resets the ESP32, which brings the servo
  bus back up in safe torque off.
- **Sustained load backs off rather than releasing.** After 25 s at high
  fingertip force the grip target is reduced to protect the servos — a
  prosthesis that drops what it is carrying to protect itself is worse than
  one that grips a little softer.
- **An uncalibrated electrode drives nothing.** Without a trustworthy
  calibration the channel reports zero activation rather than synthesising
  intent from an arbitrary scaling.
- **Configuration is stored in two slots with a CRC.** An interrupted save
  cannot destroy a working calibration, and a corrupt one falls back to
  defaults loudly rather than running on half of two calibrations.

What is *not* handled: there is no motor current sensing, so a true stall is
inferred from fingertip load and time rather than measured. There is no
redundancy in the electrode path. This has not been tested against IEC 60601
or any other medical standard.

## 10. Relationship to HACKberry

The velocity-integration idea — condition a myoelectric signal, turn it into a
speed, integrate that into a hand position — comes from exiii's HACKberry, and
this firmware was written after reading their published Arduino sketches.
**No HACKberry code is used here.** Their sketch is licensed GPLv3; this is an
independent implementation for entirely different hardware (ESP32 + PCA9685
against ATmega32u4 + direct servo pins, seven digits against three, closed-loop
force against open loop). The comparisons in the comments and in
[§8](#8-single-electrode-operation) reference their published behaviour for
engineering context. See `../NOTICE.md`.

Three things are done differently on purpose:

1. **Calibration stores baseline, noise floor and maximum separately**, and
   refuses a result where the contraction is not distinguishable from rest.
   HACKberry takes the min and max it happens to see in four seconds, which
   produces a working-looking calibration from an electrode that has fallen
   off.
2. **The speed mapping is clamped** — see [§8](#8-single-electrode-operation).
3. **Timing is in seconds, not loop iterations.** Changing the control rate
   does not change the feel of the hand.

Handedness is also handled differently: HACKberry threads left/right and
open/close endpoint swapping through the whole control path. Here direction
lives entirely in the servo endpoint ordering, so nothing downstream of the
configuration struct knows or cares which hand it is driving.

## 11. License

Same as the repository: **CC BY-NC-SA 4.0**. See `../LICENSE` and
`../NOTICE.md`.
