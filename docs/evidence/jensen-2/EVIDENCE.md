# Evidence Pack — Jensen-prosthetics #2 (compile the ESP32 firmware target in CI)

| Field | Value |
|---|---|
| Ticket | [#2](https://github.com/Matheussoranco/Jensen-prosthetics/issues/2) (parent #1) |
| Trace id | `JENSEN-2` (delegated by the Engineering Lead, mesh event 18) |
| Author | Claude Code, Opus 5.5 (session `b4efa619`) |
| Base | `origin/main` @ `a03222bf042dfac35949b10a0f5ec72413d60acd` |
| Build revision | `66bb1862e4edd6a6644650a39401f3c6cb115fd5` (workflow and pins). Later commits on the branch change docs only. |
| CI run | [44846922294](https://github.com/Matheussoranco/Jensen-prosthetics/actions/runs/44846922294), attempts 1 and 2 |
| Reviewers | **Pending**: two firmware reviewers, as the ticket requires |

## What this proves, and what it does not

| Claim | Evidence | Kind |
|---|---|---|
| The firmware **compiles and links for the ESP32** with the real Arduino-ESP32 framework and libraries | Target job `success` in both attempts; `[SUCCESS] Took 49.96 seconds` | **Target build (CI)** |
| **No new compiler warnings** under `-Wall -Wextra` | 0 `warning:` lines in the attempt-1 build log (`logs/warnings-count.txt`) | Target build (CI) |
| The image is **bit-for-bit reproducible** from the pinned inputs | All 4 image SHA-256s identical between attempt 1 (cold package install) and attempt 2 (restored cache, different runner) (`logs/digests-attempt1-vs-2.txt`) | Target build (CI) |
| Host checks are kept | `Host checks: success` in the same run (100-case core suite plus the stub syntax check) | Host (CI) |
| **No device connection or upload** | The only `pio` commands executed are `pio run -e esp32dev` (default build target), `pio --version` and `pio pkg list`. Every other "upload" in the workflow is a comment, a step name, or `actions/upload-artifact` (a GitHub CI artifact). GitHub-hosted runners have no serial device. | Static check of `.github/workflows/ci.yml` |
| Behaviour on hardware (servo motion, PCA9685 frames, watchdog recovery, ADC readings, timing) | **Not verified.** Nothing here ran on an ESP32. | **None. Requires the authorized bench campaign (#1, #4).** |

## Build record (attempt 1, `66bb186`)

| Item | Value |
|---|---|
| PlatformIO Core | 6.2.0 (pinned in the workflow) |
| Platform | `espressif32 @ 6.13.0` (pinned) |
| Framework | `framework-arduinoespressif32 @ 3.20017.241212+sha.dcc1105b`, i.e. **Arduino-ESP32 core 2.0.17** |
| Toolchain | `toolchain-xtensa-esp32 @ 8.4.0+2021r2-patch5` |
| Uploader tool (installed, **not invoked**) | `tool-esptoolpy @ 2.41100.260830 (4.11.0)` |
| Libraries | Adafruit PWM Servo Driver Library 3.0.3, Adafruit BusIO 1.17.4 (both pinned), Preferences 2.0.0, Wire 2.0.0 (bundled with the core), `jensen_core` |
| Board | Espressif ESP32 Dev Module, 240 MHz, 320 KB RAM, 4 MB flash |
| RAM | 8.2%: 26,804 of 327,680 bytes |
| Flash | 28.1%: 368,613 of 1,310,720 bytes |

| Image | SHA-256 | Bytes |
|---|---|---|
| `firmware.bin` | `eccfa70fb404fa373e82040dd5a6967b09d5774c9feb8c2bd6b1b2aa0e26f098` | 368,976 |
| `firmware.elf` | `35152ddd39b357e0bc8f9c85bc8c55d89d777b04cf2d64e8d97e0b2903262442` | 8,763,440 |
| `bootloader.bin` | `3d234a7471f67b013686dabd4dee7c1fa915c9928463616a94bc9297acf1abf8` | 17,536 |
| `partitions.bin` | `148b959cbff1c38aa8e1d5c0ba9d612c54997b945e56a63f41223eef650653a1` | 3,072 |

CI artifact `jensen-esp32dev-66bb1862…` (attempt 1) is 3,572,649 bytes, with GitHub
digest `sha256:29f8a0020b404be22217be25f2156517e7e219930eba923941b5164c1b078026`,
and expires on 2026-12-21. The images can be re-derived from the pins at any time,
because the build is reproducible.

## Decisions

1. **Pin `espressif32@6.13.0`, not the newest platform (8.0.0).** The firmware
   uses the Arduino-ESP32 core 2.x API: `ledcSetup`/`ledcAttachPin`
   (`src/hal/feedback.cpp:31-32,118-119`) and `esp_task_wdt_init(seconds, panic)`
   (`src/main.cpp:195`). The host stubs model the same API. The 6.x platform
   line ships core 2.0.x (confirmed by the resolved `3.20017…`, i.e. 2.0.17),
   while 7.x/8.x move to core 3.x, where those calls are gone.
   - The pin equals what the previous `^6.5.0` already resolved to, so this
     changes reproducibility, not behaviour.
   - *Alternative rejected:* port to core 3 now. It's a real code change to the
     LEDC and watchdog paths, with its own review and hardware implications, and
     out of scope for "compile the target".
2. **Pin the transitive `Adafruit BusIO`.** Otherwise it floats beneath the
   pinned PWM driver.
3. **Build in CI rather than locally.** This workstation has no Xtensa toolchain
   (nor any C++ compiler; MSVC is no longer installed), and the brief asks to
   avoid large local downloads. GitHub-hosted runners also give a clean,
   repeatable environment for the reproducibility check.
4. **Compile-only by construction.** No upload or monitor target is invoked, and
   no port is configured. The CI artifact is for inspection and audit, not a
   flashing path.

## Risks and unverified conditions

- **Hardware behaviour is entirely unverified.** It needs the bench campaign;
  see `verification/README.md` and issues #1/#4.
- **Pin drift is now explicit.** Raising the platform pin to 7.x/8.x will break
  the build until the LEDC and watchdog calls are ported to core 3. The comment
  in `platformio.ini` and the firmware README say so.
- **`pio test -e native` is not run in CI.** The host suite runs through
  `tools/run_core_tests.py` instead (same sources, no PlatformIO), as before
  this change.
- The CI artifact expires after 30 days. The digests above, together with the
  reproducible build, are the durable record.
