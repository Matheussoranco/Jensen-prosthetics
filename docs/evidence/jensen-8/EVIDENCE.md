# Evidence Pack — Jensen-prosthetics #8 (PCA9685 reset recovery)

- **Issue:** #8, PCA9685 is never re-initialised after a servo-bus outage
- **Pull request:** #10
- **Author:** Claude Code (Sonnet 5.5)
- **Branch:** `fix/8-servo-driver-reset`, base `main` @ `bf50935`
- **Code revision described here:** `c0bb469889f2431a461bdb239c85074947ca5104`
  (later commits on the branch add only this evidence directory)
- **Validation scope:** host tests and compile-only CI. **Nothing was run on
  hardware.**
- **Review status:** `PENDING_REVIEW`. Safety-critical firmware: two
  independent reviewers, one senior.

## Owner decision implemented

Asked what the hand should do after a servo-controller reset, the owner
answered: "Stay limp until werer acts, but continue observing and comunicating
with it".

## Scope to proof

| Requirement | Implementation | Proof (host) |
|---|---|---|
| Notice a reset chip that still acknowledges | `ServoBus::poll` reads `MODE1`, `PRE_SCALE` | HAL `a_power_cycled_chip_is_detected_cut_off_and_reconfigured` |
| Cut torque before writing to it | `poll` drops `/OE` and rail first | same test: no register write while torque available |
| Reconfigure so it actually wakes | `reset()` before `setPWMFreq()`, read-back | HAL `library_set_frequency_alone_leaves_a_fresh_chip_asleep`, `a_chip_that_will_not_take_its_setup_is_never_healthy` |
| Stay limp | `ModeMachine` latch | core `reset_is_not_cleared_by_time_muscles_or_any_other_command`, `reset_arriving_as_the_bus_recovers_does_not_resume_by_itself` |
| Until the wearer acts | very long press begun after the reset; display tap after 1 s | core `very_long_press_...`, `press_already_under_way_...`, `display_acknowledgement_has_its_own_short_guard` |
| No stale motion on resume | aperture and wrist commands dropped | core `very_long_press_...`, `wrist_command_sent_while_limp_is_dropped` |
| Keep observing | polling continues; later resets re-latch and count | core `second_reset_latches_again_and_is_counted`; HAL `a_second_reset_is_reported_again` |
| Keep communicating | fault `servo_reset`, warning cue, SOS LED, `status` count, display fault name | core `reset_makes_the_hand_limp_and_says_so`; serial and display text compiled, not exercised |

## Results

| Check | Where | Result |
|---|---|---|
| `tools/run_core_tests.py` | host, zig 0.13.0 / clang 18.1.6 | 124 run, 0 failed |
| `tools/run_hal_tests.py`, library 3.0.3 | host, same compiler | 13 run, 0 failed |
| `src/*.cpp` against stubs, `-Wall -Wextra -Werror` | host | 8 compiled |
| `verification` unit tests | host | 16 passed |
| Host checks | CI run 45724318183 | success |
| ESP32 target build, then HAL tests | same run | success; HAL 13 run, 0 failed |

Local syntax check note: `tools/check_app_syntax.py` was not run locally because
the zig compiler's `-fsyntax-only` reports a spurious error on this machine; the
same sources were compiled with `-c` instead. CI runs the script itself.

Target build at that run: RAM 26828 of 327680 bytes, flash 370005 of 1310720
bytes. `firmware.bin` SHA-256
`1b7e861ccbef4dfde4304a83887bc91f758892687623709d6ce718b789457e6c`.

## Negative evidence: do the tests fail when the code is wrong?

One defect at a time, code restored afterwards. All detected.

Core: reset handled after fault evaluation; fault clearing releases the latch;
`park()` releases the latch; no minimum limp time; queued commands survive.

Driver: no `reset()` before the frequency; torque not cut; poll only checks the
acknowledge; cache kept after an outage; no second read; configuration not
verified; cache kept across reconfiguration.

Two further defects were found by tests written before their fixes:

- resume passed through `Fault` for one tick (`modeOf = 6, expected 3`);
- a wrist command sent while limp executed on resume (`1800, expected 1500`).

Those two fixes came after the five core mutations were run.

## Chip behaviour relied on

Read from the text of the NXP PCA9685 data sheet: power-on `MODE1` has `SLEEP`
set and auto-increment clear; `PRE_SCALE` defaults to `0x1E` and is writable
only while asleep; no output while the oscillator is off; power-on reset needs
VDD below 0.2 V. Library behaviour was read from Adafruit PWM Servo Driver 3.0.3.

## Not established

- Any behaviour on a physical board, including detection latency, the effect of
  a shallow brownout, and tick timing during the roughly 15 ms reconfiguration.
- The chip model (`test/hal/fakes/pca9685_model.h`) is the author's reading of
  the data sheet.
- The ESP32 restarting together with the PCA9685, and resume after a bus outage
  with intact registers: unchanged, tracked in #9.
- No end-to-end test of `main.cpp` wiring; it is covered by review and the
  target compile only.

## Follow-up: author's re-check after the first revision (`115f8af`)

PR #10 was merged at `9d35445`. The author then found two defects in that
revision; `115f8af` fixes them in a follow-up pull request. The tables above
describe `9d35445` and are left as written.

| Defect in `9d35445` | Fix | Failing test first |
|---|---|---|
| A grip chosen from the console or display while limp was applied on resume; a host probe measured up to 1740 us pulse change (0 us after the fix) | `selectGrip`, `nextGrip`, `previousGrip` refused while waiting | `grip_changes_made_while_limp_are_dropped` |
| A press begun before the reset was rejected only by summed `dt`, which can run ahead of real time (#11) | the switch must also have been seen up since the reset (`HandInputs::buttonDown`) | `button_held_since_before_the_reset_never_counts_however_long`, `a_release_seen_before_a_second_reset_does_not_carry_over` |

Results on `115f8af`: core 127 run, 0 failed; HAL 13 run, 0 failed; 8
application sources compiled with `-Werror`; verification tests passed; push CI
run 45909883384 succeeded (host checks and ESP32 compile-only build).

Randomized host checks, written for this re-check and not part of the
repository: 3000 seeded runs (18 million control ticks, about 35 800 resets and
13 900 releases) found no energised tick while waiting and no release without
the stated consent; 4000 seeded driver runs (72 020 chip power cycles) found no
unreported power cycle. Both detected a deliberately planted defect.

Still untested: `main.cpp` wiring, including the one line that feeds
`buttonDown`, and everything on hardware.
