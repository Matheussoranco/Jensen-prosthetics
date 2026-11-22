# Evidence Pack — `Jensen-prosthetics #4` (Fault Behavior & Campaign Eligibility by Hardware Profile)

- **Repository:** `Matheussoranco/Jensen-prosthetics`
- **Issue:** `#4` (`Test fault behavior and campaign eligibility by hardware profile`, parent `#1`)
- **Trace:** `JENSEN-4`
- **Author:** Gemini 4 Neon (High reasoning effort, session `7f54c219-94aa-4eee-be8a-2152f9236c40`)
- **Branch / Worktree:** `sail/jensen-profile-validation` (`C:\Users\mathe\Documents\Code\Jensen-prosthetics-wt4`, base `origin/main` @ `7791097`)
- **Validation Scope:** **Host-only + compile-only CI.** No physical hand has been built, flashed, or bench-tested; all physical bench release gates remain `PENDING` until an authorized physical campaign is recorded on an identified assembly.
- **Verification Status:** `PASS` (host C++ core, stubbed app syntax, Python campaign verifier tests; compile-only ESP32 target build verified on PR CI)
- **Review & Merge Readiness:** `PENDING_REVIEW` (requires **two independent firmware reviewers** per `#4` and `hardware/profile-trace.md`)

---

## 1. Safety Defect Uncovered & Fixed (`firmware/lib/jensen_core/jensen/modes.h`, `modes.cpp`)

While executing the new `#4` host fault-injection tests, two tests (`profile_missing_aux_emg_calibration_fails_self_test_when_pattern_enabled` and `controller_corrupt_magic_or_version_fails_self_test_into_sto`) reproduced a **real firmware safety defect**:

- **Observed failure before fix:** When `HandConfig::isValid()` returned `false` (for example, corrupt `magic`/`version` or `pattern.enabled == true` with uncalibrated auxiliary EMG), `HandController::update()` dispatched `SystemEvent::SelfTestFailed` after `kSelfTestSeconds` (`0.4 s`), entering `ControlMode::SafeTorqueOff`. On the **very next 10 ms control tick**, `modes_.setFaults(safety_.any(), safety_.requiresSafeTorqueOff())` ran with `severeFault == false` and `!userParked_`, causing `ModeMachine::setFaults()` (`modes.cpp:52-55`) to **immediately exit `SafeTorqueOff` into `ControlMode::Active` and energize the servo rail (`servoRailEnabled = true`, `servoOutputEnabled = true`, non-zero `pulseUs`) with a structurally invalid configuration**. Additionally, `ButtonEvent::VeryLongPress` could unpark a hand that had failed self-test.
- **Minimal fix (+16 / -3 lines in `modes.h` & `modes.cpp`):** Added `bool selfTestFailed_ = false;` to `ModeMachine`:
  - Set `selfTestFailed_ = true` on `SystemEvent::SelfTestFailed` (cleared on `reset()` or `SystemEvent::SelfTestPassed`).
  - Guarded `ModeMachine::setFaults()` with `&& !selfTestFailed_` so fault-clear ticks cannot clear a failed boot self-test out of `SafeTorqueOff`.
  - Guarded `ButtonEvent::VeryLongPress` unpark with `if (severeFault_ || selfTestFailed_) return;` so button presses cannot re-energize servos after a failed self-test.
- **Regression coverage:** Extended `modes_self_test_failure_parks_the_hand` and verified both controller self-test failure tests pass (`109 test(s), 0 failed`). Because `modes.h`/`modes.cpp` compile into `jensen_core`, the PR CI `ESP32 target build (compile only, no upload)` job compiles and validates the updated ESP32 target artifacts.

---

## 2. Summary of Test & Campaign Gate Changes

### 2.1 Host C++ Fault Behavior & Profile Tests (`firmware/test/native/test_core.cpp`)
Added 9 new host control-core regression tests (`100` → `109` total tests) and strengthened `modes_self_test_failure_parks_the_hand`:
1. `profile_cad_without_thumb_opposition_or_current_channels_operates_safely`: disables `ThumbRot` & `Wrist` and sets `motorCurrent.presentMask = 0` (matching CAD BOM); verifies disabled outputs emit `0 µs` and injecting `5.0 A` on unfitted current channels does **not** fabricate `kFaultMotorStall`.
2. `profile_missing_fsr_channels_do_not_fabricate_contact_or_overforce`: disables all FSR channels (`present = false`), drives ADC counts to `4095.0f`, and confirms `contact == false`, `peakForce == 0.0f`, `kFaultOverForce == 0`.
3. `profile_missing_aux_emg_calibration_fails_self_test_when_pattern_enabled`: verifies enabling pattern mode without calibrated auxiliary EMG fails `HandConfig::isValid()` and latches `SafeTorqueOff` (`servoRailEnabled == false`, `servoOutputEnabled == false`, all `pulseUs == 0`).
4. `controller_corrupt_magic_or_version_fails_self_test_into_sto`: corrupts `HandConfig::magic` and `HandConfig::version` separately; verifies `SelfTest` latches `SafeTorqueOff` with all outputs de-energized across subsequent ticks.
5. `controller_invalid_digit_endpoints_raise_fault_and_enter_sto`: sets degenerate `Index` endpoints (`1500`/`1510 µs`); verifies `kFaultDigitCalibration`, `SafeTorqueOff`, and `pulseUs[Index] == 0`.
6. `controller_zero_and_negative_dt_are_clamped_and_preserve_slew_limits`: injects `dt = 0.0f` and `dt = -0.05f` during active flexion; confirms clamping prevents NaN or slew-rate bypass.
7. `controller_sustained_timing_discontinuity_raises_loop_overrun_and_recovers`: injects sustained `40 ms` ticks (`> 2 × 10 ms`); verifies `kFaultLoopOverrun` (`ControlMode::Fault`, warning severity, rail kept energized) and clean recovery to `ControlMode::Active` once nominal `10 ms` ticks resume.
8. `controller_single_stalled_tick_records_worst_overrun_without_false_sustained_fault`: injects one isolated `50 ms` stalled tick; verifies `loop().worstSeconds() > 0.049f` and `overrunCount() == 1` while average stays below `20 ms`.
9. `controller_severe_fault_wins_over_concurrent_button_press_and_recovers`: asserts `servoBusOk = false` concurrently with `ButtonEvent::Click`; verifies `SafeTorqueOff` wins over grip switching, removes all pulses, and recovers cleanly when bus health returns.

### 2.2 Profile-Aware Campaign Eligibility (`verification/verify_campaign.py`, `verification/campaign_template.json`, `verification/test_verify_campaign.py`, `verification/README.md`)
- Added **`Evidence provenance`** gate rejecting `metadata.synthetic: true`, placeholder device IDs (`EXAMPLE-NOT-A-REAL-TEST`, `SYNTHETIC`, `DUMMY`, `TBD`), all-zero commits (`0000000`), and placeholder operators (`example`).
- Added **`Hardware profile`** gate distinguishing `unidentified_design_stage`, `cad_design_stage` (5 SG90 digits + wrist, no thumb opposition, 0 current channels), `firmware_default` (6 digits incl. `thumb_rot`, 0 current channels), and `identified_assembly` (requires `assembly_id`, `actuators`, and `emg_front_end_isolated: true`). Design-stage profiles remain `PENDING` and can never produce an overall `PASS`.
- Made **`Servo endpoints`**, **`FSR force fit`**, and **`Direct stall detection`** profile-aware:
  - Missing profile channels (`thumb_rot`, `wrist`, or any fitted FSR/current channel) keep gates `PENDING`.
  - Reporting stall trials when `fitted_current_channels` is empty (or for any unfitted current/FSR channel) explicitly **FAILS** the gate.
- Added **`Control timing`** (`worst_tick_ms <= 10.0 ms`, `watchdog_reset_verified`) and **`Hardware disconnect / STO`** (`reset`, `OE`, `I2C fault`, `independent power cut`) physical gates — `PENDING` when absent.
- Added 13 unit tests in `verification/test_verify_campaign.py` and wired them into `.github/workflows/ci.yml` under `Host checks` (keeping compile-only `target-build` untouched).

---

## 3. Commands Executed & Observed Results

| # | Layer | Command | Exit Code | Observed Result |
|---|---|---|---|---|
| 1 | Baseline reproduction (before `modes` fix) | `python firmware/tools/run_core_tests.py` (`MSYS2 g++ 15.1.0`) | `1` | `109 test(s), 2 failed` (`profile_missing_aux_emg_calibration_fails_self_test_when_pattern_enabled`, `controller_corrupt_magic_or_version_fails_self_test_into_sto` exited STO into `Active`) |
| 2 | Host C++ core (after `modes` fix) | `python firmware/tools/run_core_tests.py` (`MSYS2 g++ 15.1.0`) | `0` | `109 test(s), 0 failed` |
| 3 | Host app syntax (stubs) | `python firmware/tools/check_app_syntax.py` | `0` | `all application sources compile against the stub API` (8 files) |
| 4 | Host Python verification | `python -m unittest discover -s verification -p "test_*.py" -v` | `0` | `Ran 13 tests in 0.501s — OK` |
| 5 | Target build (compile only) | GitHub Actions `ESP32 target build (compile only, no upload)` | Verified on PR CI | Compiles updated `jensen_core` (`modes.h`/`modes.cpp`) into ESP32 target image with zero upload |
| 6 | Physical validation | N/A | `PENDING` | No built hand or authorized bench campaign; physical gates retained |

---

## 4. Open Physical Requirements (Unchanged / Retained)

- Physical actuator identification (SG90 vs MG996R, thumb opposition axis, wrist population).
- Bench-measured worst-case control loop tick, watchdog reset torque loss, active-low OE / I2C bus-fault / independent power-cut disconnect behavior, EMG isolation, grasp force, and cycle life.
