# Engineering Interaction Report — `Jensen-prosthetics #4`

- **Trace:** `JENSEN-4`
- **Repository:** `Matheussoranco/Jensen-prosthetics`
- **Issue:** `#4` (`Test fault behavior and campaign eligibility by hardware profile`, parent `#1`)
- **Author:** Gemini 4 Neon (High, session `7f54c219-94aa-4eee-be8a-2152f9236c40`)
- **Branch:** `sail/jensen-profile-validation`

## Coordination Summary

1. **Unblocked Verification & Claim:** Verified PR #5 (`486bfb2`, channel traceability) and PR #6 (`7791097`, compile-only ESP32 target build) were merged to `main`. Acquired exclusive `.sail-mesh` lease (`0736d4abb9976c97a68fdf01`, re-acquired `064e9e24e46ad55305cbba53` after overnight pause), notified `lead`, posted structured `<!-- sail-claim:... -->` on `Matheussoranco/Jensen-prosthetics#4#issuecomment-7071867470`, and worked in isolated worktree `Jensen-prosthetics-wt4`.
2. **Real Safety Defect Caught & Fixed:** Executing the new host fault-injection tests exposed that `ModeMachine::setFaults(anyFault, false)` cleared `SafeTorqueOff` 10 ms after `SystemEvent::SelfTestFailed`, energizing servos on a structurally invalid/corrupt `HandConfig`. Fixed minimally in `firmware/lib/jensen_core/jensen/modes.h` and `modes.cpp` (+16/-3 lines) by latching `selfTestFailed_` across both `setFaults()` and `ButtonEvent::VeryLongPress`.
3. **Profile-Aware Eligibility & Regression Coverage:** Added 9 host C++ fault/profile tests (`100` → `109`), strengthened `modes_self_test_failure_parks_the_hand`, upgraded `verification/verify_campaign.py` with profile-aware eligibility + anti-synthetic/example provenance checks + physical timing/disconnect gates, and added 13 automated tests in `verification/test_verify_campaign.py` wired into `.github/workflows/ci.yml`.
4. **Local Host Verification:** Compiled and ran all 109 C++ host control-core tests + 8 stubbed application syntax checks using local MSYS2 `g++ 15.1.0`, plus all 13 Python campaign verification tests (`109/109`, `8/8`, `13/13`).
5. **Two-Reviewer Firmware Gate:** Requested two independent firmware reviews (Claude Code + Engineering Lead) before integration, highlighting the `modes.h`/`modes.cpp` production safety fix so both firmware reviewers inspect it and the updated ESP32 compile-only CI build.

See `events.jsonl` in this directory for the machine-readable event log.
