# Design-stage channel trace

This trace links the [CAD-derived BOM](BOM.md), [board declarations](../firmware/src/board_config.h), [output driver](../firmware/src/hal/servo_bus.cpp) and [verification gates](../verification/README.md) at base revision a03222b. No fitted assembly has been identified.

| Signal | CAD or design record | Firmware mapping | Decision or evidence needed |
|---|---|---|---|
| Thumb flexion and opposition | One SG90 at the thumb, axis undocumented | PCA9685 channels 0 and 1 | Identify the physical axis and whether opposition-dependent grips are available. |
| Index, middle, ring, little finger | Four SG90s | Channels 2, 3, 4, 5 respectively, described as MG996R | Identify fitted parts and linkage range per digit before selecting calibration or current limits. |
| Wrist | One MG996R at the wrist | Optional channel 6 | Record whether the wrist is populated and its travel limits. |
| Force and contact | Four FSRs and five comparators | Thumb, index and middle use GPIO 35/32/33 and 25/19/23 respectively | Account for spare sensors. Record per-finger calibration and coverage. |
| EMG and current | EMG front end unspecified. No fitted current instrumentation documented. | GPIO 36/39 for primary EMG, auxiliary sites and all seven current channels disabled by default | Record front end, isolation and fitted measurement channels. An unavailable channel cannot substantiate a stall measurement. |
| Supply and battery | 2S pack and buck converter in BOM | GPIO 34 battery divider, GPIO 26 rail enable and GPIO 27 active-low output enable | Verify fitted divider, rail cut, reset-state behavior and independent disconnect on an authorized fixture. A GPIO state alone cannot demonstrate loss of torque. |
| Servo transport | PCA9685 in BOM | I2C address 0x40, nominal 50 Hz PWM | Measure actual frames and bus-fault behavior on a fixture. |
| Operator interface | Nextion, button, LEDs, haptic module and optional voice interface | Nextion UART 9600, button GPIO 4, status GPIO 13, heartbeat GPIO 2, haptic GPIO 14, reserved voice UART | Establish fitted peripherals and behavior during loss of UI communication. |

The existing BOM documents the actuator mismatch. Selecting the CAD as a deployment profile would give unmeasured limits the status of calibration. Selecting the firmware defaults would assume an extra thumb axis. Keep each as a separate design description until an identified assembly and measured channel record resolve the discrepancy.

The nominal 10 ms control tick and two-second watchdog are software settings. Capture worst-case tick interval and the actual disable signals during reset and bus failure as separate bench evidence. The example campaign is synthetic. An unavailable optional sensor or unbuilt assembly leaves the applicable physical claim pending.
