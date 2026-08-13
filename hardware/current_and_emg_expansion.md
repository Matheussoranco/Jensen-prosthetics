# Current-sensing and multi-site EMG expansion

The control core accepts four calibrated EMG sites and one current measurement
per servo. The reference 30-pin ESP32 carrier does not expose enough unused ADC1
channels for all nine extra signals, so a populated build needs an external ADC
or a revised carrier. Do not put safety-critical analogue inputs on ESP32 ADC2;
it becomes unavailable when Wi-Fi is active.

## Motor current channels

Each fitted channel must provide a unipolar, galvanically appropriate signal in
the ADC's linear range, with bandwidth high enough to capture a servo stall.
A shunt/current-sense amplifier or Hall sensor may be used. Measure and enter its
zero-current voltage and volts-per-amp in `board_config.h`, route the resulting
amperes into `HandInputs::motorCurrentAmps`, and enable the corresponding digit
bit with `set current.mask`.

Digit bits follow firmware order: 0 thumb flex, 1 thumb opposition, 2 index,
3 middle, 4 ring, 5 pinky, 6 wrist. `current.trip` must be established from
measured free-running and restrained-current distributions, below destructive
current but above worst-case normal transients. A trip removes pulses only from
that channel; a `motor_stall` warning remains until measured current stays below
`current.clear` for `current.clear_s`.

Current sensors, ADC, ESP32, PCA9685, and servo supply must share a defined
reference unless the selected sensors are isolated. Size shunts, traces, and
connectors for worst-case aggregate servo current; do not route motor current
through the ESP32 board.

## Auxiliary EMG sites

Sites 0 and 1 remain the primary flexor/extensor channels on GPIO 36/39. Sites 2
and 3 are optional and default to `-1` pins. Condition every auxiliary electrode
to the same safe ADC range and sample it at the 100 Hz control tick or faster.
Populate `HandInputs::emgAuxCounts` from local ADC1 pins or an external ADC.

Pattern recognition is deliberately small and deterministic: calibrated site
activations are compared with trained centroids, then rejected on low signal,
excess distance, or ambiguity. A candidate must remain stable for the dwell
time and must return below the release threshold before it can fire again. This
keeps classification behavior host-testable and bounded on the ESP32.

Electrodes for experiments belong on a bench simulator or a properly supervised
research setup. This repository does not supply isolation, leakage-current, or
IEC 60601 compliance needed for a clinical electrode path.
