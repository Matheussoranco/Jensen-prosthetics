# Verification and release gates

This directory turns the repository's former “Future Work” list into explicit,
auditable gates. Software support is tested on the host, but **no physical result
is claimed until a real campaign JSON makes `verify_campaign.py` return 0**.

## Run a campaign

1. Copy `campaign_template.json` to a device-specific file outside the template.
2. Record the exact firmware commit, device serial, operator, UTC time, and each
   instrument's ID/calibration date.
3. Perform the procedures below and enter the raw results. Do not copy values
   from `example_campaign.json`; it exists only to demonstrate the schema.
4. Generate the release report:

   ```bash
   python verification/verify_campaign.py campaign.json --report report.md
   ```

Exit 0 is PASS, 1 is FAIL, 2 is PENDING/undersampled, and 3 means the evidence
file is malformed. Commit a passing campaign, its report, raw serial CSV, and
load-cell/cycle-rig logs together.

## Bench validation

Work on an unloaded, restrained hand with an accessible emergency power cut.
Use a current-limited bench supply for endpoint and stall work.

### Servo endpoints

Run `cal digit <name>` as described in `firmware/README.md`. For thumb flexion,
index, middle, ring, and pinky, record both saved endpoints and confirm visible
clearance at both mechanical limits. The verifier requires at least 200 µs of
usable electrical travel and a clearance pass; it does not decide whether a
particular tendon is safe by looking at pulse widths.

### Battery divider

At minimum, test near full, nominal, and low pack voltage. Measure at the pack
terminals with a calibrated DMM while logging `vb` telemetry. Adjust
`kBatteryDividerRatio`, rebuild, and repeat until maximum error is 2% or less.

### FSR fit

Tare unloaded sensors. Apply at least three traceable loads spanning the working
range to each fitted fingertip (thumb, index, middle), log the stable telemetry
value, then set `zeroCounts`, `spanCounts`, `referenceNewtons`, and `exponent`.
The default campaign gate is 20% mean absolute error. Keep loading geometry and
pad temperature consistent and record them in notes.

### Direct motor-stall sensing

Fit and calibrate a current channel for every mask bit enabled by
`set current.mask`. Restrain one unloaded digit at a time, command motion, and
measure time from current rise to that channel's pulse becoming zero. Confirm
the other channels remain controllable. The default gate is 300 ms, matching a
250 ms trip setting plus one 100 Hz control interval and logging allowance.
Never conduct this test on a person.

### Pattern recognition

Calibrate auxiliary sites from telemetry (`srN` raw baseline and `seN` envelope):

```text
pattern site 2 <baseline> <rest-noise-peak> <mvc-envelope>
pattern site 3 <baseline> <rest-noise-peak> <mvc-envelope>
set pattern.sites 4
pattern enable on
pattern train pinch
pattern train tripod
pattern train power
save
```

For each grip, run at least ten blinded, randomized trials, returning all sites
below the release threshold between trials. Record expected and predicted grip.
The gate is at least 90% overall and 80% for every trained grip. Pattern mode
replaces co-contraction selection; flexor/extensor still provide proportional
close/open velocity.

## Grasp force and cycle life

Measure whole-hand grasp force with a calibrated fixture, not by summing the
three fingertip FSR values. Run at least five power and five pinch trials.
The default evidence gates are:

- mean power grasp at least 45 N, the practical minimum reported in the review;
- maximum pinch no more than 65 N, the review's recommended fingertip maximum;
- 300,000 complete open/grasp/release cycles with no functional loss as the
  baseline durability benchmark; 500,000 cycles is retained as the service goal.

The force-control experiment should additionally retain commanded target,
measured force versus time, overshoot, and steady-state error. Reference [6]
motivates closed-loop force/sensory feedback, but the repository does not claim
a numeric result from that paper that was unavailable for independent checking.

The benchmark source is Belter et al., *JRRD* 50(5), 2013,
<https://doi.org/10.1682/JRRD.2011.10.0188>. The test report is engineering
evidence, not medical-device certification.

## Native SolidWorks release

Native files are not currently in the repository. When clean-template reauthored
files are available, run:

```bash
python verification/audit_solidworks_release.py path/to/native \
  --identity-term "full name" --identity-term "email@example.com" \
  --identity-term "workstation-name" --manifest cad-release-manifest.json
```

The screen hashes every `.SLDPRT`/`.SLDASM` and rejects visible metadata labels
or supplied identity terms. Because proprietary compressed regions remain
opaque, a passing automated screen is not release approval. Two people who did
not author the original files must independently inspect clean exports and sign
the manifest before `release_approved` can truthfully become `true`.
