// Host-side tests for the Jensen Hand control core.
//
// Run with:  python firmware/tools/run_core_tests.py
// or under PlatformIO with:  pio test -e native
#include "test_harness.h"

#include "jensen/button_gestures.h"
#include "jensen/calibration.h"
#include "jensen/digit.h"
#include "jensen/emg.h"
#include "jensen/filter.h"
#include "jensen/force.h"
#include "jensen/grip.h"
#include "jensen/hand_config.h"
#include "jensen/hand_controller.h"
#include "jensen/modes.h"
#include "jensen/safety.h"
#include "jensen/util.h"

using namespace jensen;

// =============================================================================
// util
// =============================================================================

TEST(util_clamp_and_lerp) {
    CHECK_NEAR(clamp(5.0f, 0.0f, 3.0f), 3.0f, 1e-6);
    CHECK_NEAR(clamp(-1.0f, 0.0f, 3.0f), 0.0f, 1e-6);
    CHECK_NEAR(clamp01(1.5f), 1.0f, 1e-6);
    CHECK_NEAR(lerp(10.0f, 20.0f, 0.25f), 12.5f, 1e-6);
    CHECK_NEAR(lerpClamped(10.0f, 20.0f, 2.0f), 20.0f, 1e-6);
}

TEST(util_map_clamped_does_not_extrapolate) {
    // This is the specific defect inherited from HACKberry's control law:
    // `map(v, 40, 100, 0, 6)` was evaluated for v below 40 as well, so inputs
    // that the surrounding code treats as "inside the deadband" produced a
    // small negative speed and the hand crept open.
    CHECK_LT(mapRange(35.0f, 40.0f, 100.0f, 0.0f, 6.0f), 0.0f);   // the old behaviour
    CHECK_NEAR(mapClamped(35.0f, 40.0f, 100.0f, 0.0f, 6.0f), 0.0f, 1e-6);
    CHECK_NEAR(mapClamped(100.0f, 40.0f, 100.0f, 0.0f, 6.0f), 6.0f, 1e-6);
    CHECK_NEAR(mapClamped(70.0f, 40.0f, 100.0f, 0.0f, 6.0f), 3.0f, 1e-6);
    // Reversed output range must clamp too.
    CHECK_NEAR(mapClamped(0.0f, 40.0f, 100.0f, 6.0f, 0.0f), 6.0f, 1e-6);
}

TEST(util_map_range_handles_degenerate_input) {
    CHECK_NEAR(mapRange(5.0f, 2.0f, 2.0f, 7.0f, 9.0f), 7.0f, 1e-6);
}

TEST(util_approach) {
    CHECK_NEAR(approach(0.0f, 1.0f, 0.25f), 0.25f, 1e-6);
    CHECK_NEAR(approach(1.0f, 0.0f, 0.25f), 0.75f, 1e-6);
    CHECK_NEAR(approach(0.9f, 1.0f, 0.25f), 1.0f, 1e-6);  // no overshoot
}

// =============================================================================
// filters
// =============================================================================

TEST(filter_ema_reaches_step_and_respects_time_constant) {
    Ema ema(0.1f);
    ema.reset(0.0f);
    // One time constant of a first-order step response is ~63%.
    for (int i = 0; i < 100; ++i) ema.update(1.0f, 0.001f);
    CHECK_NEAR(ema.value(), 0.63f, 0.03);
    for (int i = 0; i < 2000; ++i) ema.update(1.0f, 0.001f);
    CHECK_NEAR(ema.value(), 1.0f, 0.01);
}

TEST(filter_ema_first_sample_seeds_when_unprimed) {
    Ema ema(10.0f);
    ema.clear();
    CHECK_NEAR(ema.update(7.5f, 0.01f), 7.5f, 1e-6);
}

TEST(filter_median3_rejects_single_spike) {
    Median3 median;
    median.update(10.0f);
    median.update(10.0f);
    CHECK_NEAR(median.update(1000.0f), 10.0f, 1e-6);
    CHECK_NEAR(median.update(10.0f), 10.0f, 1e-6);
}

TEST(filter_schmitt_has_hysteresis) {
    Schmitt schmitt(0.6f, 0.4f);
    CHECK_FALSE(schmitt.update(0.5f));
    CHECK(schmitt.update(0.7f));
    CHECK(schmitt.update(0.5f));   // still latched
    CHECK_FALSE(schmitt.update(0.3f));
}

TEST(filter_schmitt_tolerates_swapped_thresholds) {
    Schmitt schmitt(0.4f, 0.6f);  // deliberately backwards
    CHECK_FALSE(schmitt.update(0.5f));
    CHECK(schmitt.update(0.7f));
    CHECK_FALSE(schmitt.update(0.3f));
}

TEST(filter_rate_limiter) {
    RateLimiter limiter(2.0f);
    limiter.reset(0.0f);
    limiter.update(1.0f, 0.1f);
    CHECK_NEAR(limiter.value(), 0.2f, 1e-6);
    for (int i = 0; i < 20; ++i) limiter.update(1.0f, 0.1f);
    CHECK_NEAR(limiter.value(), 1.0f, 1e-6);
}

TEST(filter_envelope_is_asymmetric) {
    Envelope envelope;
    envelope.configure(0.01f, 0.5f);
    envelope.setBaseline(100.0f);
    envelope.reset(0.0f);

    for (int i = 0; i < 200; ++i) envelope.update(i % 2 ? 200.0f : 0.0f, 0.001f);
    const float attacked = envelope.value();
    CHECK_GT(attacked, 90.0f);  // rectified amplitude about the baseline is 100

    for (int i = 0; i < 200; ++i) envelope.update(100.0f, 0.001f);
    // Release is 50x slower, so 200 ms barely moves it.
    CHECK_GT(envelope.value(), attacked * 0.5f);
}

// =============================================================================
// EMG
// =============================================================================

namespace {

/// Feeds a synthetic bipolar EMG burst: `amplitude` counts either side of the
/// baseline, which is what the rectifying envelope is designed to measure.
void feedEmg(EmgChannel& channel, float baseline, float amplitude, int samples, float dt) {
    for (int i = 0; i < samples; ++i) {
        channel.update(baseline + (i % 2 ? amplitude : -amplitude), dt);
    }
}

EmgCalibration runCalibration(EmgChannel& channel, float baseline, float restAmplitude,
                              float effortAmplitude) {
    channel.beginCalibration();
    for (int i = 0; i < 2000; ++i) {
        channel.observeRest(baseline + (i % 2 ? restAmplitude : -restAmplitude), 0.001f);
    }
    for (int i = 0; i < 1000; ++i) {
        channel.observeEffort(baseline + (i % 2 ? effortAmplitude : -effortAmplitude), 0.001f);
    }
    return channel.finishCalibration();
}

}  // namespace

TEST(emg_calibration_captures_baseline_noise_and_mvc) {
    EmgChannel channel;
    channel.configure(EmgConfig{});

    const EmgCalibration cal = runCalibration(channel, 512.0f, 3.0f, 600.0f);
    CHECK(cal.valid);
    CHECK(cal.isUsable());
    CHECK_NEAR(cal.baselineCounts, 512.0f, 2.0);
    CHECK_LT(cal.noiseCounts, 20.0f);
    CHECK_NEAR(cal.mvcCounts, 600.0f, 40.0);
}

TEST(emg_calibration_rejects_a_dead_electrode) {
    // A loose electrode produces a contraction that is indistinguishable from
    // rest. HACKberry would happily rescale that into full-range control.
    EmgChannel channel;
    channel.configure(EmgConfig{});
    const EmgCalibration cal = runCalibration(channel, 512.0f, 3.0f, 6.0f);
    CHECK_FALSE(cal.isUsable());
    CHECK_FALSE(cal.valid);
}

TEST(emg_without_calibration_produces_no_activation) {
    EmgChannel channel;
    channel.configure(EmgConfig{});
    feedEmg(channel, 512.0f, 900.0f, 500, 0.001f);
    CHECK_NEAR(channel.activation(), 0.0f, 1e-6);
}

TEST(emg_activation_spans_rest_to_mvc) {
    EmgChannel channel;
    channel.configure(EmgConfig{});
    runCalibration(channel, 512.0f, 3.0f, 600.0f);

    feedEmg(channel, 512.0f, 3.0f, 500, 0.001f);
    CHECK_NEAR(channel.activation(), 0.0f, 1e-6);

    feedEmg(channel, 512.0f, 600.0f, 800, 0.001f);
    CHECK_GT(channel.activation(), 0.9f);
    CHECK(channel.activation() <= 1.0f);
}

TEST(emg_deadband_suppresses_low_level_activity) {
    EmgChannel channel;
    EmgConfig config;
    config.deadband = 0.25f;
    config.gamma = 1.0f;
    channel.configure(config);
    runCalibration(channel, 512.0f, 2.0f, 1000.0f);

    // 10% of MVC sits inside a 25% deadband.
    feedEmg(channel, 512.0f, 100.0f, 800, 0.001f);
    CHECK_NEAR(channel.activation(), 0.0f, 1e-6);
    CHECK_GT(channel.normalized(), 0.02f);  // ...but the raw normalisation sees it
}

// =============================================================================
// drive law
// =============================================================================

TEST(drive_single_channel_deadband_has_no_creep) {
    // The regression test for the HACKberry mapping defect. Every activation
    // inside the hold band must produce exactly zero velocity.
    ProportionalDrive drive;
    DriveConfig config;
    config.mode = DriveMode::SingleChannel;
    config.singleOpenBelow = 0.12f;
    config.singleHoldBelow = 0.32f;
    config.openSpeed = 2.8f;
    config.closeSpeed = 2.2f;
    drive.configure(config);

    for (float a = 0.12f; a < 0.32f; a += 0.01f) {
        CHECK_NEAR(drive.update(a, 0.0f, 0.01f), 0.0f, 1e-6);
    }
    CHECK_NEAR(drive.update(0.05f, 0.0f, 0.01f), -2.8f, 1e-6);
    CHECK_NEAR(drive.update(1.0f, 0.0f, 0.01f), 2.2f, 1e-6);
    // Just above the hold band the hand starts closing from zero, not from a
    // step.
    CHECK_NEAR(drive.update(0.32f, 0.0f, 0.01f), 0.0f, 1e-6);
    CHECK_GT(drive.update(0.5f, 0.0f, 0.01f), 0.0f);
}

TEST(drive_dual_channel_opposes) {
    ProportionalDrive drive;
    DriveConfig config;
    config.mode = DriveMode::DualChannel;
    config.closeSpeed = 2.0f;
    config.openSpeed = 3.0f;
    drive.configure(config);

    CHECK_NEAR(drive.update(1.0f, 0.0f, 0.01f), 2.0f, 1e-6);
    CHECK_NEAR(drive.update(0.0f, 1.0f, 0.01f), -3.0f, 1e-6);
    CHECK_NEAR(drive.update(0.3f, 0.2f, 0.01f), 0.3f * 2.0f - 0.2f * 3.0f, 1e-6);
}

TEST(drive_co_contraction_fires_once_and_rearms) {
    ProportionalDrive drive;
    DriveConfig config;
    config.mode = DriveMode::DualChannel;
    config.coContractionLevel = 0.6f;
    config.coContractionReleaseLevel = 0.3f;
    config.coContractionSeconds = 0.2f;
    drive.configure(config);

    // Held below the dwell time: nothing fires.
    for (int i = 0; i < 10; ++i) drive.update(0.8f, 0.8f, 0.01f);
    CHECK_FALSE(drive.consumeCoContraction());

    // Past the dwell time: exactly one event.
    for (int i = 0; i < 20; ++i) drive.update(0.8f, 0.8f, 0.01f);
    CHECK(drive.consumeCoContraction());
    CHECK_FALSE(drive.consumeCoContraction());

    // Motion is suppressed while the gesture is held, so the hand does not
    // lurch toward whichever channel is stronger.
    CHECK_NEAR(drive.update(0.9f, 0.7f, 0.01f), 0.0f, 1e-6);

    // Still held: must not repeat.
    for (int i = 0; i < 100; ++i) drive.update(0.8f, 0.8f, 0.01f);
    CHECK_FALSE(drive.consumeCoContraction());

    // Release both, then contract again: re-arms.
    for (int i = 0; i < 20; ++i) drive.update(0.1f, 0.1f, 0.01f);
    for (int i = 0; i < 30; ++i) drive.update(0.8f, 0.8f, 0.01f);
    CHECK(drive.consumeCoContraction());
}

// =============================================================================
// digits
// =============================================================================

namespace {
DigitCalibration makeCal(uint16_t openUs, uint16_t closedUs, uint16_t minUs = 700,
                         uint16_t maxUs = 2300) {
    DigitCalibration cal;
    cal.openUs = openUs;
    cal.closedUs = closedUs;
    cal.minUs = minUs;
    cal.maxUs = maxUs;
    cal.enabled = true;
    return cal;
}
}  // namespace

TEST(digit_calibration_validity) {
    CHECK(makeCal(1000, 2000).isValid());
    CHECK_FALSE(makeCal(1500, 1520).isValid());          // travel too small
    CHECK_FALSE(makeCal(1000, 2000, 1200, 1800).isValid());  // endpoints outside guards
    CHECK_FALSE(makeCal(1000, 2000, 1500, 1500).isValid());  // degenerate guard band
}

TEST(digit_pulse_mapping_handles_both_directions) {
    const DigitCalibration forward = makeCal(1000, 2000);
    CHECK_EQ(forward.pulseFor(0.0f), 1000);
    CHECK_EQ(forward.pulseFor(1.0f), 2000);
    CHECK_EQ(forward.pulseFor(0.5f), 1500);

    // A reversed servo is expressed purely by the endpoint order — nothing
    // downstream needs a "reversed" flag or a left/right branch.
    const DigitCalibration reversed = makeCal(2000, 1000);
    CHECK_EQ(reversed.pulseFor(0.0f), 2000);
    CHECK_EQ(reversed.pulseFor(1.0f), 1000);
    CHECK_EQ(reversed.pulseFor(0.25f), 1750);
}

TEST(digit_guard_band_clamps_output) {
    DigitCalibration cal = makeCal(900, 2100, 1000, 2000);
    // Endpoints outside the guards make the calibration invalid, but the
    // mapping itself must still refuse to emit an out-of-range pulse.
    CHECK_EQ(cal.pulseFor(0.0f), 1000);
    CHECK_EQ(cal.pulseFor(1.0f), 2000);
}

TEST(digit_controller_rate_limits) {
    DigitController digit;
    digit.setCalibration(makeCal(1000, 2000));
    digit.setSpeed(2.0f);  // full travel in 500 ms
    digit.snapTo(0.0f);
    digit.setTarget(1.0f);

    digit.update(0.1f);
    CHECK_NEAR(digit.closure(), 0.2f, 1e-5);
    CHECK_EQ(digit.pulseUs(), 1200);
    CHECK_FALSE(digit.settled());

    for (int i = 0; i < 10; ++i) digit.update(0.05f);
    CHECK_NEAR(digit.closure(), 1.0f, 1e-5);
    CHECK(digit.settled());
    CHECK_EQ(digit.pulseUs(), 2000);
}

TEST(digit_controller_lock_freezes_motion) {
    DigitController digit;
    digit.setCalibration(makeCal(1000, 2000));
    digit.setSpeed(2.0f);
    digit.snapTo(0.5f);
    digit.setLocked(true);
    digit.setTarget(1.0f);
    for (int i = 0; i < 20; ++i) digit.update(0.05f);
    CHECK_NEAR(digit.closure(), 0.5f, 1e-6);

    digit.setLocked(false);
    for (int i = 0; i < 20; ++i) digit.update(0.05f);
    CHECK_NEAR(digit.closure(), 1.0f, 1e-5);
}

TEST(digit_closure_limit_caps_travel_but_not_opening) {
    DigitController digit;
    digit.setCalibration(makeCal(1000, 2000));
    digit.setSpeed(5.0f);
    digit.snapTo(0.0f);
    digit.setTarget(1.0f);
    digit.setClosureLimit(0.4f);
    for (int i = 0; i < 20; ++i) digit.update(0.05f);
    CHECK_NEAR(digit.closure(), 0.4f, 1e-5);

    // Opening is never restricted by the force loop.
    digit.setTarget(0.0f);
    for (int i = 0; i < 20; ++i) digit.update(0.05f);
    CHECK_NEAR(digit.closure(), 0.0f, 1e-5);
}

TEST(digit_disabled_or_invalid_emits_no_pulse) {
    DigitController digit;
    DigitCalibration cal = makeCal(1000, 2000);
    cal.enabled = false;
    digit.setCalibration(cal);
    digit.snapTo(0.5f);
    CHECK_EQ(digit.pulseUs(), 0);

    digit.setCalibration(makeCal(1500, 1510));  // enabled but no usable travel
    CHECK_EQ(digit.pulseUs(), 0);
}

TEST(digit_idle_release_only_when_open) {
    DigitController digit;
    digit.setCalibration(makeCal(1000, 2000));
    digit.setSpeed(5.0f);
    digit.setIdleRelease(true, 0.03f, 0.5f);
    digit.snapTo(0.0f);

    for (int i = 0; i < 20; ++i) digit.update(0.05f);
    CHECK(digit.released());
    CHECK_EQ(digit.pulseUs(), 0);

    // Closing on an object must re-energise the channel and stay energised.
    digit.setTarget(0.8f);
    digit.update(0.05f);
    CHECK_FALSE(digit.released());
    for (int i = 0; i < 60; ++i) digit.update(0.05f);
    CHECK_FALSE(digit.released());
    CHECK_EQ(digit.pulseUs(), 1800);
}

// =============================================================================
// grips
// =============================================================================

TEST(grip_pose_interpolates_between_endpoints) {
    const GraspPose open = GripLibrary::pose(GripId::Power, 0.0f);
    const GraspPose closed = GripLibrary::pose(GripId::Power, 1.0f);
    const GraspPose half = GripLibrary::pose(GripId::Power, 0.5f);
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        CHECK_NEAR(half.closure[i], 0.5f * (open.closure[i] + closed.closure[i]), 1e-5);
    }
    CHECK_NEAR(closed[Digit::Index], 1.0f, 1e-6);
    CHECK_NEAR(open[Digit::Index], 0.0f, 1e-6);
}

TEST(grip_point_keeps_the_index_extended) {
    for (float aperture = 0.0f; aperture <= 1.0f; aperture += 0.1f) {
        const GraspPose pose = GripLibrary::pose(GripId::Point, aperture);
        CHECK_NEAR(pose[Digit::Index], 0.0f, 1e-6);
    }
    CHECK_GT(GripLibrary::pose(GripId::Point, 1.0f)[Digit::Middle], 0.9f);
}

TEST(grip_key_moves_only_the_thumb) {
    const GraspPose open = GripLibrary::pose(GripId::LateralKey, 0.0f);
    const GraspPose closed = GripLibrary::pose(GripId::LateralKey, 1.0f);
    CHECK_GT(closed[Digit::ThumbFlex] - open[Digit::ThumbFlex], 0.5f);
    CHECK_NEAR(closed[Digit::Middle], open[Digit::Middle], 1e-6);
    CHECK_NEAR(closed[Digit::ThumbRot], open[Digit::ThumbRot], 1e-6);
}

TEST(grip_rest_ignores_aperture) {
    const GraspPose open = GripLibrary::pose(GripId::Rest, 0.0f);
    const GraspPose closed = GripLibrary::pose(GripId::Rest, 1.0f);
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        CHECK_NEAR(open.closure[i], closed.closure[i], 1e-6);
    }
}

TEST(grip_hook_parks_the_thumb) {
    const GraspPose closed = GripLibrary::pose(GripId::Hook, 1.0f);
    CHECK_NEAR(closed[Digit::ThumbFlex], 0.0f, 1e-6);
    CHECK_NEAR(closed[Digit::ThumbRot], 0.0f, 1e-6);
    CHECK_NEAR(closed[Digit::Pinky], 1.0f, 1e-6);
}

TEST(grip_selector_skips_disabled_patterns) {
    GripSelector selector;
    selector.setEnabledMask(0);  // Rest is forced back on
    CHECK(selector.enabled(GripId::Rest));
    CHECK_EQ(selector.enabledCount(), 1);
    CHECK_EQ(static_cast<int>(selector.next()), static_cast<int>(GripId::Rest));

    selector.setEnabled(GripId::Power, true);
    selector.setEnabled(GripId::Hook, true);
    selector.select(GripId::Power);
    CHECK_EQ(static_cast<int>(selector.next()), static_cast<int>(GripId::Hook));
    CHECK_EQ(static_cast<int>(selector.next()), static_cast<int>(GripId::Rest));
    CHECK_EQ(static_cast<int>(selector.previous()), static_cast<int>(GripId::Hook));
}

TEST(grip_selector_never_lands_on_a_disabled_pattern) {
    GripSelector selector;
    selector.select(GripId::Pinch);
    selector.setEnabled(GripId::Pinch, false);
    CHECK_FALSE(selector.enabled(GripId::Pinch));
    CHECK(selector.enabled(selector.current()));
}

// =============================================================================
// force
// =============================================================================

namespace {
ForceCalibration makeForceCal(bool present = true, float exponent = 1.0f) {
    ForceCalibration cal;
    cal.present = present;
    cal.zeroCounts = 60;
    cal.spanCounts = 2060;  // 2000 counts of span
    cal.referenceNewtons = 20.0f;
    cal.exponent = exponent;
    return cal;
}

/// Raw counts that map to a given force under `makeForceCal` with exponent 1.
float countsForNewtons(float newtons) {
    return 60.0f + (newtons / 20.0f) * 2000.0f;
}

void settleForce(ForceSensor& sensor, float newtons, int samples = 400, float dt = 0.001f) {
    for (int i = 0; i < samples; ++i) sensor.update(countsForNewtons(newtons), dt);
}
}  // namespace

TEST(force_calibration_maps_counts_to_newtons) {
    const ForceCalibration cal = makeForceCal();
    CHECK_NEAR(cal.newtons(60.0f), 0.0f, 1e-6);
    CHECK_NEAR(cal.newtons(2060.0f), 20.0f, 1e-4);
    CHECK_NEAR(cal.newtons(1060.0f), 10.0f, 1e-4);
    CHECK_NEAR(cal.newtons(0.0f), 0.0f, 1e-6);  // below zero point
}

TEST(force_absent_sensor_reads_zero) {
    ForceSensor sensor;
    sensor.configure(ForceSensorConfig{});
    sensor.setCalibration(makeForceCal(false));
    settleForce(sensor, 15.0f);
    CHECK_NEAR(sensor.force(), 0.0f, 1e-6);
    CHECK_FALSE(sensor.contact());
}

TEST(force_contact_uses_hysteresis) {
    ForceSensor sensor;
    ForceSensorConfig config;
    config.contactOnNewtons = 1.0f;
    config.contactOffNewtons = 0.5f;
    sensor.configure(config);
    sensor.setCalibration(makeForceCal());

    settleForce(sensor, 0.2f);
    CHECK_FALSE(sensor.contact());
    settleForce(sensor, 3.0f);
    CHECK(sensor.contact());
    settleForce(sensor, 0.7f);
    CHECK(sensor.contact());  // between the thresholds: stays latched
    settleForce(sensor, 0.1f);
    CHECK_FALSE(sensor.contact());
}

TEST(force_slip_detected_on_rapid_unload_while_gripping) {
    ForceSensor sensor;
    ForceSensorConfig config;
    config.slipRateNewtonsPerSec = -30.0f;
    config.smoothingSeconds = 0.005f;
    config.derivativeSeconds = 0.005f;
    sensor.configure(config);
    sensor.setCalibration(makeForceCal());

    settleForce(sensor, 12.0f);
    CHECK(sensor.contact());
    CHECK_FALSE(sensor.consumeSlip());

    // Drop from 12 N to 4 N over 20 ms: -400 N/s.
    for (int i = 0; i < 20; ++i) sensor.update(countsForNewtons(12.0f - 0.4f * i), 0.001f);
    CHECK(sensor.consumeSlip());
    CHECK_FALSE(sensor.consumeSlip());  // one-shot
}

TEST(force_controller_limits_closure_under_load) {
    ForceSensor sensors[kGraspDigitCount];
    for (auto& sensor : sensors) {
        sensor.configure(ForceSensorConfig{});
        sensor.setCalibration(makeForceCal(false));
    }
    sensors[index(Digit::Index)].setCalibration(makeForceCal(true));

    GripForceController controller;
    GripForceConfig config;
    config.targetNewtons = 10.0f;
    config.integralGain = 0.5f;
    controller.configure(config);
    controller.reset();

    // Hand closed on something that is pushing back harder than the setpoint.
    for (int i = 0; i < 500; ++i) {
        sensors[index(Digit::Index)].update(countsForNewtons(18.0f), 0.005f);
        controller.update(sensors, kGraspDigitCount, 0.8f, 0.005f);
    }
    CHECK(controller.contact());
    CHECK_LT(controller.closureLimit(), 0.9f);
    CHECK_NEAR(controller.peakForce(), 18.0f, 0.5);
}

TEST(force_controller_releases_limit_when_the_object_is_gone) {
    ForceSensor sensors[kGraspDigitCount];
    for (auto& sensor : sensors) {
        sensor.configure(ForceSensorConfig{});
        sensor.setCalibration(makeForceCal(false));
    }
    sensors[index(Digit::Index)].setCalibration(makeForceCal(true));

    GripForceController controller;
    GripForceConfig config;
    config.targetNewtons = 10.0f;
    config.integralGain = 0.5f;
    config.releasePerSecond = 2.0f;
    controller.configure(config);
    controller.reset();

    for (int i = 0; i < 500; ++i) {
        sensors[index(Digit::Index)].update(countsForNewtons(18.0f), 0.005f);
        controller.update(sensors, kGraspDigitCount, 0.8f, 0.005f);
    }
    CHECK_LT(controller.closureLimit(), 0.9f);

    for (int i = 0; i < 500; ++i) {
        sensors[index(Digit::Index)].update(countsForNewtons(0.0f), 0.005f);
        controller.update(sensors, kGraspDigitCount, 0.8f, 0.005f);
    }
    CHECK_FALSE(controller.contact());
    CHECK_NEAR(controller.closureLimit(), 1.0f, 1e-3);
}

TEST(force_controller_tightens_on_slip) {
    ForceSensor sensors[kGraspDigitCount];
    for (auto& sensor : sensors) {
        sensor.configure(ForceSensorConfig{});
        sensor.setCalibration(makeForceCal(false));
    }
    ForceSensorConfig sensorConfig;
    sensorConfig.smoothingSeconds = 0.005f;
    sensorConfig.derivativeSeconds = 0.005f;
    sensorConfig.slipRateNewtonsPerSec = -30.0f;
    sensors[index(Digit::Index)].configure(sensorConfig);
    sensors[index(Digit::Index)].setCalibration(makeForceCal(true));

    GripForceController controller;
    GripForceConfig config;
    config.targetNewtons = 10.0f;
    config.integralGain = 0.0f;  // isolate the slip response
    config.slipTightenStep = 0.05f;
    controller.configure(config);
    controller.reset();

    for (int i = 0; i < 400; ++i) {
        sensors[index(Digit::Index)].update(countsForNewtons(9.0f), 0.001f);
        controller.update(sensors, kGraspDigitCount, 0.95f, 0.001f);
    }
    const uint32_t before = controller.slipCount();

    for (int i = 0; i < 20; ++i) {
        sensors[index(Digit::Index)].update(countsForNewtons(9.0f - 0.3f * i), 0.001f);
        controller.update(sensors, kGraspDigitCount, 0.95f, 0.001f);
    }
    CHECK_GT(controller.slipCount(), before);
}

TEST(force_controller_target_scales_per_grip) {
    GripForceController controller;
    GripForceConfig config;
    config.targetNewtons = 12.0f;
    controller.configure(config);
    CHECK_NEAR(controller.targetNewtons(), 12.0f, 1e-6);
    controller.setTargetScale(0.5f);
    CHECK_NEAR(controller.targetNewtons(), 6.0f, 1e-6);
    controller.setTargetScale(0.0f);  // rejected; a zero setpoint is not useful
    CHECK_NEAR(controller.targetNewtons(), 12.0f, 1e-6);
}

// =============================================================================
// safety
// =============================================================================

TEST(battery_reports_charge_and_latches_with_hysteresis) {
    BatteryMonitor battery;
    BatteryConfig config;
    config.cellCount = 2;
    config.warnCellVolts = 3.45f;
    config.criticalCellVolts = 3.20f;
    config.hysteresisVolts = 0.06f;
    config.smoothingSeconds = 0.01f;
    battery.configure(config);
    battery.reset();

    auto settle = [&](float volts) {
        for (int i = 0; i < 200; ++i) battery.update(volts, 0.01f);
    };

    settle(8.4f);
    CHECK_NEAR(battery.cellVolts(), 4.2f, 0.01);
    CHECK_NEAR(battery.stateOfCharge(), 1.0f, 0.02);
    CHECK_FALSE(battery.low());
    CHECK_FALSE(battery.critical());

    settle(6.8f);  // 3.40 V/cell
    CHECK(battery.low());
    CHECK_FALSE(battery.critical());

    settle(6.2f);  // 3.10 V/cell
    CHECK(battery.critical());

    settle(6.45f);  // 3.225 V/cell — above the threshold but inside hysteresis
    CHECK(battery.critical());

    settle(6.6f);  // 3.30 V/cell — clears critical, still low
    CHECK_FALSE(battery.critical());
    CHECK(battery.low());
}

TEST(battery_first_sample_seeds_the_filter) {
    // Regression: priming a 1.5 s filter with 0 V made every boot look like a
    // flat pack, which parked the hand for the first seconds after power-up.
    BatteryMonitor battery;
    BatteryConfig config;
    config.smoothingSeconds = 1.5f;
    battery.configure(config);
    battery.reset();
    battery.update(8.0f, 0.01f);
    CHECK_NEAR(battery.packVolts(), 8.0f, 1e-3);
    CHECK_FALSE(battery.critical());
}

TEST(hold_guard_backs_off_after_sustained_load_and_recovers) {
    HoldGuard guard;
    HoldGuardConfig config;
    config.engageNewtons = 8.0f;
    config.holdSeconds = 2.0f;
    config.coolSeconds = 1.0f;
    config.backoffScale = 0.5f;
    guard.configure(config);
    guard.reset();

    for (int i = 0; i < 100; ++i) guard.update(10.0f, 0.01f);  // 1 s
    CHECK_FALSE(guard.backedOff());
    CHECK_NEAR(guard.forceScale(), 1.0f, 1e-6);

    for (int i = 0; i < 120; ++i) guard.update(10.0f, 0.01f);  // 2.2 s total
    CHECK(guard.backedOff());
    CHECK_NEAR(guard.forceScale(), 0.5f, 1e-6);

    for (int i = 0; i < 120; ++i) guard.update(0.0f, 0.01f);
    CHECK_FALSE(guard.backedOff());
}

TEST(loop_monitor_flags_overruns) {
    LoopMonitor monitor;
    monitor.configure(0.010f, 2.5f);
    monitor.reset();
    for (int i = 0; i < 100; ++i) monitor.update(0.010f);
    CHECK_FALSE(monitor.overrun());
    CHECK_EQ(monitor.overrunCount(), 0);

    monitor.update(0.050f);
    CHECK(monitor.overrun());
    CHECK_EQ(monitor.overrunCount(), 1);
    CHECK_NEAR(monitor.worstSeconds(), 0.050f, 1e-6);
}

TEST(safety_supervisor_severity) {
    SafetySupervisor safety;
    safety.reset();
    CHECK_FALSE(safety.any());
    CHECK_FALSE(safety.requiresSafeTorqueOff());

    safety.set(kFaultBatteryLow, true);
    CHECK(safety.any());
    CHECK(safety.requiresWarning());
    CHECK_FALSE(safety.requiresSafeTorqueOff());

    safety.set(kFaultServoBus, true);
    CHECK(safety.requiresSafeTorqueOff());
    CHECK_FALSE(safety.requiresWarning());
    CHECK(std::strcmp(safety.worstFaultName(), "servo_bus") == 0);

    safety.set(kFaultServoBus, false);
    CHECK_FALSE(safety.requiresSafeTorqueOff());
    CHECK(std::strcmp(safety.worstFaultName(), "battery_low") == 0);
}

// =============================================================================
// modes
// =============================================================================

namespace {
ModeMachine bootedMachine() {
    ModeMachine modes;
    modes.reset();
    modes.update(0.01f, false);  // Boot -> SelfTest
    modes.dispatch(SystemEvent::SelfTestPassed);
    return modes;
}
}  // namespace

TEST(modes_boot_sequence) {
    ModeMachine modes;
    modes.reset();
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Boot));
    modes.update(0.01f, false);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::SelfTest));
    CHECK_FALSE(modes.drivesServos());
    modes.dispatch(SystemEvent::SelfTestPassed);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
    CHECK(modes.drivesServos());
}

TEST(modes_self_test_failure_parks_the_hand) {
    ModeMachine modes;
    modes.reset();
    modes.update(0.01f, false);
    modes.dispatch(SystemEvent::SelfTestFailed);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::SafeTorqueOff));
    CHECK_FALSE(modes.drivesServos());
}

TEST(modes_hold_toggles) {
    ModeMachine modes = bootedMachine();
    modes.dispatch(ButtonEvent::DoubleClick);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Hold));
    modes.dispatch(ButtonEvent::DoubleClick);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
}

TEST(modes_idle_after_timeout_and_wakes_on_activity) {
    ModeMachine modes = bootedMachine();
    modes.setIdleTimeout(1.0f);
    for (int i = 0; i < 200; ++i) modes.update(0.01f, false);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Idle));
    modes.update(0.01f, true);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
}

TEST(modes_severe_fault_forces_safe_torque_off_and_returns) {
    ModeMachine modes = bootedMachine();
    modes.setFaults(true, true);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::SafeTorqueOff));
    CHECK_FALSE(modes.drivesServos());
    modes.setFaults(false, false);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
}

TEST(modes_warning_fault_keeps_the_hand_usable) {
    ModeMachine modes = bootedMachine();
    modes.setFaults(true, false);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Fault));
    CHECK(modes.drivesServos());
    CHECK(modes.acceptsUserInput());
    modes.setFaults(false, false);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
}

TEST(modes_user_park_survives_a_fault_clearing) {
    ModeMachine modes = bootedMachine();
    modes.dispatch(ButtonEvent::VeryLongPress);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::SafeTorqueOff));

    // A transient fault appearing and clearing must not silently re-energise
    // servos on a hand the user deliberately switched off.
    modes.setFaults(true, true);
    modes.setFaults(false, false);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::SafeTorqueOff));

    modes.dispatch(ButtonEvent::VeryLongPress);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
}

TEST(modes_cannot_re_energise_into_a_severe_fault) {
    ModeMachine modes = bootedMachine();
    modes.setFaults(true, true);
    modes.dispatch(ButtonEvent::VeryLongPress);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::SafeTorqueOff));
}

TEST(modes_calibration_returns_to_the_previous_state) {
    ModeMachine modes = bootedMachine();
    modes.dispatch(ButtonEvent::LongPress);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Calibrating));
    modes.dispatch(SystemEvent::CalibrationComplete);
    CHECK_EQ(static_cast<int>(modes.mode()), static_cast<int>(ControlMode::Active));
}

// =============================================================================
// button gestures
// =============================================================================

namespace {

/// Drives a gesture recogniser one millisecond at a time, collecting whatever
/// it emits. Returns the first non-None event, or None.
class ButtonRig {
public:
    ButtonRig() {
        ButtonTiming timing;
        timing.debounceMillis = 25;
        timing.doubleClickMillis = 350;
        timing.longPressMillis = 1200;
        timing.veryLongPressMillis = 4000;
        gestures.configure(timing);
        gestures.reset(now);
    }

    /// Advances time with the button in a fixed state, returning the first
    /// event seen. Later events in the same span are recorded in `extra`.
    ButtonEvent hold(bool pressed, uint32_t millisToAdvance) {
        ButtonEvent first = ButtonEvent::None;
        for (uint32_t i = 0; i < millisToAdvance; ++i) {
            const ButtonEvent event = gestures.update(pressed, now++);
            if (event == ButtonEvent::None) continue;
            if (first == ButtonEvent::None) {
                first = event;
            } else {
                extra = event;
            }
        }
        return first;
    }

    ButtonGestures gestures;
    uint32_t now = 1000;
    ButtonEvent extra = ButtonEvent::None;
};

}  // namespace

TEST(button_single_click_after_the_double_click_window) {
    ButtonRig rig;
    CHECK_EQ(static_cast<int>(rig.hold(true, 80)), static_cast<int>(ButtonEvent::None));
    // The click is withheld until the double-click window has passed.
    CHECK_EQ(static_cast<int>(rig.hold(false, 200)), static_cast<int>(ButtonEvent::None));
    CHECK_EQ(static_cast<int>(rig.hold(false, 400)), static_cast<int>(ButtonEvent::Click));
}

TEST(button_double_click) {
    ButtonRig rig;
    rig.hold(true, 80);
    rig.hold(false, 100);
    rig.hold(true, 80);
    CHECK_EQ(static_cast<int>(rig.hold(false, 100)), static_cast<int>(ButtonEvent::DoubleClick));
    // ...and no stray single click follows it.
    CHECK_EQ(static_cast<int>(rig.hold(false, 800)), static_cast<int>(ButtonEvent::None));
}

TEST(button_long_press_fires_while_held_and_suppresses_the_click) {
    ButtonRig rig;
    CHECK_EQ(static_cast<int>(rig.hold(true, 1000)), static_cast<int>(ButtonEvent::None));
    CHECK_EQ(static_cast<int>(rig.hold(true, 400)), static_cast<int>(ButtonEvent::LongPress));
    // Releasing after a hold must not also report a click.
    CHECK_EQ(static_cast<int>(rig.hold(false, 800)), static_cast<int>(ButtonEvent::None));
}

TEST(button_very_long_press_is_reachable_through_the_long_press) {
    // Regression: an early version latched a single "hold consumed" flag on the
    // long press, so continuing to hold could never reach the very-long
    // gesture — and that gesture is the wearer's only way to park the hand.
    ButtonRig rig;
    CHECK_EQ(static_cast<int>(rig.hold(true, 1300)), static_cast<int>(ButtonEvent::LongPress));
    CHECK_EQ(static_cast<int>(rig.hold(true, 3000)), static_cast<int>(ButtonEvent::VeryLongPress));
    CHECK_EQ(static_cast<int>(rig.hold(false, 800)), static_cast<int>(ButtonEvent::None));
}

TEST(button_each_hold_gesture_fires_once_per_press) {
    ButtonRig rig;
    rig.hold(true, 5000);  // through both thresholds
    // Keeping it down for another five seconds must produce nothing more.
    CHECK_EQ(static_cast<int>(rig.hold(true, 5000)), static_cast<int>(ButtonEvent::None));
    rig.hold(false, 500);
    // A fresh press re-arms both.
    CHECK_EQ(static_cast<int>(rig.hold(true, 1300)), static_cast<int>(ButtonEvent::LongPress));
}

TEST(button_debounces_contact_bounce) {
    ButtonRig rig;
    // 5 ms of chatter, well inside the 25 ms debounce, must not register.
    for (int i = 0; i < 6; ++i) rig.hold(i % 2 == 0, 5);
    CHECK_EQ(static_cast<int>(rig.hold(false, 800)), static_cast<int>(ButtonEvent::None));
    CHECK_FALSE(rig.gestures.pressed());
}

TEST(button_survives_the_millis_wrap) {
    ButtonRig rig;
    // Start just before the 32-bit millisecond counter wraps.
    rig.now = 0xFFFFFF00u;
    rig.gestures.reset(rig.now);
    CHECK_EQ(static_cast<int>(rig.hold(true, 1300)), static_cast<int>(ButtonEvent::LongPress));
    CHECK_EQ(static_cast<int>(rig.hold(true, 3000)), static_cast<int>(ButtonEvent::VeryLongPress));
}

// =============================================================================
// calibration sequencing
// =============================================================================

TEST(calibration_runner_walks_the_sequence_and_saves) {
    EmgChannel flexor, extensor;
    flexor.configure(EmgConfig{});
    extensor.configure(EmgConfig{});

    EmgCalibrationRunner runner;
    CalibrationTiming timing;
    timing.settleSeconds = 0.1f;
    timing.restSeconds = 0.5f;
    timing.effortSeconds = 0.5f;
    runner.configure(timing);
    runner.begin(flexor, extensor, true);
    CHECK(runner.running());

    const float dt = 0.001f;
    int guard = 0;
    int sample = 0;
    while (runner.running() && guard++ < 20000) {
        const bool restPhase = runner.step() == EmgCalibrationRunner::Step::CaptureRest ||
                               runner.step() == EmgCalibrationRunner::Step::SettleRest;
        const bool flexPhase = runner.step() == EmgCalibrationRunner::Step::CaptureFlexor;
        const bool extPhase = runner.step() == EmgCalibrationRunner::Step::CaptureExtensor;
        const float sign = (sample++ % 2) ? 1.0f : -1.0f;
        const float flexAmp = restPhase ? 3.0f : (flexPhase ? 700.0f : 3.0f);
        const float extAmp = restPhase ? 3.0f : (extPhase ? 650.0f : 3.0f);
        runner.update(flexor, extensor, 512.0f + sign * flexAmp, 512.0f + sign * extAmp, dt);
    }

    CHECK(runner.finished());
    CHECK(runner.succeeded());
    CHECK(runner.flexorResult().isUsable());
    CHECK(runner.extensorResult().isUsable());
    CHECK_NEAR(runner.progress(), 1.0f, 1e-6);
}

TEST(calibration_runner_fails_on_a_dead_flexor_channel) {
    EmgChannel flexor, extensor;
    flexor.configure(EmgConfig{});
    extensor.configure(EmgConfig{});

    EmgCalibrationRunner runner;
    CalibrationTiming timing;
    timing.settleSeconds = 0.05f;
    timing.restSeconds = 0.3f;
    timing.effortSeconds = 0.3f;
    runner.configure(timing);
    runner.begin(flexor, extensor, true);

    int guard = 0;
    int sample = 0;
    while (runner.running() && guard++ < 20000) {
        const float sign = (sample++ % 2) ? 1.0f : -1.0f;
        runner.update(flexor, extensor, 512.0f + sign * 3.0f, 512.0f + sign * 3.0f, 0.001f);
    }
    CHECK(runner.finished());
    CHECK_FALSE(runner.succeeded());
}

TEST(digit_calibration_runner_builds_guarded_endpoints) {
    DigitCalibrationRunner runner;
    DigitCalibration base = makeCal(1200, 1800, 700, 2300);
    runner.begin(Digit::Index, base);
    CHECK(runner.active());

    runner.setPulseUs(1050);
    runner.markOpen();
    runner.setPulseUs(2150);
    runner.markClosed();

    DigitCalibration result;
    CHECK(runner.result(result));
    CHECK_EQ(result.openUs, 1050);
    CHECK_EQ(result.closedUs, 2150);
    CHECK_EQ(result.minUs, 1010);
    CHECK_EQ(result.maxUs, 2190);
    CHECK(result.enabled);
    CHECK(result.isValid());
}

TEST(digit_calibration_runner_needs_both_endpoints) {
    DigitCalibrationRunner runner;
    runner.begin(Digit::Index, makeCal(1200, 1800));
    runner.markOpen();
    DigitCalibration result;
    CHECK_FALSE(runner.result(result));
}

// =============================================================================
// configuration
// =============================================================================

TEST(config_defaults_are_structurally_valid) {
    const HandConfig right = HandConfig::defaults(HandSide::Right);
    CHECK(right.isValid());
    CHECK(right.digitsCalibrated());
    CHECK_FALSE(right.emgCalibrated());  // an electrode calibration must be earned
    CHECK_EQ(static_cast<int>(right.side), static_cast<int>(HandSide::Right));

    const HandConfig left = HandConfig::defaults(HandSide::Left);
    CHECK(left.isValid());
    // Handedness only flips the initial travel direction guess.
    CHECK(left.digits[index(Digit::Index)].openUs != right.digits[index(Digit::Index)].openUs);
}

TEST(config_wrist_is_optional) {
    const HandConfig config = HandConfig::defaults(HandSide::Right);
    CHECK_FALSE(config.digits[index(Digit::Wrist)].enabled);
    CHECK(config.digitsCalibrated());  // still considered a calibrated hand
}

TEST(config_rejects_a_broken_enabled_digit) {
    HandConfig config = HandConfig::defaults(HandSide::Right);
    config.digits[index(Digit::Index)].closedUs = config.digits[index(Digit::Index)].openUs;
    CHECK_FALSE(config.isValid());
    CHECK_FALSE(config.digitsCalibrated());
}

TEST(config_checksum_detects_change) {
    HandConfig config = HandConfig::defaults(HandSide::Right);
    const uint32_t before = config.checksum();
    config.digitSpeed[index(Digit::Index)] += 0.1f;
    CHECK(config.checksum() != before);
}

TEST(config_crc32_matches_known_vector) {
    // CRC-32/IEEE of "123456789" is 0xCBF43926.
    const char* data = "123456789";
    CHECK_EQ(crc32(data, 9), 0xCBF43926u);
}

// =============================================================================
// controller integration
// =============================================================================

namespace {

EmgCalibration goodEmgCalibration() {
    EmgCalibration cal;
    cal.baselineCounts = 512.0f;
    cal.noiseCounts = 5.0f;
    cal.mvcCounts = 600.0f;
    cal.valid = true;
    return cal;
}

HandConfig calibratedConfig() {
    HandConfig config = HandConfig::defaults(HandSide::Right);
    config.emgFlexor = goodEmgCalibration();
    config.emgExtensor = goodEmgCalibration();
    return config;
}

struct Rig {
    HandController hand;
    HandInputs in;
    int sample = 0;

    void reset(const HandConfig& config) {
        hand.begin(config);
        in = HandInputs{};
        in.batteryVolts = 8.0f;
        in.servoBusOk = true;
        in.emgFlexorCounts = 512.0f;
        in.emgExtensorCounts = 512.0f;
        for (float& counts : in.fsrCounts) counts = 60.0f;
    }

    /// Runs `seconds` of control loop with the given bipolar EMG amplitudes.
    void run(float seconds, float flexAmplitude = 0.0f, float extAmplitude = 0.0f,
             float dt = 0.005f) {
        const int steps = static_cast<int>(seconds / dt);
        for (int i = 0; i < steps; ++i) {
            const float sign = (sample++ % 2) ? 1.0f : -1.0f;
            in.emgFlexorCounts = 512.0f + sign * flexAmplitude;
            in.emgExtensorCounts = 512.0f + sign * extAmplitude;
            hand.update(in, dt);
            in.button = ButtonEvent::None;
        }
    }

    void press(ButtonEvent event, float dt = 0.005f) {
        in.button = event;
        const float sign = (sample++ % 2) ? 1.0f : -1.0f;
        in.emgFlexorCounts = 512.0f + sign * 3.0f;
        in.emgExtensorCounts = 512.0f + sign * 3.0f;
        hand.update(in, dt);
        in.button = ButtonEvent::None;
    }
};

}  // namespace

TEST(controller_boots_through_self_test_into_active) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Active));
    CHECK(rig.hand.outputs().servoRailEnabled);
    CHECK_EQ(rig.hand.telemetry().faults, 0);
}

TEST(controller_without_emg_calibration_warns_but_keeps_working) {
    Rig rig;
    rig.reset(HandConfig::defaults(HandSide::Right));
    rig.run(1.0f, 3.0f, 3.0f);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Fault));
    CHECK(rig.hand.telemetry().faults & kFaultEmgCalibration);
    CHECK(rig.hand.outputs().servoRailEnabled);
    // ...but it must not synthesise motion from an uncalibrated channel.
    rig.run(2.0f, 900.0f, 3.0f);
    CHECK_NEAR(rig.hand.telemetry().aperture, 0.0f, 1e-4);
}

TEST(controller_closes_on_flexion_and_opens_on_extension) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);

    rig.run(1.0f, 600.0f, 3.0f);
    const float closed = rig.hand.telemetry().aperture;
    CHECK_GT(closed, 0.5f);
    CHECK_GT(rig.hand.telemetry().digits[index(Digit::Index)].closure, 0.4f);
    CHECK_GT(rig.hand.telemetry().digits[index(Digit::Index)].pulseUs, 1200);

    rig.run(1.0f, 3.0f, 600.0f);
    CHECK_LT(rig.hand.telemetry().aperture, closed);
    CHECK_NEAR(rig.hand.telemetry().aperture, 0.0f, 1e-3);
}

TEST(controller_holds_position_with_no_activity) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    rig.run(0.3f, 600.0f, 3.0f);
    CHECK_GT(rig.hand.telemetry().aperture, 0.1f);

    // Let the envelope's release constant run out first — the hand is still
    // legitimately closing for a few hundred milliseconds after the burst
    // stops, which is what makes the motion feel continuous.
    rig.run(2.0f, 3.0f, 3.0f);
    const float held = rig.hand.telemetry().aperture;

    // Once activation is genuinely zero the aperture must not move at all.
    // This is the property HACKberry's unclamped speed mapping broke: inside
    // what reads as a deadband it produced a small negative speed, so a
    // relaxed hand crept open.
    rig.run(5.0f, 3.0f, 3.0f);
    CHECK_NEAR(rig.hand.telemetry().aperture, held, 1e-4);
}

TEST(controller_button_click_cycles_grips) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    const GripId first = rig.hand.grip();
    rig.press(ButtonEvent::Click);
    CHECK(rig.hand.grip() != first);
    CHECK_EQ(static_cast<int>(rig.hand.outputs().haptic), static_cast<int>(HapticCue::Tick));
}

TEST(controller_hold_freezes_the_digits) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    rig.run(0.4f, 600.0f, 3.0f);
    const float aperture = rig.hand.telemetry().aperture;

    rig.press(ButtonEvent::DoubleClick);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Hold));

    // Even with the extensor firing, a held hand does not let go.
    rig.run(2.0f, 3.0f, 600.0f);
    CHECK_NEAR(rig.hand.telemetry().aperture, aperture, 1e-3);
    CHECK(rig.hand.telemetry().digits[index(Digit::Index)].locked);

    rig.press(ButtonEvent::DoubleClick);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Active));
}

TEST(controller_park_removes_every_pulse) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    rig.run(0.4f, 600.0f, 3.0f);
    CHECK_GT(rig.hand.outputs().pulseUs[index(Digit::Index)], 0);

    rig.press(ButtonEvent::VeryLongPress);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode),
             static_cast<int>(ControlMode::SafeTorqueOff));
    CHECK_FALSE(rig.hand.outputs().servoRailEnabled);
    CHECK_FALSE(rig.hand.outputs().servoOutputEnabled);
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        CHECK_EQ(rig.hand.outputs().pulseUs[i], 0);
    }
}

TEST(controller_servo_bus_failure_parks_the_hand) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    rig.in.servoBusOk = false;
    rig.run(0.1f, 3.0f, 3.0f);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode),
             static_cast<int>(ControlMode::SafeTorqueOff));
    CHECK(rig.hand.telemetry().faults & kFaultServoBus);

    rig.in.servoBusOk = true;
    rig.run(0.1f, 3.0f, 3.0f);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Active));
}

TEST(controller_flat_battery_parks_the_hand) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    rig.in.batteryVolts = 6.0f;  // 3.0 V/cell
    rig.run(5.0f, 3.0f, 3.0f);
    CHECK(rig.hand.telemetry().faults & kFaultBatteryCritical);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode),
             static_cast<int>(ControlMode::SafeTorqueOff));
}

TEST(controller_grip_force_stops_the_hand_closing) {
    Rig rig;
    HandConfig config = calibratedConfig();
    config.gripForce.targetNewtons = 8.0f;
    config.gripForce.integralGain = 0.4f;
    rig.reset(config);
    rig.run(1.0f, 3.0f, 3.0f);

    // Close hard onto a fingertip load well above the setpoint.
    rig.in.fsrCounts[index(Digit::Index)] = 2600.0f;
    rig.run(3.0f, 600.0f, 3.0f);

    CHECK(rig.hand.telemetry().contact);
    CHECK_LT(rig.hand.telemetry().closureLimit, 1.0f);
    CHECK_LT(rig.hand.telemetry().aperture, 1.0f);
    CHECK_GT(rig.hand.telemetry().peakForce, config.gripForce.targetNewtons);
}

TEST(controller_stuck_electrode_raises_a_sensor_fault) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);

    // Pin the flexor input at the rail, as a disconnected electrode does.
    const int steps = static_cast<int>(3.0f / 0.005f);
    for (int i = 0; i < steps; ++i) {
        rig.in.emgFlexorCounts = 4095.0f;
        rig.in.emgExtensorCounts = 512.0f;
        rig.hand.update(rig.in, 0.005f);
    }
    CHECK(rig.hand.telemetry().faults & kFaultSensorRange);
}

TEST(controller_direct_aperture_command_yields_to_the_wearer) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);

    rig.hand.commandAperture(0.8f);
    rig.run(1.5f, 3.0f, 3.0f);
    CHECK_NEAR(rig.hand.telemetry().aperture, 0.8f, 0.02);

    // A real contraction takes the command back immediately.
    rig.hand.commandAperture(0.0f);
    rig.run(0.5f, 600.0f, 3.0f);
    CHECK_GT(rig.hand.telemetry().aperture, 0.8f);
}

TEST(controller_co_contraction_changes_grip) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    const GripId first = rig.hand.grip();
    rig.run(1.5f, 600.0f, 600.0f);
    CHECK(rig.hand.grip() != first);
}

TEST(controller_calibration_completes_and_marks_config_dirty) {
    Rig rig;
    rig.reset(HandConfig::defaults(HandSide::Right));
    rig.run(1.0f, 3.0f, 3.0f);
    CHECK_FALSE(rig.hand.config().emgCalibrated());

    rig.press(ButtonEvent::LongPress);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode),
             static_cast<int>(ControlMode::Calibrating));
    CHECK(rig.hand.calibrating());

    // Default timing: 1 s settle, 3 s rest, 1 s settle, 3 s effort, 1 s settle,
    // 3 s effort. Drive each phase with the amplitude it expects.
    const float dt = 0.001f;
    int guard = 0;
    int sample = 0;
    while (rig.hand.calibrating() && guard++ < 60000) {
        const auto step = rig.hand.calibrationRunner().step();
        const bool flexPhase = step == EmgCalibrationRunner::Step::CaptureFlexor;
        const bool extPhase = step == EmgCalibrationRunner::Step::CaptureExtensor;
        const float sign = (sample++ % 2) ? 1.0f : -1.0f;
        rig.in.emgFlexorCounts = 512.0f + sign * (flexPhase ? 700.0f : 3.0f);
        rig.in.emgExtensorCounts = 512.0f + sign * (extPhase ? 650.0f : 3.0f);
        rig.hand.update(rig.in, dt);
    }

    CHECK(rig.hand.calibrationRunner().succeeded());
    CHECK(rig.hand.config().emgCalibrated());
    CHECK(rig.hand.consumeConfigDirty());
    CHECK_FALSE(rig.hand.consumeConfigDirty());  // one-shot

    // Calibration returns to whichever state it was entered from — here that
    // was Fault, because the hand had no usable electrode calibration. The
    // fault clears on the next iteration now that it does.
    rig.run(0.2f, 3.0f, 3.0f);
    CHECK_EQ(rig.hand.telemetry().faults & kFaultEmgCalibration, 0);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Active));
}

TEST(controller_relaxes_and_releases_outputs_when_idle) {
    Rig rig;
    HandConfig config = calibratedConfig();
    config.idleTimeoutSeconds = 1.0f;
    config.idleReleaseEnabled = true;
    rig.reset(config);
    rig.run(1.0f, 3.0f, 3.0f);

    rig.run(6.0f, 3.0f, 3.0f);
    CHECK_EQ(static_cast<int>(rig.hand.telemetry().mode), static_cast<int>(ControlMode::Idle));
    // Fully open and settled, so the finger channels stop emitting pulses.
    CHECK_EQ(rig.hand.outputs().pulseUs[index(Digit::Index)], 0);
    CHECK(rig.hand.telemetry().digits[index(Digit::Index)].released);
}

TEST(controller_does_not_release_outputs_while_holding_an_object) {
    Rig rig;
    HandConfig config = calibratedConfig();
    config.idleTimeoutSeconds = 1.0f;
    rig.reset(config);
    rig.run(1.0f, 3.0f, 3.0f);
    rig.run(1.0f, 600.0f, 3.0f);

    // Close on something and then relax the muscles entirely.
    rig.in.fsrCounts[index(Digit::Index)] = 1200.0f;
    rig.run(6.0f, 3.0f, 3.0f);
    CHECK_GT(rig.hand.outputs().pulseUs[index(Digit::Index)], 0);
    CHECK_FALSE(rig.hand.telemetry().digits[index(Digit::Index)].released);
}

TEST(controller_telemetry_is_populated) {
    Rig rig;
    rig.reset(calibratedConfig());
    rig.run(1.0f, 3.0f, 3.0f);
    const HandTelemetry& t = rig.hand.telemetry();
    CHECK(t.gripName != nullptr);
    CHECK(std::strlen(t.gripName) > 0);
    CHECK_NEAR(t.batteryVolts, 8.0f, 0.05);
    CHECK_NEAR(t.batteryCellVolts, 4.0f, 0.05);
    CHECK_GT(t.stateOfCharge, 0.7f);
    CHECK_GT(t.loopCount, 100u);
    CHECK_NEAR(t.forceTarget, rig.hand.config().gripForce.targetNewtons *
                                  rig.hand.config().gripForceScale[static_cast<uint8_t>(t.grip)],
               0.01);
}

// =============================================================================

int main() {
    std::printf("Jensen Hand — control core tests\n\n");
    return ::jtest::Registry::instance().run();
}
