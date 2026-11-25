// HAL tests: the real servo_bus.cpp and the real Adafruit PWM driver library
// against a model of the PCA9685. Build with tools/run_hal_tests.py.
#include <cstdio>

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "hal/servo_bus.h"

namespace {

int failures = 0;
int tests = 0;

#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::printf("    %s:%d CHECK(%s)\n", __FILE__, __LINE__, #condition); \
            failed = true;                                                  \
        }                                                                   \
    } while (0)

bool failed = false;

void run(const char* name, void (*body)()) {
    fake::chip() = fake::Pca9685Model{};
    failed = false;
    body();
    ++tests;
    if (failed) ++failures;
    std::printf("  %s %s\n", failed ? "FAIL" : "ok  ", name);
}

bool runningConfiguration() {
    const fake::Pca9685Model& chip = fake::chip();
    return !chip.asleep() && chip.autoIncrement() && chip.prescale() == 131;
}

bool everyChannelOff() {
    for (int channel = 0; channel < 16; ++channel) {
        if (!fake::chip().channelFullOff(channel)) return false;
    }
    return true;
}

/// A bus that has booted and is driving channel 0, as in normal use.
void startDriving(jensen::ServoBus& bus) {
    bus.begin();
    bus.setRailEnabled(true);
    bus.setOutputEnabled(true);
    bus.write(0, 1500);
}

// Why configure() calls reset() before setPWMFreq(). If a library update ever
// changes this, the test says so.
void library_set_frequency_alone_leaves_a_fresh_chip_asleep() {
    Adafruit_PWMServoDriver driver(0x40, Wire);
    driver.begin();
    fake::chip().powerCycle();
    driver.setOscillatorFrequency(27000000);
    driver.setPWMFreq(50);
    CHECK(fake::chip().asleep());
}

void begin_configures_the_chip_with_every_output_off() {
    jensen::ServoBus bus;
    CHECK(bus.begin());
    CHECK(bus.healthy());
    CHECK(runningConfiguration());
    CHECK(everyChannelOff());
    CHECK(!bus.resetPending());
    CHECK(!fake::chip().rail());
    CHECK(!fake::chip().outputsEnabled());
}

void a_write_produces_a_pulse() {
    jensen::ServoBus bus;
    startDriving(bus);
    CHECK(fake::chip().channelPulsing(0));
}

void a_healthy_poll_changes_nothing() {
    jensen::ServoBus bus;
    startDriving(bus);
    const uint16_t pulse = fake::chip().channelOffCount(0);
    for (int i = 0; i < 20; ++i) CHECK(bus.poll());
    CHECK(!bus.takeReset());
    CHECK(fake::chip().rail());
    CHECK(fake::chip().channelOffCount(0) == pulse);
}

void a_power_cycled_chip_is_detected_cut_off_and_reconfigured() {
    jensen::ServoBus bus;
    startDriving(bus);
    fake::chip().powerCycle();
    CHECK(fake::chip().asleep());
    const int writesWhileDriving = fake::chip().writesWithTorqueAvailable;

    CHECK(bus.poll());
    CHECK(bus.resetPending());
    CHECK(bus.takeReset());
    CHECK(!bus.takeReset());  // once
    CHECK(!fake::chip().rail());
    CHECK(!fake::chip().outputsEnabled());
    CHECK(!bus.railEnabled());
    // Nothing was written to the chip while it could have moved a servo.
    CHECK(fake::chip().writesWithTorqueAvailable == writesWhileDriving);
    CHECK(runningConfiguration());
    CHECK(everyChannelOff());
}

void a_value_written_before_the_reset_is_sent_again_afterwards() {
    jensen::ServoBus bus;
    startDriving(bus);
    fake::chip().powerCycle();
    bus.poll();
    bus.write(0, 1500);  // same value as before: must not be skipped as cached
    CHECK(fake::chip().channelPulsing(0));
}

void a_bus_outage_without_a_reset_reports_none() {
    jensen::ServoBus bus;
    startDriving(bus);
    fake::chip().present = false;
    CHECK(!bus.poll());
    CHECK(!bus.healthy());
    fake::chip().present = true;
    CHECK(bus.poll());
    CHECK(!bus.takeReset());
    CHECK(runningConfiguration());
    CHECK(fake::chip().channelPulsing(0));  // the chip kept its registers
}

void a_value_lost_during_an_outage_is_sent_again() {
    jensen::ServoBus bus;
    startDriving(bus);
    const uint16_t before = fake::chip().channelOffCount(0);
    fake::chip().present = false;
    bus.write(0, 1800);  // the bus has died but no poll has noticed yet
    CHECK(!bus.poll());
    fake::chip().present = true;
    CHECK(bus.poll());
    bus.write(0, 1800);  // must not be skipped as already sent
    CHECK(fake::chip().channelOffCount(0) != before);
}

void one_corrupted_read_is_not_a_reset() {
    jensen::ServoBus bus;
    startDriving(bus);
    fake::chip().corruptReads = 2;  // one MODE1 + PRE_SCALE pair
    CHECK(bus.poll());
    CHECK(!bus.takeReset());
    CHECK(fake::chip().rail());
}

void a_chip_that_appears_after_a_failed_boot_is_configured() {
    fake::chip().present = false;
    jensen::ServoBus bus;
    CHECK(!bus.begin());
    CHECK(!bus.poll());
    fake::chip().present = true;
    fake::chip().powerCycle();
    CHECK(bus.poll());
    CHECK(runningConfiguration());
    CHECK(everyChannelOff());
    CHECK(!bus.takeReset());  // it was never driving, so nothing was lost
}

void a_chip_that_will_not_take_its_setup_is_never_healthy() {
    fake::chip().ignoresPrescale = true;
    jensen::ServoBus bus;
    CHECK(!bus.begin());
    CHECK(!bus.poll());
    CHECK(!bus.healthy());
    fake::chip().ignoresPrescale = false;
    CHECK(bus.poll());
    CHECK(runningConfiguration());
}

void a_failed_reconfiguration_is_retried_and_reported_once() {
    jensen::ServoBus bus;
    startDriving(bus);
    fake::chip().powerCycle();
    fake::chip().ignoresPrescale = true;
    CHECK(!bus.poll());
    CHECK(bus.takeReset());
    CHECK(!fake::chip().rail());
    CHECK(!bus.poll());
    CHECK(!bus.takeReset());
    fake::chip().ignoresPrescale = false;
    CHECK(bus.poll());
    CHECK(!bus.takeReset());
    CHECK(runningConfiguration());
}

void a_second_reset_is_reported_again() {
    jensen::ServoBus bus;
    startDriving(bus);
    fake::chip().powerCycle();
    bus.poll();
    CHECK(bus.takeReset());
    fake::chip().powerCycle();
    bus.poll();
    CHECK(bus.takeReset());
}

}  // namespace

#define RUN(name) run(#name, name)

int main() {
    std::printf("Jensen Hand — servo bus HAL tests\n\n");
    RUN(library_set_frequency_alone_leaves_a_fresh_chip_asleep);
    RUN(begin_configures_the_chip_with_every_output_off);
    RUN(a_write_produces_a_pulse);
    RUN(a_healthy_poll_changes_nothing);
    RUN(a_power_cycled_chip_is_detected_cut_off_and_reconfigured);
    RUN(a_value_written_before_the_reset_is_sent_again_afterwards);
    RUN(a_bus_outage_without_a_reset_reports_none);
    RUN(a_value_lost_during_an_outage_is_sent_again);
    RUN(one_corrupted_read_is_not_a_reset);
    RUN(a_chip_that_appears_after_a_failed_boot_is_configured);
    RUN(a_chip_that_will_not_take_its_setup_is_never_healthy);
    RUN(a_failed_reconfiguration_is_retried_and_reported_once);
    RUN(a_second_reset_is_reported_again);
    std::printf("\n%d test(s), %d failed\n", tests, failures);
    return failures == 0 ? 0 : 1;
}
