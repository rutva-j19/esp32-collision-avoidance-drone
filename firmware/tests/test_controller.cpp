#include <cstdlib>
#include <iostream>
#include "controller.h"

// A small test program for the decision-making code. No ESP32 is needed.
// Each test starts with a new controller so tests do not affect each other.
void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

SensorReadings makeReadings(int frontMm, int leftMm, int rightMm,
                           uint32_t timeMs = 0) {
    SensorReadings readings;
    readings.front = {static_cast<uint16_t>(frontMm), timeMs, true, static_cast<uint16_t>(frontMm)};
    readings.left = {static_cast<uint16_t>(leftMm), timeMs, true, static_cast<uint16_t>(leftMm)};
    readings.right = {static_cast<uint16_t>(rightMm), timeMs, true, static_cast<uint16_t>(rightMm)};
    return readings;
}

void checkMotors(const ObstacleController& controller,
                 int frontLeft, int frontRight, int backLeft, int backRight) {
    const auto& motors = controller.getMotorCommands();
    check(motors.frontLeft == frontLeft, "front-left PWM");
    check(motors.frontRight == frontRight, "front-right PWM");
    check(motors.backLeft == backLeft, "back-left PWM");
    check(motors.backRight == backRight, "back-right PWM");
}

void testStartsDisarmed() {
    ObstacleController controller;
    controller.update(makeReadings(2000, 2000, 2000), 0);
    check(controller.getState() == ControllerState::Disarmed, "starts disarmed");
    checkMotors(controller, 0, 0, 0, 0);
}

void testArmingAndDisarming() {
    ObstacleController controller;
    SensorReadings missingReadings;
    check(!controller.arm(missingReadings, 0), "missing readings reject arming");
    check(controller.arm(makeReadings(2000, 2000, 2000), 0), "clear readings allow arming");
    checkMotors(controller, 100, 100, 100, 100);
    controller.disarm();
    check(controller.getState() == ControllerState::Disarmed, "disarm changes state");
    checkMotors(controller, 0, 0, 0, 0);
}

void testFrontObstacle() {
    ObstacleController controller;
    controller.arm(makeReadings(2000, 2000, 2000), 0);
    controller.update(makeReadings(300, 2000, 2000, 50), 50);
    checkMotors(controller, 50, 50, 100, 100);
    // Moving the obstacle away restores normal commands in this basic version.
    controller.update(makeReadings(2000, 2000, 2000, 100), 100);
    checkMotors(controller, 100, 100, 100, 100);
}

void testSideObstacles() {
    ObstacleController controller;
    controller.arm(makeReadings(2000, 2000, 2000), 0);
    controller.update(makeReadings(2000, 300, 2000), 0);
    checkMotors(controller, 50, 100, 50, 100);
    controller.update(makeReadings(2000, 2000, 300), 0);
    checkMotors(controller, 100, 50, 100, 50);
}

void testMultipleObstaclesAndLimits() {
    ObstacleController controller;
    controller.arm(makeReadings(300, 300, 300), 0);
    checkMotors(controller, 0, 0, 50, 50);
    const auto& motors = controller.getMotorCommands();
    for (int pwm : {motors.frontLeft, motors.frontRight, motors.backLeft, motors.backRight}) {
        check(pwm >= 0 && pwm <= Config::maximumMotorPwm, "PWM within limits");
    }
}

void testThresholdBoundary() {
    ObstacleController controller;
    controller.arm(makeReadings(500, 2000, 2000), 0);
    checkMotors(controller, 100, 100, 100, 100); // Trigger is below 500, not <=.
    controller.update(makeReadings(499, 2000, 2000), 0);
    checkMotors(controller, 50, 50, 100, 100);
}

void testStaleReadingsStopMotors() {
    ObstacleController controller;
    auto readings = makeReadings(2000, 2000, 2000, 1000);
    controller.arm(readings, 1000);
    controller.update(readings, 1200);
    check(controller.getState() == ControllerState::Armed, "200 ms old is accepted");
    controller.update(readings, 1201);
    check(controller.getState() == ControllerState::Fault, "201 ms old causes fault");
    checkMotors(controller, 0, 0, 0, 0);
}

void testInvalidReadingAndFaultRecovery() {
    ObstacleController controller;
    auto readings = makeReadings(2000, 2000, 2000);
    controller.arm(readings, 0);
    readings.left.valid = false;
    controller.update(readings, 0);
    check(controller.getState() == ControllerState::Fault, "invalid reading causes fault");
    checkMotors(controller, 0, 0, 0, 0);
    check(!controller.clearFault(readings, 0), "cannot clear with bad readings");

    readings = makeReadings(2000, 2000, 2000, 50);
    controller.update(readings, 50);
    check(controller.getState() == ControllerState::Fault, "fault remains latched");
    check(!controller.arm(readings, 50), "cannot arm directly from fault");
    check(controller.clearFault(readings, 50), "healthy readings permit fault clear");
    check(controller.getState() == ControllerState::Disarmed, "clearing does not rearm");
    checkMotors(controller, 0, 0, 0, 0);
    check(controller.arm(readings, 50), "separate arm command succeeds");
}


// Feed fresh raw samples through the production filter, then into the controller.
struct TestBench {
    DistanceFilter front, left, right;
    ObstacleController controller;
    SensorReadings readings;
    uint32_t timeMs = 0;

    void sample(uint16_t frontMm, uint16_t leftMm = 1000, uint16_t rightMm = 1000) {
        readings.front = front.add(frontMm, true, timeMs);
        readings.left = left.add(leftMm, true, timeMs);
        readings.right = right.add(rightMm, true, timeMs);
        controller.update(readings, timeMs);
        timeMs += 50;
    }

    void start() {
        sample(1000); sample(1000); sample(1000);
        check(controller.arm(readings, timeMs - 50), "bench arms with fresh data");
    }
};

void testFilterStartupAndMedian() {
    DistanceFilter filter;
    check(filter.add(900, true, 0).distanceMm == 900, "first sample passes through");
    check(filter.add(700, true, 50).distanceMm == 700, "two samples use nearer value");
    check(filter.add(800, true, 100).distanceMm == 800, "third sample gives median");
    check(filter.add(600, true, 150).distanceMm == 700, "oldest sample is replaced");
    // Every ordering should produce the same median.
    const uint16_t orders[6][3] = {
        {300, 600, 900}, {300, 900, 600}, {600, 300, 900},
        {600, 900, 300}, {900, 300, 600}, {900, 600, 300}
    };
    for (const auto& order : orders) {
        filter.reset();
        filter.add(order[0], true, 0);
        filter.add(order[1], true, 50);
        check(filter.add(order[2], true, 100).distanceMm == 600, "median ordering");
    }
    filter.reset();
    filter.add(700, true, 0); filter.add(700, true, 50);
    check(filter.add(300, true, 100).distanceMm == 700, "duplicate values");
}

void testIsolatedSpikes() {
    TestBench bench;
    bench.start();
    bench.sample(300);
    check(bench.readings.front.rawDistanceMm == 300, "raw spike retained");
    check(bench.readings.front.distanceMm == 1000, "low spike filtered");
    check(!bench.controller.getObstacles().front, "isolated noncritical spike ignored");
    bench.sample(1000);
    checkMotors(bench.controller, 100, 100, 100, 100);

    bench.sample(300); bench.sample(300); bench.sample(300);
    check(bench.controller.getObstacles().front, "sustained close readings detected");
    bench.sample(2000);
    check(bench.controller.getObstacles().front, "isolated high spike does not clear");
}

void testSustainedObstacleAndClearance() {
    TestBench bench;
    bench.start();
    bench.sample(300);
    check(!bench.controller.getObstacles().front, "one low sample waits for median");
    bench.sample(300);
    check(bench.controller.getObstacles().front, "second low sample detects obstacle");
    checkMotors(bench.controller, 50, 50, 100, 100);
    bench.sample(800);
    check(bench.controller.getObstacles().front, "one clear sample does not release");
    bench.sample(800);
    check(!bench.controller.getObstacles().front, "second clear sample releases");
    checkMotors(bench.controller, 100, 100, 100, 100);
}

void testHysteresisBoundaries() {
    ObstacleController controller;
    controller.arm(makeReadings(500, 1000, 1000), 0);
    check(!controller.getObstacles().front, "exact detection boundary stays clear");
    controller.update(makeReadings(499, 1000, 1000), 0);
    check(controller.getObstacles().front, "below detection boundary blocks");
    for (int distance : {500, 503, 497, 506, 600, 650}) {
        controller.update(makeReadings(distance, 1000, 1000), 0);
        check(controller.getObstacles().front, "deadband holds blocked condition");
    }
    controller.update(makeReadings(651, 1000, 1000), 0);
    check(!controller.getObstacles().front, "above clearance boundary clears");
    controller.update(makeReadings(550, 1000, 1000), 0);
    check(!controller.getObstacles().front, "deadband also holds clear condition");
    controller.disarm();
    controller.arm(makeReadings(550, 1000, 1000), 0);
    check(!controller.getObstacles().front, "new arm starts with clear latch");
}

void testNoisySequenceThroughFilter() {
    TestBench bench;
    bench.start();
    bench.sample(490); bench.sample(490);
    check(bench.controller.getObstacles().front, "sequence enters blocked state");
    for (int distance : {503, 497, 506, 498, 510, 640, 645, 650}) {
        bench.sample(distance);
        check(bench.controller.getObstacles().front, "filtered threshold noise stays blocked");
    }
    bench.sample(700); bench.sample(700);
    check(!bench.controller.getObstacles().front, "stable clearance releases");
}

void testIndependentDirections() {
    TestBench bench;
    bench.start();
    bench.sample(1000, 300, 1000);
    bench.sample(1000, 300, 1000);
    check(!bench.controller.getObstacles().front &&
          bench.controller.getObstacles().left &&
          !bench.controller.getObstacles().right, "left history is independent");
    bench.sample(1000, 600, 300);
    bench.sample(1000, 600, 300);
    check(bench.controller.getObstacles().left &&
          bench.controller.getObstacles().right, "left latch held while right triggers");
    checkMotors(bench.controller, 50, 50, 50, 50);
}

void testInvalidAndStaleFilterHistory() {
    DistanceFilter filter;
    filter.add(1000, true, 0); filter.add(1000, true, 50);
    check(!filter.add(900, false, 100).valid, "status failure invalidates reading");
    check(filter.add(300, true, 150).distanceMm == 300, "invalid data resets history");
    check(!filter.add(39, true, 200).valid, "below sensor range rejected");
    check(!filter.add(4001, true, 250).valid, "above sensor range rejected");
    filter.add(1000, true, 300); filter.add(1000, true, 350);
    check(filter.add(300, true, 551).distanceMm == 300, "stale gap resets history");

    filter.reset();
    filter.add(1000, true, 0); filter.add(1000, true, 50);
    check(filter.add(300, true, 250).distanceMm == 1000, "exact 200 ms gap retains history");
}

void testCriticalRawReading() {
    TestBench bench;
    bench.start();
    bench.sample(149);
    check(bench.readings.front.distanceMm == 1000, "median has not caught critical sample");
    check(bench.controller.getState() == ControllerState::Fault, "raw sample bypasses filter");
    checkMotors(bench.controller, 0, 0, 0, 0);
    check(!bench.controller.clearFault(bench.readings, bench.timeMs - 50),
          "cannot clear while critically close");
    bench.sample(1000);
    check(bench.controller.getState() == ControllerState::Fault, "critical stop stays latched");
    check(bench.controller.clearFault(bench.readings, bench.timeMs - 50), "clear after recovery");
    checkMotors(bench.controller, 0, 0, 0, 0);
    check(bench.controller.arm(bench.readings, bench.timeMs - 50), "explicit rearm");

    ObstacleController controller;
    check(controller.arm(makeReadings(150, 1000, 1000), 0), "exact critical boundary not fault");
    controller.update(makeReadings(1000, 149, 1000), 0);
    check(controller.getState() == ControllerState::Fault, "left critical sample faults");
    ObstacleController rightController;
    check(!rightController.arm(makeReadings(1000, 1000, 149), 0), "right critical sample refuses arm");
}

void testPollingAndClockWrap() {
    TestBench bench;
    bench.start();
    bench.sample(300);
    // Main loop can update many times without a new sensor measurement.
    for (int i = 0; i < 10; ++i)
        bench.controller.update(bench.readings, bench.timeMs - 50);
    check(!bench.controller.getObstacles().front, "repeat updates do not fill filter");

    DistanceFilter filter;
    filter.add(1000, true, UINT32_MAX - 80);
    filter.add(1000, true, UINT32_MAX - 30);
    const auto wrapped = filter.add(300, true, 19);
    check(wrapped.distanceMm == 1000, "wrap does not discard recent history");
    auto readings = makeReadings(1000, 1000, 1000, UINT32_MAX - 30);
    ObstacleController controller;
    check(controller.arm(readings, 19), "freshness calculation survives clock wrap");
    controller.update(readings, 170);
    check(controller.getState() == ControllerState::Fault, "stale across wrap faults");
}

int main() {
    std::cout << "Testing settings: base PWM 100, reduction 50, threshold 500 mm\n";
    testStartsDisarmed();                  std::cout << "PASS: starts disarmed\n";
    testArmingAndDisarming();              std::cout << "PASS: arming and disarming\n";
    testFrontObstacle();                   std::cout << "PASS: front obstacle and clear path\n";
    testSideObstacles();                   std::cout << "PASS: left and right obstacles\n";
    testMultipleObstaclesAndLimits();      std::cout << "PASS: multiple obstacles and PWM limits\n";
    testThresholdBoundary();              std::cout << "PASS: threshold boundary\n";
    testStaleReadingsStopMotors();          std::cout << "PASS: stale readings stop motors\n";
    testInvalidReadingAndFaultRecovery();  std::cout << "PASS: invalid reading and fault recovery\n";
    testFilterStartupAndMedian();         std::cout << "PASS: filter startup and ordering\n";
    testIsolatedSpikes();                 std::cout << "PASS: isolated low and high spikes\n";
    testSustainedObstacleAndClearance();   std::cout << "PASS: sustained detection and clearance\n";
    testHysteresisBoundaries();           std::cout << "PASS: hysteresis boundaries\n";
    testNoisySequenceThroughFilter();     std::cout << "PASS: noisy sequence through filter\n";
    testIndependentDirections();          std::cout << "PASS: independent directions\n";
    testInvalidAndStaleFilterHistory();   std::cout << "PASS: invalid and stale filter history\n";
    testCriticalRawReading();             std::cout << "PASS: critical raw interlock\n";
    testPollingAndClockWrap();            std::cout << "PASS: repeated polling and clock wrap\n";
    std::cout << "All 17 test groups passed.\n";
}
