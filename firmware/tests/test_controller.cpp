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
    readings.front = {static_cast<uint16_t>(frontMm), timeMs, true};
    readings.left = {static_cast<uint16_t>(leftMm), timeMs, true};
    readings.right = {static_cast<uint16_t>(rightMm), timeMs, true};
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

int main() {
    std::cout << "Using initial settings: base PWM 100, reduction 50, threshold 500 mm\n";
    testStartsDisarmed();                  std::cout << "PASS: starts disarmed\n";
    testArmingAndDisarming();              std::cout << "PASS: arming and disarming\n";
    testFrontObstacle();                   std::cout << "PASS: front obstacle and clear path\n";
    testSideObstacles();                   std::cout << "PASS: left and right obstacles\n";
    testMultipleObstaclesAndLimits();      std::cout << "PASS: multiple obstacles and PWM limits\n";
    testThresholdBoundary();              std::cout << "PASS: threshold boundary\n";
    testStaleReadingsStopMotors();          std::cout << "PASS: stale readings stop motors\n";
    testInvalidReadingAndFaultRecovery();  std::cout << "PASS: invalid reading and fault recovery\n";
    std::cout << "All 8 basic controller tests passed.\n";
}
