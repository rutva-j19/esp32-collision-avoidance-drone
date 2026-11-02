#pragma once
#include <cstdint>

// These pin assignments are provisional until the schematic is reviewed.
namespace Config {
    constexpr int sdaPin = 21;
    constexpr int sclPin = 22;
    constexpr uint32_t i2cFrequencyHz = 100000;
    constexpr uint16_t i2cTimeoutMs = 20;

    constexpr int frontShutdownPin = 25;
    constexpr int leftShutdownPin = 26;
    constexpr int rightShutdownPin = 27;
    constexpr uint8_t frontSensorAddress = 0x30;
    constexpr uint8_t leftSensorAddress = 0x31;
    constexpr uint8_t rightSensorAddress = 0x32;
    constexpr uint16_t sensorStartupTimeoutMs = 100;
    constexpr uint32_t measurementBudgetUs = 33000;
    constexpr uint32_t measurementPeriodMs = 50;
    constexpr uint32_t sensorPollIntervalMs = 10;
    constexpr uint32_t sensorStaleAfterMs = 200;
    constexpr uint16_t minimumDistanceMm = 40;
    constexpr uint16_t maximumDistanceMm = 4000;

    constexpr int frontLeftMotorPin = 13;
    constexpr int frontRightMotorPin = 14;
    constexpr int backLeftMotorPin = 32;
    constexpr int backRightMotorPin = 33;
    constexpr uint32_t pwmFrequencyHz = 10000;
    constexpr uint8_t pwmResolutionBits = 8;
    constexpr int baseMotorPwm = 100;
    constexpr int maximumMotorPwm = 180;
    constexpr int obstaclePwmReduction = 50;
    constexpr uint16_t obstacleDistanceMm = 500;

    // Leave false until the motor circuit has been reviewed and checked.
    // Commanded PWM is still calculated and printed when output is disabled.
    constexpr bool enableMotorOutput = false;
    constexpr uint32_t serialBaudRate = 115200;
    constexpr uint32_t statusIntervalMs = 500;

    static_assert(baseMotorPwm >= 0 && baseMotorPwm <= maximumMotorPwm,
                  "Base PWM must fit within the motor limit");
    static_assert(maximumMotorPwm <= ((1 << pwmResolutionBits) - 1),
                  "Motor limit must fit the PWM resolution");
    static_assert(obstaclePwmReduction >= 0, "Reduction cannot be negative");
    static_assert(measurementPeriodMs * 1000 >= measurementBudgetUs,
                  "Measurement period must cover the timing budget");
}
