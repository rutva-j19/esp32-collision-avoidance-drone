#pragma once
#include "config.h"

// One measurement, with its age and validity kept alongside the distance.
// A missing reading must not look like a clear path.
struct DistanceReading {
    uint16_t distanceMm = 0;
    uint32_t updatedAtMs = 0;
    bool valid = false;
};

struct SensorReadings {
    DistanceReading front;
    DistanceReading left;
    DistanceReading right;
};


// Signed values allow reductions before clamping, without unsigned wraparound.
struct MotorCommands {
    int frontLeft = 0;
    int frontRight = 0;
    int backLeft = 0;
    int backRight = 0;
};

enum class ControllerState { Disarmed, Armed, Fault };

// No Arduino calls here: this file makes decisions, not hardware accesses.
class ObstacleController {
public:
    bool arm(const SensorReadings& readings, uint32_t nowMs) {
        if (state != ControllerState::Disarmed ||
            !readingsAreUsable(readings, nowMs)) {
            return false;
        }
        state = ControllerState::Armed;
        update(readings, nowMs);
        return true;
    }

    void disarm() {
        commands = {};
        // Disarming stops outputs but does not acknowledge an existing fault.
        if (state != ControllerState::Fault) {
            state = ControllerState::Disarmed;
        }
    }

    void reportFault() {
        state = ControllerState::Fault;
        commands = {};
    }

    bool clearFault(const SensorReadings& readings, uint32_t nowMs) {
        if (state != ControllerState::Fault ||
            !readingsAreUsable(readings, nowMs)) {
            return false;
        }
        state = ControllerState::Disarmed;
        commands = {};
        return true; // The user must issue a separate arm command.
    }

    void update(const SensorReadings& readings, uint32_t nowMs) {
        commands = {};
        if (state != ControllerState::Armed) {
            return;
        }
        if (!readingsAreUsable(readings, nowMs)) {
            reportFault();
            return;
        }

        commands = {Config::baseMotorPwm, Config::baseMotorPwm,
                    Config::baseMotorPwm, Config::baseMotorPwm};

        // First version: demonstrate which sensor triggered using motor pairs.
        // This is NOT a flight mixer or a steering/navigation algorithm.
        if (readings.front.distanceMm < Config::obstacleDistanceMm) {
            commands.frontLeft -= Config::obstaclePwmReduction;
            commands.frontRight -= Config::obstaclePwmReduction;
        }
        if (readings.left.distanceMm < Config::obstacleDistanceMm) {
            commands.frontLeft -= Config::obstaclePwmReduction;
            commands.backLeft -= Config::obstaclePwmReduction;
        }
        if (readings.right.distanceMm < Config::obstacleDistanceMm) {
            commands.frontRight -= Config::obstaclePwmReduction;
            commands.backRight -= Config::obstaclePwmReduction;
        }
        commands.frontLeft = limitPwm(commands.frontLeft);
        commands.frontRight = limitPwm(commands.frontRight);
        commands.backLeft = limitPwm(commands.backLeft);
        commands.backRight = limitPwm(commands.backRight);
    }

    ControllerState getState() const { return state; }
    const MotorCommands& getMotorCommands() const { return commands; }

    bool readingsAreUsable(const SensorReadings& readings, uint32_t nowMs) const {
        return readingIsUsable(readings.front, nowMs) &&
               readingIsUsable(readings.left, nowMs) &&
               readingIsUsable(readings.right, nowMs);
    }

private:
    ControllerState state = ControllerState::Disarmed;
    MotorCommands commands;

    bool readingIsUsable(const DistanceReading& reading, uint32_t nowMs) const {
        // Unsigned subtraction also works when the millisecond counter wraps.
        return reading.valid &&
               reading.distanceMm >= Config::minimumDistanceMm &&
               reading.distanceMm <= Config::maximumDistanceMm &&
               uint32_t(nowMs - reading.updatedAtMs) <= Config::sensorStaleAfterMs;
    }

    int limitPwm(int pwm) const {
        if (pwm < 0) return 0;
        if (pwm > Config::maximumMotorPwm) return Config::maximumMotorPwm;
        return pwm;
    }
};
