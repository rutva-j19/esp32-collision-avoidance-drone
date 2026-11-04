#ifndef OBSTACLE_CONTROLLER_H
#define OBSTACLE_CONTROLLER_H
#include "config.h"
#include "distance_filter.h"

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

struct ObstacleFlags {
    bool front = false;
    bool left = false;
    bool right = false;
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
        obstacles = {};
        state = ControllerState::Armed;
        update(readings, nowMs);
        return state == ControllerState::Armed;
    }

    void disarm() {
        commands = {};
        obstacles = {};
        // Disarming stops outputs but does not acknowledge an existing fault.
        if (state != ControllerState::Fault) {
            state = ControllerState::Disarmed;
        }
    }

    void reportFault() {
        state = ControllerState::Fault;
        commands = {};
        obstacles = {};
    }

    bool clearFault(const SensorReadings& readings, uint32_t nowMs) {
        if (state != ControllerState::Fault ||
            readings.front.rawDistanceMm < Config::criticalDistanceMm ||
            readings.left.rawDistanceMm < Config::criticalDistanceMm ||
            readings.right.rawDistanceMm < Config::criticalDistanceMm ||
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

        obstacles.front = updateObstacle(obstacles.front, readings.front);
        obstacles.left = updateObstacle(obstacles.left, readings.left);
        obstacles.right = updateObstacle(obstacles.right, readings.right);

        // Bench interlock: a valid critical raw sample bypasses the median.
        // It latches a fault, so recovery requires clear + arm, not auto-restart.
        if (readings.front.rawDistanceMm < Config::criticalDistanceMm ||
            readings.left.rawDistanceMm < Config::criticalDistanceMm ||
            readings.right.rawDistanceMm < Config::criticalDistanceMm) {
            reportFault();
            return;
        }

        commands = {Config::baseMotorPwm, Config::baseMotorPwm,
                    Config::baseMotorPwm, Config::baseMotorPwm};

        // Demonstrate each obstacle flag using the existing motor-pair response.
        // This is NOT a flight mixer or a steering/navigation algorithm.
        if (obstacles.front) {
            commands.frontLeft -= Config::obstaclePwmReduction;
            commands.frontRight -= Config::obstaclePwmReduction;
        }
        if (obstacles.left) {
            commands.frontLeft -= Config::obstaclePwmReduction;
            commands.backLeft -= Config::obstaclePwmReduction;
        }
        if (obstacles.right) {
            commands.frontRight -= Config::obstaclePwmReduction;
            commands.backRight -= Config::obstaclePwmReduction;
        }
        commands.frontLeft = limitPwm(commands.frontLeft);
        commands.frontRight = limitPwm(commands.frontRight);
        commands.backLeft = limitPwm(commands.backLeft);
        commands.backRight = limitPwm(commands.backRight);
    }

    const ObstacleFlags& getObstacles() const { return obstacles; }

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
    ObstacleFlags obstacles;

    bool updateObstacle(bool blocked, const DistanceReading& reading) const {
        if (reading.distanceMm < Config::obstacleDistanceMm) return true;
        if (reading.distanceMm > Config::obstacleClearanceMm) return false;
        return blocked;
    }

    bool readingIsUsable(const DistanceReading& reading, uint32_t nowMs) const {
        // Unsigned subtraction also works when the millisecond counter wraps.
        return reading.valid &&
               reading.rawDistanceMm >= Config::minimumDistanceMm &&
               reading.rawDistanceMm <= Config::maximumDistanceMm &&
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

#endif // OBSTACLE_CONTROLLER_H
