#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "controller.h"

// Motor PWM setup and output stay here alongside the hardware loop.
class MotorOutputs {
public:
    bool begin() {
        for (unsigned channel = 0; channel < 4; ++channel) {
            digitalWrite(pins[channel], LOW);
            pinMode(pins[channel], OUTPUT);
        }
        // Use the channel-based LEDC API from the pinned Arduino-ESP32 2.x core.
        for (unsigned channel = 0; channel < 4; ++channel) {
            if (ledcSetup(channel, Config::pwmFrequencyHz,
                          Config::pwmResolutionBits) == 0) {
                stop();
                return false;
            }
            ledcAttachPin(pins[channel], channel);
            channelsReady = channel + 1;
            ledcWrite(channel, 0);
        }
        return true;
    }

    void apply(const MotorCommands& commands) {
        if (!Config::enableMotorOutput || channelsReady != 4) {
            stop();
            return;
        }
        ledcWrite(0, limitPwm(commands.frontLeft));
        ledcWrite(1, limitPwm(commands.frontRight));
        ledcWrite(2, limitPwm(commands.backLeft));
        ledcWrite(3, limitPwm(commands.backRight));
    }

    void stop() {
        for (unsigned channel = 0; channel < channelsReady; ++channel) {
            ledcWrite(channel, 0);
        }
    }

private:
    const int pins[4] = {Config::frontLeftMotorPin, Config::frontRightMotorPin,
                         Config::backLeftMotorPin, Config::backRightMotorPin};
    unsigned channelsReady = 0;
    int limitPwm(int pwm) const {
        if (pwm < 0) return 0;
        if (pwm > Config::maximumMotorPwm) return Config::maximumMotorPwm;
        return pwm;
    }
};

DistanceSensors sensors;
MotorOutputs motors;
ObstacleController controller;
bool hardwareReady = false;
uint32_t lastSensorPollMs = 0;
uint32_t lastStatusMs = 0;

const char* stateName() {
    switch (controller.getState()) {
        case ControllerState::Disarmed: return "DISARMED";
        case ControllerState::Armed: return "ARMED";
        case ControllerState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

void printHelp() {
    Serial.println("Commands: a=arm, d=disarm, r=clear fault, s=status, h=help");
    Serial.println("Fault clearing returns to DISARMED; it never starts motors.");
}

void printStatus() {
    const auto& readings = sensors.getReadings();
    const auto& commands = controller.getMotorCommands();
    Serial.printf("%s | filtered mm F:%u L:%u R:%u | valid F:%d L:%d R:%d",
        stateName(), readings.front.distanceMm, readings.left.distanceMm,
        readings.right.distanceMm, readings.front.valid, readings.left.valid,
        readings.right.valid);
    const auto& obstacles = controller.getObstacles();
    Serial.printf(" | raw mm F:%u L:%u R:%u | blocked F:%d L:%d R:%d",
        readings.front.rawDistanceMm, readings.left.rawDistanceMm,
        readings.right.rawDistanceMm, obstacles.front, obstacles.left, obstacles.right);
    Serial.printf(" | fresh+valid:%d | commanded PWM FL:%d FR:%d BL:%d BR:%d",
        controller.readingsAreUsable(readings, millis()), commands.frontLeft,
        commands.frontRight, commands.backLeft, commands.backRight);
    Serial.println(Config::enableMotorOutput ? " | output ENABLED" : " | output DISABLED");
}

void handleSerialCommands() {
    // Bound the work so a stream of characters cannot monopolize the loop.
    for (unsigned count = 0; count < 16 && Serial.available(); ++count) {
        const char command = Serial.read();
        switch (command) {
            case 'a':
                if (hardwareReady && controller.arm(sensors.getReadings(), millis()))
                    Serial.println("Armed.");
                else
                    Serial.println("Cannot arm: check initialization, readings and fault state.");
                break;
            case 'd':
                controller.disarm();
                motors.stop();
                Serial.println("Motor commands stopped.");
                break;
            case 'r':
                if (hardwareReady && controller.clearFault(sensors.getReadings(), millis()))
                    Serial.println("Fault cleared. Disarmed.");
                else
                    Serial.println("Fault not cleared. Initialization failure requires reset.");
                break;
            case 's': printStatus(); break;
            case 'h': printHelp(); break;
            case '\r': case '\n': case ' ': break;
            default: Serial.println("Unknown command. Enter h for help."); break;
        }
    }
}

void setup() {
    // Motor pins are initialized before sensor setup or startup messages.
    const bool motorsReady = motors.begin();
    Serial.begin(Config::serialBaudRate);
    Serial.println("\nESP32 obstacle controller - filtered bench firmware");
    const bool sensorsReady = sensors.begin();
    hardwareReady = motorsReady && sensorsReady;
    if (!hardwareReady) {
        controller.reportFault();
        motors.stop();
        Serial.println("Initialization failed. Check wiring and reset.");
    } else {
        Serial.println("Initialized. Waiting for valid readings; motors disarmed.");
    }
    printHelp();
    printStatus();
}

void loop() {
    handleSerialCommands();
    controller.update(sensors.getReadings(), millis());
    motors.apply(controller.getMotorCommands());

    const uint32_t nowMs = millis();
    if (hardwareReady &&
        uint32_t(nowMs - lastSensorPollMs) >= Config::sensorPollIntervalMs) {
        lastSensorPollMs = nowMs;
        sensors.pollNextSensor();
    }
    // Recheck after I2C work: invalid or now-stale data must stop the commands.
    controller.update(sensors.getReadings(), millis());
    motors.apply(controller.getMotorCommands());

    if (uint32_t(millis() - lastStatusMs) >= Config::statusIntervalMs) {
        lastStatusMs = millis();
        printStatus();
    }
    delay(1);
}
