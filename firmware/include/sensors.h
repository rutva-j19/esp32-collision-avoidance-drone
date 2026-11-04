#ifndef OBSTACLE_SENSORS_H
#define OBSTACLE_SENSORS_H
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "config.h"
#include "controller.h"

class DistanceSensors {
public:
    bool begin() {
        readings = {};
        initialized = false;
        nextSensor = 0;
        frontFilter.reset();
        leftFilter.reset();
        rightFilter.reset();
        // All sensors boot at 0x29. Hold all off before enabling them individually.
        holdOff(Config::frontShutdownPin);
        holdOff(Config::leftShutdownPin);
        holdOff(Config::rightShutdownPin);
        delay(10);

        if (!Wire.begin(Config::sdaPin, Config::sclPin)) return false;
        Wire.setClock(Config::i2cFrequencyHz);
        Wire.setTimeOut(Config::i2cTimeoutMs);

        if (!startSensor(frontSensor, Config::frontShutdownPin,
                         Config::frontSensorAddress) ||
            !startSensor(leftSensor, Config::leftShutdownPin,
                         Config::leftSensorAddress) ||
            !startSensor(rightSensor, Config::rightShutdownPin,
                         Config::rightSensorAddress)) {
            // A partial startup is not a usable three-sensor system.
            holdOff(Config::frontShutdownPin);
            holdOff(Config::leftShutdownPin);
            holdOff(Config::rightShutdownPin);
            return false;
        }
        initialized = true;
        return true;
    }

    // Read one direction per scheduled call, limiting I2C work per loop pass.
    // Each sensor is polled every 30 ms with the default 10 ms poll interval.
    void pollNextSensor() {
        if (!initialized) return;
        switch (nextSensor) {
            case 0: readSensor(frontSensor, frontFilter, readings.front); break;
            case 1: readSensor(leftSensor, leftFilter, readings.left); break;
            case 2: readSensor(rightSensor, rightFilter, readings.right); break;
        }
        nextSensor = (nextSensor + 1) % 3;
    }

    const SensorReadings& getReadings() const { return readings; }

private:
    VL53L1X frontSensor;
    VL53L1X leftSensor;
    VL53L1X rightSensor;
    DistanceFilter frontFilter;
    DistanceFilter leftFilter;
    DistanceFilter rightFilter;
    SensorReadings readings;
    bool initialized = false;
    uint8_t nextSensor = 0;

    void holdOff(int pin) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }

    bool startSensor(VL53L1X& sensor, int shutdownPin, uint8_t address) {
        // Release XSHUT instead of driving it HIGH. The selected breakout must
        // have its own pull-up to the correct sensor logic voltage.
        pinMode(shutdownPin, INPUT);
        delay(10);
        sensor.setTimeout(Config::sensorStartupTimeoutMs);
        if (!sensor.init()) return false;

        sensor.setAddress(address);
        if (sensor.last_status != 0) return false;
        Wire.beginTransmission(address);
        if (Wire.endTransmission() != 0) return false;

        if (!sensor.setDistanceMode(VL53L1X::Long) ||
            !sensor.setMeasurementTimingBudget(Config::measurementBudgetUs)) {
            return false;
        }
        sensor.startContinuous(Config::measurementPeriodMs);
        return sensor.last_status == 0;
    }

    void readSensor(VL53L1X& sensor, DistanceFilter& filter, DistanceReading& reading) {
        const bool ready = sensor.dataReady();
        if (sensor.last_status != 0) {
            reading = filter.add(0, false, millis());
            return;
        }
        if (!ready) {
            // Preserve the last timestamp. The controller detects stale data.
            return;
        }
        const uint16_t distance = sensor.read(false);
        const bool timedOut = sensor.timeoutOccurred();
        const bool valid = !timedOut && sensor.last_status == 0 &&
            sensor.ranging_data.range_status == VL53L1X::RangeValid &&
            distance >= Config::minimumDistanceMm &&
            distance <= Config::maximumDistanceMm;
        // Only a newly completed measurement enters the filter.
        reading = filter.add(distance, valid, millis());
    }
};

#endif // OBSTACLE_SENSORS_H
