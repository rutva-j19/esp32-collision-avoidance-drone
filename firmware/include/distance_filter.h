#ifndef OBSTACLE_DISTANCE_FILTER_H
#define OBSTACLE_DISTANCE_FILTER_H

#include "config.h"

struct DistanceReading {
    uint16_t distanceMm = 0; // Filtered distance used by normal obstacle logic.
    uint32_t updatedAtMs = 0;
    bool valid = false;
    uint16_t rawDistanceMm = 0; // Latest sample, also used by the bench interlock.
};

// One instance per sensor. No hardware dependencies, allocation or delays.
class DistanceFilter {
public:
    void reset() {
        count = 0;
        next = 0;
        lastSampleMs = 0;
    }

    DistanceReading add(uint16_t rawMm, bool valid, uint32_t nowMs) {
        if (!valid || rawMm < Config::minimumDistanceMm ||
            rawMm > Config::maximumDistanceMm) {
            reset();
            return {0, nowMs, false, rawMm};
        }
        // Don't combine a fresh measurement with history from before an outage.
        if (count != 0 && uint32_t(nowMs - lastSampleMs) > Config::sensorStaleAfterMs)
            reset();

        samples[next] = rawMm;
        next = (next + 1) % Config::medianWindowSize;
        if (count < Config::medianWindowSize) ++count;
        lastSampleMs = nowMs;

        uint16_t filtered = samples[0];
        if (count == 2) {
            // With only two samples, use the nearer one rather than inventing
            // a distance by averaging. Full median behavior begins at sample 3.
            filtered = samples[0] < samples[1] ? samples[0] : samples[1];
        } else if (count == 3) {
            uint16_t a = samples[0], b = samples[1], c = samples[2];
            if (a > b) swap(a, b);
            if (b > c) swap(b, c);
            if (a > b) swap(a, b);
            filtered = b;
        }
        return {filtered, nowMs, true, rawMm};
    }

private:
    uint16_t samples[Config::medianWindowSize] = {};
    unsigned count = 0;
    unsigned next = 0;
    uint32_t lastSampleMs = 0;

    static void swap(uint16_t& a, uint16_t& b) {
        const uint16_t temporary = a;
        a = b;
        b = temporary;
    }
};

#endif // OBSTACLE_DISTANCE_FILTER_H
