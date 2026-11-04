# ESP32 Obstacle-Aware Drone Controller

An embedded systems project combining real-time distance sensing, obstacle-response logic and motor control for a small drone platform. Built around an ESP32 and three time-of-flight sensors, the project covers C++ firmware, motor-driver electronics and custom PCB design, with bench testing used to evaluate behavior before flight integration.

## System Overview

- **ESP32:** Sensor acquisition, obstacle detection and PWM generation.
- **Three VL53L1X sensors:** Measure clearance ahead, left and right.
- **Shared I²C bus:** Individual XSHUT pins allow each sensor to receive a unique address during startup.
- **Four PWM outputs:** Intended for brushed-motor driver circuits.
- **Hardware design:** System architecture and planned PCB integration for power, sensors and motor drivers.

## Obstacle-Response Logic

Each sensor has an independent three-reading median filter to reject isolated measurement spikes. Only new measurements enter the filter; invalid readings or stale gaps reset its history.

Obstacle detection uses hysteresis to prevent repeated switching near a threshold.

Separate detection and clearance thresholds prevent rapid switching between blocked and clear states. Critically close raw readings bypass the filter to trigger a protective bench stop, while invalid or stale sensor data also triggers a fault. Faults remain latched until explicitly cleared.

## Project Files

| File | Purpose |
|---|---|
| `firmware/src/main.cpp` | Startup, main loop, serial interface and motor PWM |
| `firmware/include/config.h` | Pins, timing, thresholds and PWM settings |
| `firmware/include/sensors.h` | Sensor initialization, I²C addressing and measurement acquisition |
| `firmware/include/distance_filter.h` | Median filtering, reading validity and history management |
| `firmware/include/controller.h` | Obstacle hysteresis, arming, fault handling and PWM decisions |
| `firmware/tests/test_controller.cpp` | Filter and controller tests using simulated readings |
| `firmware/platformio.ini` | Board, framework and library configuration |
| `docs/` | System block diagram and design documents |

## Build

Install PlatformIO in VS Code, open the `firmware` folder and select **Build**.

Alternatively, run from the repository root:

```sh
pio run -d firmware
```

Dependencies are declared in `platformio.ini`. Motor output is disabled by default through `enableMotorOutput` in `config.h`.

## Tests

The filter and controller can be tested on a computer without an ESP32. From the repository root:

```sh
c++ -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/include firmware/tests/test_controller.cpp \
  -o /tmp/test_controller

/tmp/test_controller
```

The host test groups cover arming, PWM limits, isolated spikes, sustained obstacles, hysteresis boundaries, independent sensor histories, stale data, fault recovery and timestamp rollover.
