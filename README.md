# ESP32 Obstacle-Aware Drone Controller

An embedded systems project exploring how a small drone can sense nearby obstacles and respond through motor control. It combines an ESP32, three distance sensors and a custom motor-driver PCB, bringing together firmware, circuit design and hardware testing.

## System Overview

- **ESP32:** Reads sensors and runs the controller.
- **Three VL53L1X sensors:** Measure distance ahead, left and right.
- **I²C interface:** Connects the sensors on a shared bus, using XSHUT pins to assign unique addresses.
- **Four PWM outputs:** Control brushed motors through motor-driver circuits.
- **Custom PCB:** Intended to integrate power, sensor connections and motor drivers.

## Obstacle Detection and Navigation

The project uses three directional time-of-flight sensors to support local obstacle detection and avoidance. Its navigation design combines distance filtering, threshold-based obstacle detection and left–right clearance comparison to select an avoidance direction. Hysteresis separates obstacle detection and clearance thresholds to reduce rapid switching near a boundary.


## Project Files

| File | Purpose |
|---|---|
| `firmware/src/main.cpp` | Startup, main loop, serial commands and motor PWM |
| `firmware/include/config.h` | Pins, distance thresholds, timing and PWM settings |
| `firmware/include/sensors.h` | Sensor setup, unique I²C addresses and distance readings |
| `firmware/include/controller.h` | Obstacle response, arming and fault handling |
| `firmware/tests/test_controller.cpp` | Basic controller tests with simulated readings |
| `firmware/platformio.ini` | ESP32 build configuration and dependencies |
| `docs/` | System block diagram and design documents |

## Build

Install the PlatformIO extension in VS Code, open the `firmware` folder and select **Build**.

Alternatively, run from the repository root:

```sh
pio run -d firmware
```

Motor output is disabled by default in `config.h`.

## Tests

The controller tests run on a computer without an ESP32. With a C++ compiler installed, run from the repository root:

```sh
c++ -std=c++17 -Ifirmware/include firmware/tests/test_controller.cpp -o test_controller
./test_controller
```

Tests cover arming and disarming, obstacle responses, PWM limits and sensor faults. Physical sensor readings and motor behavior require separate bench testing.