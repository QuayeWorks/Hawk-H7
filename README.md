# Hawk-H7 Flight Controller

STM32H743-based flight controller firmware for multicopter platforms.

![image](https://github.com/user-attachments/assets/4ce48747-8ada-4eb6-930e-fb27f005508e)
![image](https://github.com/user-attachments/assets/e76cfe3a-583a-495e-aa54-c03995705b6b)
![image](https://github.com/user-attachments/assets/37a7f0a2-3996-4085-a9fd-f79346cb6480)

# Hawk-H7 Flight Controller  
### STM32H743 Multicopter Control Platform

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![MCU](https://img.shields.io/badge/MCU-STM32H743-03234B)
![Core](https://img.shields.io/badge/Core-Cortex--M7-orange)
![Status](https://img.shields.io/badge/Status-Active-green)

## Overview

Hawk-H7 is a real-time flight control platform built around the STM32H743. It is designed for deterministic task scheduling, structured diagnostics, sensor validation, and safe hardware testing during development.

The project supports multicopter configurations from quadcopters through octocopters and is intended for developers who want direct visibility into system behavior instead of relying on a closed flight stack.

## Core Capabilities

- Deterministic scheduler-based firmware architecture
- Structured sensor health and identity validation
- Non-blocking interrupt handling for time-critical paths
- Disarmed-only active hardware test functions
- Diagnostic CLI for runtime inspection and validation
- Support for common navigation, power, telemetry, and altitude sensors

## Hardware Platform

### MCU

- STM32H743
- ARM Cortex-M7 at 400 MHz
- Hardware floating point unit
- Timer-based PWM output support

### Debug and Programming

- Onboard ST-Link
- USB programming interface
- UART console interface

### Supported Interfaces

| Interface | Purpose |
|----------|---------|
| MPU6050 | IMU |
| QMC5883L | Magnetometer |
| BMP/BME-class devices | Barometer |
| INA219 | Voltage and current monitoring |
| USART6 | GPS |
| Sonar port | Altitude assist |
| AT7456E | OSD |
| HC-05 | Bluetooth telemetry |

## Firmware Architecture

The firmware uses a fixed-rate scheduler for time-sensitive subsystems and avoids placing slow or blocking work inside interrupt context.

### Task Rates

| Module | Rate |
|--------|------|
| IMU | 500 Hz |
| EKF | 200 Hz |
| Barometer | 50 Hz |
| GPS Parse | 500 Hz |
| GPS Health | 5 Hz |
| RC | 100 Hz |
| Battery | 50 Hz |

### Design Rules

- No GPS parsing inside UART interrupts
- No SD card writes inside interrupt context
- No unbounded blocking timeouts
- Health state is based on measurable sensor behavior

## Sensor and System Monitoring

### IMU

- WHO_AM_I verification
- Read success and failure tracking
- Plausibility checks
- Health state integration

### Barometer

- Chip ID verification
- Pressure sanity checks
- Filtered altitude estimate
- Climb rate smoothing
- EKF vertical reference input

### Magnetometer

- Mahalanobis-based gating
- Structured health reporting
- Stub detection support

### GPS

- Ring-buffered NMEA parsing
- Sentence rate tracking
- Dropped byte detection
- HDOP and satellite-count gating
- Overflow warning reporting

### Power Monitoring

- INA219 scan support on I2C addresses `0x40–0x4F`
- Multi-node current aggregation
- I2C error and ADC timeout tracking

### Sonar

- Trigger and echo event counting
- Timeout detection
- Data recency validation

## State Estimation

The EKF uses barometric altitude as the primary vertical reference and fuses GPS altitude with lower influence. Innovation checks are applied to both sources, and vertical velocity is bounded to improve stability and fault tolerance.

## Diagnostic CLI

Hawk-H7 includes a command-line diagnostic interface for runtime inspection, health monitoring, and controlled hardware testing.

### System Commands

| Command | Description |
|---------|-------------|
| `help` | Show command groups |
| `status` | Show one-line system state |
| `health` | Show health bit summary |
| `rates` | Show sensor update rates |
| `period <ms>` | Set stream interval |
| `stop` | Stop active streams and tests |

### Sensor Inspection

| Command | Description |
|---------|-------------|
| `show <module>` | One-shot output |
| `show <module> verbose` | Extended counters and details |
| `stream <module> on/off` | Toggle module stream |
| `stream all on/off` | Toggle all streams |

Supported modules:

`imu`, `baro`, `mag`, `ina`, `rc`, `gps`, `sonar`, `osd`, `sys`

### Active Tests

These functions are intended for disarmed development and bench validation only.

| Command | Description |
|---------|-------------|
| `test active once` | Run active sensor validation |
| `test bus once` | Run I2C scan |
| `test storage run` | Run SD card test |
| `test pwm start` | Start motor or servo sweep |
| `test pwm stop` | Stop motor or servo sweep |

Mutating diagnostics are blocked while armed.

## Runtime Output Format

Runtime messages follow a structured key-value format:

```text
TAG key=value key=value ...
```

Example:

```text
SYS state=READY armed=NO health_all=YES loop=198Hz
IMU id=0x68 id_ok=YES read_ok=1520 fail=0 hal=OK
GPS fix=3D sats=12 hdop=0.9 sps=5 drop=0 ring=12
```

## Board Revision Notes

| Feature | Rev-1.0A | Rev-1.1A |
|---------|----------|----------|
| HC-05 TX | PE8 | PB10 |
| HC-05 RX | PE9 (disabled in firmware) | PB11 |
| Bluetooth CLI RX | No | Yes |
| Telemetry TX | Yes | Yes |

Legacy boards support Bluetooth telemetry transmit only. CLI input over Bluetooth is not available on those revisions.

## RC Input

Primary input:

- PPM on PF9 using EXTI timing capture

Fallback:

- PWM1 input mode

For FlySky FS-iA6B receivers, enable PPM mode on the receiver.

RC health monitoring includes frame freshness, stale detection, and optional RSSI gating.

## Servo and PWM Test

Servo debug mode provides a basic staged sweep for output validation.

```text
servo debug on
```

Sequence:

1. Two-second forward sweep on PA0 and PA1
2. Two-second reverse sweep on PA0 and PA1
3. Gimbal stabilization output on:
   - PA2: Pitch
   - PA3: Yaw

Available only while disarmed.

## GPS Configuration

Default GPS baud rate is `57600`.

For modules configured at `9600` baud, update `settings.ini`:

```ini
gps_baud=9600
```

The firmware reads this value at boot.

## Safety

The firmware includes several safeguards intended to reduce unsafe test behavior and improve fault visibility during development:

- Diagnostics blocked while armed
- CPU timing watchdog
- GPS overflow detection
- Sensor innovation gating
- Rate-limited error tones
- Per-module health reporting

## Intended Use

Hawk-H7 is intended for:

- Flight control development
- Sensor fusion testing
- Research platforms
- Advanced DIY multicopter builds

## Disclaimer

This hardware and firmware are provided for lawful and responsible use. The user is responsible for safe operation, testing, and regulatory compliance.

## License

MIT License

Copyright (c) 2025 QuayeWorks
