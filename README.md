![image](https://github.com/user-attachments/assets/4ce48747-8ada-4eb6-930e-fb27f005508e)
![image](https://github.com/user-attachments/assets/e76cfe3a-583a-495e-aa54-c03995705b6b)
![image](https://github.com/user-attachments/assets/37a7f0a2-3996-4085-a9fd-f79346cb6480)

# Hawk-H7 Flight Controller  
### STM32H743 Multicopter Control Platform

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![MCU](https://img.shields.io/badge/MCU-STM32H743-03234B)
![Core](https://img.shields.io/badge/Core-Cortex--M7-orange)
![Status](https://img.shields.io/badge/Status-Active-green)

---

## Overview

The **Hawk-H7** is a high-performance STM32H743-based multicopter flight controller designed for deterministic real-time control, structured diagnostics, and robust sensor validation.

This project focuses on:

- Deterministic scheduling (no blocking ISR logic)
- Structured sensor health gating
- Real, measurable diagnostics
- Safe disarmed-only hardware testing
- Transparent firmware behavior

The Hawk-H7 supports quadcopters through octocopters and is intended for developers, researchers, and advanced builders who want full visibility and control over their flight stack.

---

# Hardware Overview

## MCU
- **STM32H743**
- ARM Cortex-M7 @ 400 MHz
- Hardware FPU
- High-resolution PWM timers
- Deterministic scheduler loop

## Integrated Debug
- Onboard ST-Link (STM32F103)
- USB programming support
- UART console interface

## Major Interfaces

| Interface | Purpose |
|------------|----------|
| MPU6050 | 6-axis IMU |
| QMC5883L | Magnetometer |
| BMP/BME class | Barometer |
| INA219 (0x40–0x4F) | Voltage/Current monitor |
| USART6 | GPS |
| Sonar port | Altitude assist |
| AT7456E | OSD overlay |
| HC-05 | Bluetooth telemetry |

---

# Firmware Architecture

## Deterministic Scheduler

| Module | Rate |
|--------|------|
| IMU | 500 Hz |
| EKF | 200 Hz |
| Barometer | 50 Hz |
| GPS Parse | 500 Hz |
| GPS Health | 5 Hz |
| RC | 100 Hz |
| Battery | 50 Hz |

### Design Principles

- No NMEA parsing inside UART interrupts
- No SD card writes inside interrupt context
- Bounded HAL timeouts (no infinite blocking)
- Health bits tied to measurable sensor evidence

---

# Sensor Suite

## IMU – MPU6050
- WHO_AM_I identity verification
- Plausibility validation
- Read success/failure counters
- Last HAL status reporting
- Health bit integration

## Barometer
- Chip ID verification
- Pressure sanity gating
- IIR altitude filtering
- Smoothed climb rate
- Primary EKF vertical reference

## Magnetometer – QMC5883L
- Mahalanobis gating
- Stub detection mode
- Structured health reporting

## GPS
- Ring-buffered NMEA parsing
- Sentence rate tracking (Hz)
- Dropped byte detection
- HDOP + satellite gating
- Structured overflow warning:


WARN code=GPS_RX_OVF delta=<n>


## INA219 Power Monitoring
- Scans I2C 0x40–0x4F
- Supports up to 4 nodes
- Sums current draw
- Tracks I2C errors and ADC timeouts

## Sonar
- Trigger + echo edge counters
- Timeout detection
- Recency validation

---

# EKF & Vertical Fusion

- Barometer is primary vertical reference
- GPS altitude fused with lower gain
- Innovation gating applied to baro and GPS
- Vertical velocity bounded for stability
- Health bits reflect real innovation limits

---

# Debug CLI v2

The firmware includes a structured diagnostic command interface.

## System Commands

| Command | Description |
|----------|--------------|
| help | Show command groups |
| status | One-line system state |
| health | Health bit summary |
| rates | Sensor update rates |
| period <ms> | Set stream interval |
| stop | Stop all streams/tests safely |

---

## Sensor Inspection

| Command | Function |
|----------|----------|
| show \<module> | One-shot output |
| show \<module> verbose | Extended counters |
| stream \<module> on/off | Enable/disable stream |
| stream all on/off | Toggle all streams |

Modules:

imu, baro, mag, ina, rc, gps, sonar, osd, sys


---

## Active Tests (Disarmed Only)

| Command | Purpose |
|----------|----------|
| test active once | Active sensor validation |
| test bus once | I2C scan |
| test storage run | SD test |
| test pwm start | Motor/servo sweep |
| test pwm stop | Stop sweep |

While armed:


ERR code=ARMED_DIAGNOSTICS_LOCKED


---

## Output Format

All runtime output follows:


TAG key=value key=value ...


Example:


SYS state=READY armed=NO health_all=YES loop=198Hz
IMU id=0x68 id_ok=YES read_ok=1520 fail=0 hal=OK
GPS fix=3D sats=12 hdop=0.9 sps=5 drop=0 ring=12


---

# Board Revision Compatibility

| Feature | Rev-1.0A | Rev-1.1A |
|----------|----------|----------|
| HC-05 TX | PE8 | PB10 |
| HC-05 RX | PE9 (firmware-disabled) | PB11 |
| Bluetooth CLI RX | ❌ | ✔ |
| Telemetry TX | ✔ | ✔ |

### Legacy Boards

Bluetooth is TX-only telemetry.  
CLI commands cannot be sent over Bluetooth.

---

# RC Input

Primary input:
- **PPM on PF9** (EXTI timing capture)

Fallback:
- PWM1 input mode

For FlySky FS-iA6B:
- Enable receiver PPM mode.

RC health includes:
- Frame freshness
- Stale detection
- Optional RSSI gating

---

# Servo & PWM Test Procedure


servo debug on


Sequence:
1. 2 seconds: PA0 + PA1 sweep forward
2. 2 seconds: PA0 + PA1 sweep reverse
3. Gimbal stabilization:
   - PA2 = Pitch
   - PA3 = Yaw

Disarmed only.

---

# GPS Setup

Default baud rate: **57600**

For NEO-6M (9600):

Edit `settings.ini`:


gps_baud=9600


Firmware reads baud at boot.

---

# Safety Model

- Mutating diagnostics blocked while armed
- CPU timing watchdog
- GPS overflow detection
- Innovation gating for sensors
- Rate-limited error tones
- Structured per-module health bits

---

# Applications

- Research platforms
- Custom autonomy development
- Sensor fusion experimentation
- Advanced DIY multicopter builds

---

# Disclaimer

The Hawk-H7 hardware and firmware are intended for responsible and lawful use.  
Users assume full responsibility for safe operation and regulatory compliance.

---

# License

MIT License  
Copyright (c) 2025 QuayeWorks
