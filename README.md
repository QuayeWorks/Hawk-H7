QuayeWorks Hawk-H7 Multicopter Flight Controller & AI Vision Board (Full Release 6-30-25)

![image](https://github.com/user-attachments/assets/4ce48747-8ada-4eb6-930e-fb27f005508e)
![image](https://github.com/user-attachments/assets/e76cfe3a-583a-495e-aa54-c03995705b6b)
![image](https://github.com/user-attachments/assets/37a7f0a2-3996-4085-a9fd-f79346cb6480)

**Unlock Next-Level Drone Performance**

Take your drone projects from standard to exceptional with the QuayeWorks Hawk-H7. Specially engineered for quadcopters to octocopters, this advanced flight controller board delivers powerful stabilization, robust sensor integration, and built-in machine learning capabilities. Whether you’re designing for FPV racing, aerial photography, research, or autonomous flight, the Hawk-H7 is your versatile solution.

**Key Features & Benefits:**

  **Advanced Flight Control:**
  
  - ARM Cortex®-M7 STM32H743 MCU (400 MHz) for unmatched real-time performance.
    
  - Supports precise stabilization and autonomous flight functions.

  **Built-in AI & Machine Learning:**
  
  - On-board processing power for real-time object detection, recognition, and tracking (humans, vehicles, landmarks, etc.).

  **Comprehensive FPV System:**
  
  - Integrated AT7456E On-Screen Display (OSD) with dedicated CN11/CN12 video input/output for telemetry overlays.
    
  - Seamless connection to external video transmitters.

  **Wireless Flexibility:**
  
  - Bluetooth (HC-05) enables remote tuning, camera/gimbal control, real-time telemetry, and wireless system configuration.

  **Robust Sensor Suite:**
  
  - MPU6050 IMU for accurate flight stabilization.
    
  - Memory-based magnetometer derived from the IMU for orientation.
    
  - INA219 voltage/current sensor ensures optimal power management.
    
  - GPS port for location tracking, waypoint navigation, and return-to-home.
    
  - Sonar port enabling collision avoidance, terrain-following, and safe landings.

  **Smart Power Management:**
  
  - Efficient onboard buck converters and MOSFET power switching ensure battery efficiency and prolonged flight times.
    
  - USB power and integrated ST-Link debugger simplify development and testing.
    
  **Expandable & Versatile:**
  
  - Plenty of GPIO, SPI, UART, and I2C breakout headers for custom sensors, additional modules, and peripheral devices.

  **Technical Specifications:**
  
  - MCU: STM32H743 (ARM Cortex®-M7 at 400 MHz)
    
  - Debugger: STM32F103 integrated ST-Link
    
  - Wireless: HC-05 Bluetooth Module
    
  - Video OSD: AT7456E with dedicated Video In (CN11) and Video Out (CN12)
    
  - Sensors: MPU6050 (6-axis IMU), memory magnetometer, INA219 (Voltage/Current Monitor)
    
  - Additional Interfaces: MicroSD Card, GPS connector, Sonar connector, USB-to-Serial (CH340C)
    
  - Power Supply: Integrated DC-DC converters (3.3V, 5V), USB power option, MOSFET-controlled power rails

  **Ideal Applications:**
  
  - FPV Drone Racing & Cinematic Aerial Videography
    
  - Research and Educational Platforms
    
  - Autonomous Navigation & Surveillance Drones
    
  - Object Detection & AI-driven Robotics
    
  - Advanced DIY Drone Enthusiasts

## GPS Setup

The UART used for GPS (USART6) defaults to **57600 bps**. If your GPS module
operates at a different rate—such as the NEO‑6M running at 9600 bps—edit
`settings.ini` and change the `gps_baud` value accordingly. The firmware
initializes the UART using this setting during startup, so be sure it matches
your module's configuration.

Elevate your drone’s capabilities with the QuayeWorks Hawk-H7—where advanced flight control meets cutting-edge artificial intelligence.

Disclaimer: The QuayeWorks Hawk-H7 hardware and software are intended solely for responsible and legal use. By using these materials, you agree that QuayeWorks (or the creator) shall not be held liable for any misuse, harm, or illegal activities involving this product. Users assume full responsibility for their actions and agree to comply with applicable laws and regulations.

MIT License

Copyright (c) 2025 QuayeWorks

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
