<!--
  🐾 Pet Feeder - Self-Serving Dog Feeder Project
-->

![Project Badge](https://img.shields.io/badge/status-complete-brightgreen) ![License: MIT](https://img.shields.io/badge/license-MIT-blue)

# 🐾 Self-Serving Pet Feeder

> A CC3200-powered “self-serving” pet feeder that lets your dog dispense its meals while still keeping the owner in control. A simple pedal sensor triggers the dispenser, and each dispense event is accurately timestamped by a battery-backed real-time clock so the owner can review exactly when their pet ate. In the on-device settings menu, the owner can choose “small”, “medium”, or “large” portion sizes and set a daily dispense limit so their pets can’t overindulge. An intuitive OLED display – with joystick navigation – shows current time, remaining dispenses, and live fill-level warnings for both food and water. If supplies run low or the dog helps itself, the system pushes an alert via AWS IoT straight to the owner’s phone.

---

## 📋 Table of Contents

1. [🔍 Description](#-description)
2. [🎨 Design](#-design)
   - [State Machine Diagram](#state-machine-diagram)
   - [System Architecture](#system-architecture)
3. [🔧 Implementation](#-implementation)
   - [Mechanical & Structural Build](#mechanical--structural-build)
   - [Pressure Pedal](#pressure-pedal)
   - [Photo-Resistor Fill-Level Sensors](#photo-resistor-fill-level-sensors)
   - [Water-Level Sensor](#water-level-sensor)
   - [Real-Time Clock (DS1307)](#real-time-clock-ds1307)
   - [Joystick-Driven Menu & OLED Display](#joystick-driven-menu--oled-display)
   - [Servo Control & Portion Sizes](#servo-control--portion-sizes)
   - [Wireless & Cloud Notifications](#wireless--cloud-notifications)
   - [Power & Integration Notes](#power--integration-notes)
4. [⚠️ Challenges](#-challenges)
5. [🚀 Future Work](#-future-work)
6. [📄 License](#-license)

---

## 🔍 Description

A CC3200-powered “self-serving” pet feeder that lets your dog dispense its meals while still keeping the owner in control. A simple pedal sensor triggers the dispenser, and each dispense event is accurately timestamped by a battery-backed real-time clock so the owner can review exactly when their pet ate. In the on-device settings menu, the owner can choose “small”, “medium”, or “large” portion sizes and set a daily dispense limit so their pets can’t overindulge. An intuitive OLED display – with joystick navigation – shows current time, remaining dispenses, and live fill-level warnings for both food and water. If supplies run low or the dog helps itself, the system pushes an alert via AWS IoT straight to the owner’s phone.

---

## 🎨 Design

### System Architecture

- **Pressure Sensor**: Force-sensitive resistor for pedal detection.
- **Photo Resistor**: Light-based fill-level sensors inside the food hopper.
- **Water Level Sensor**: Analog sensor to measure bowl level as a percentage.
- **DS1307 RTC**: I2C-backed real-time clock with coin-cell backup.
- **Joystick Controller**: Y-axis analog input for menu navigation; push-button for selection.
- **SG90 Servo**: 50 Hz PWM control to open/close the trap-door.
- **128×128 OLED**: SPI interface for UI (logs, settings, live status).
- **AWS IoT**: Secure TLS connection to publish telemetry and alerts.
- **Power**: 3.3 V for sensors/MCU; 5 V external supply for servo; common ground.

---

## 🔧 Implementation

<details>
<summary>Mechanical & Structural Build 🛠️</summary>

Built from plywood panels forming a funnel enclosure with a servo-actuated trap-door at the base. The trap-door drops kibble when the servo rotates.

</details>

<details>
<summary>Pressure Pedal 🚪</summary>

A force-sensitive resistor under the pedal reads analog voltage changes via ADC to detect a firm press.

</details>

<details>
<summary>Photo-Resistor Fill-Level Sensors 📏</summary>

Mounted at fixed heights inside the funnel; firmware samples each analog channel each loop. A sudden change flags “food empty” and triggers an alert.

</details>

<details>
<summary>Water-Level Sensor 💧</summary>

Reads ADC value once per second, converts to a percentage (displayed on OLED). Below 10%, sends a low-water notification and resets upon refill.

</details>

<details>
<summary>Real-Time Clock (DS1307) ⏰</summary>

Initializes over I²C; converts BCD registers to decimal fields (MM/DD/YY, HH:MM:SS). Stores last 10 dispense timestamps in RAM, accessible via menu.

</details>

<details>
<summary>Joystick-Driven Menu & OLED Display 🖥️</summary>

Vertical joystick (Y-axis) moves cursor; push-button selects. Displays three options (Feeding History, Settings, Back). Submenus allow pagination and setting adjustments.

</details>

<details>
<summary>Servo Control & Portion Sizes 🎛️</summary>

Bit-banged 50 Hz PWM: ~1.5 ms pulse closes trap-door, ~2.1 ms opens. Pulse duration sequences correspond to small/medium/large scoops.

</details>

<details>
<summary>Wireless & Cloud Notifications ☁️</summary>

Connects to Wi-Fi, establishes TLS to AWS IoT endpoint. Publishes JSON payloads (e.g., “Dispensed Medium portion”). AWS IoT Rule → SNS → Email.

</details>

<details>
<summary>Power & Integration Notes 🔌</summary>

MCU and sensors run on 3.3 V. Servo uses 5 V from external supply. Common ground references. DS1307 coin cell preserves logs through power cycles.

</details>

---

## ⚠️ Challenges

- **ADC Calibration**: Tuned reference voltages and sampling rates to stabilize readings for FSR, water sensor, and joystick.
- **Servo Power**: LaunchPad’s 5 V rail insufficient. Added external 5 V supply; used logic analyzer to diagnose.

---

## 🚀 Future Work

- **RFID Multi-Pet Support**: Authorized dispense by pet. Custom settings per tag.
- **Weight Sensor Integration**: Measure exact kibble dispensed for dietary tracking.
- **Web Dashboard**: Visualize feeding history, consumption metrics, alerts, and analytics.

---

## 📄 License

Distributed under the MIT License. See [LICENSE](LICENSE) for details.
