# Solarium - Solar-Powered Garden Automation System

![solarium Diagram drawio](https://github.com/user-attachments/assets/f8a33487-a0ce-4170-98f9-2d25f3874cc6)

## TOPO / Work in progres...
![solarium_topo](https://github.com/user-attachments/assets/24fc8973-ec19-44c9-9583-1ec91f9fd074)


## Overview

Solarium is an ESP32-S3 based garden automation system that combines intelligent irrigation, weather monitoring, solar tracking, and power management. The system is designed with a modular architecture using ESP-IDF and FreeRTOS, with each major subsystem implemented as separate components that can operate independently or as part of a distributed network.

## System Architecture

The current Alpha implementation runs on a single ESP32-S3 with four main subsystems:

**IMPLUVIUM (Irrigation)**: Multi-zone irrigation controller with learning algorithms that adjust watering based on historical data and temperature readings. Implements safety interlocks including over-pressure, low-flow, and empty tank detection. Features emergency diagnostics to isolate faults between system-wide (pump/filter) and zone-specific (valve) issues.

**FLUCTUS (Power Management & Solar Tracking)**: Manages a 4-bus power distribution system (3.3V, 5V, 6.2V, 12V) with buck converter control and reference counting for multiple consumers. Implements 5-state load shedding (Normal -> Power Saving -> Low Power -> Very Low -> Critical) with inter-component communication. Includes dual-axis solar tracking using photoresistor feedback and automatic parking during low-light conditions.

**TEMPESTA (Weather Station)**: Environmental monitoring using SHT4x (temperature/humidity), BME280 (t/h/pressure), AS5600 (wind speed via rotary encoder), and PMS5003 (air quality). Will feature custom wind measurement with triple-cup anemometer, rainfall detection via tipping bucket HAL sensor, and sensor fusion with historical averaging.

**STELLARIA (Ambient Lighting)**: LED lighting control with constant current driver, PWM intensity control, and automatic light sensing with hysteresis. Integrates with power management for load shedding support.

## Technical Features

### Power Management
- Battery monitoring with voltage-based load shedding
- Reference-counted bus sharing prevents premature shutdowns
- Overcurrent protection (1.5A/3s warning, 3A immediate shutdown)
- Consumer tracking for fault diagnosis

### Irrigation Intelligence  
- State machine: IDLE -> MEASURING -> WATERING -> STOPPING
- Learning algorithm correlates water volume with moisture change
- Temperature-adaptive watering volumes
- Real-time monitoring task with safety interlocks
- Emergency diagnostics with automated test cycles

### Weather Data Processing
- Sensor fusion combining multiple temperature/humidity sources
- Weighted averaging with more recent samples prioritized
- AS5600 wind measurement
- Quarter-hour aligned data collection cycles
- Power-aware sensor operation with progressive load shedding

### Load Shedding Integration
All components implement coordinated load shedding APIs:
- Power save mode (reduced operation frequency)
- Feature disabling (PMS5003 air quality sensor)
- Complete shutdown for critical power states
- Automatic restoration when power levels recover

## Hardware Integration

**Sensors**: SHT4x, BME280, AS5600, PMS5003, multiple ADS1115 ADCs, Honeywell ABP delta pressure sensor, flow sensor with pulse counting

**Actuators**: Water valves, pump control, dual-axis servos, LED drivers, 12V PWM fan

**Power**: Solar panel input, battery bank, multi-stage voltage regulation, INA219 power monitoring

**Communication**: I2C, SPI, UART, GPIO, PWM interfaces

## Development Status

This Alpha stage focuses on single-node operation with integrated power management and a temporary blackbox MPPT. The planned Beta architecture will transition to a distributed multi-node system with dedicated controllers for power management, RS485/ESP-NOW communication, and Linux server integration through Home Assistant or database systems.

Current implementation includes complete integration of all four subsystems with centralized FLUCTUS power management, comprehensive load shedding communication, and thread-safe operation across concurrent FreeRTOS tasks.

## Work in progres...

![Zdjęcie WhatsApp 2025-05-09 o 20 46 51_e84f5e36](https://github.com/user-attachments/assets/1abb4618-83d8-4f8f-a14d-57da2c8549b5)
