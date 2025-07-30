
## Work in progress!

![solarium Diagram drawio](https://github.com/user-attachments/assets/f8a33487-a0ce-4170-98f9-2d25f3874cc6)


## Smart zone controlled irrigation and weather data system

This project is an automated irrigation/garden control system, built on an ESP32 microcontroller using the ESP-IDF framework and FreeRTOS. It manages a complex array of sensors and actuators to maintain an optimal environment. The system's core features include an intelligent, multi-zone irrigation system with a learning algorithm, a closed-loop solar tracking system for a photovoltaic panel, and extensive power management and real-time monitoring capabilities. Each major component runs as a dedicated RTOS task, ensuring concurrent and responsive operation.

## Core Systems & Features:
- Intelligent Irrigation: A multi-zone system using a state machine, adaptive learning, and safety protocols.
- Closed-Loop Solar Tracking: A dual-axis solar tracker orients a PV panel using photoresistors and servos. It includes logic to power servos only when adjustments are needed.
- Robust Sensor Integration: Manages environmental (SHT4x, BMP280), rotary position (AS5600) and power (INA219) sensors + analog devices (through ADS1115).
- Power Management: Utilizes MOSFETs to control power to different hardware buses, minimizing idle energy consumption.
## Irrigation System Highlights:
- State Machine: Follows a safe sequence: IDLE -> MEASURING -> STARTING -> WATERING -> STOPPING.
- Event-Driven Monitoring Task: A high-priority parallel task provides real-time control during watering:
- Smart Cutoffs: Stops watering based on target water volume (flow meter) or if moisture levels rise too quickly (e.g., rain).
- Safety Interlocks: Triggers an emergency stop for over-pressure, low-flow, empty water tank or over-current conditions.
## Learning Algorithm:
- Logs historical data (water volume vs. moisture change) to accurately predict future watering needs.
- Automatically adjusts water volume based on ambient temperature.
- Emergency Diagnostics: If a critical fault occurs, it runs automated test cycles to diagnose if the issue is system-wide (pump/filter) or a single-zone fault (valve), disabling only the faulty part.

## Work in progress!

![Zdjęcie WhatsApp 2025-05-09 o 20 46 51_e84f5e36](https://github.com/user-attachments/assets/1abb4618-83d8-4f8f-a14d-57da2c8549b5)
