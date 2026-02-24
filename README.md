# Solarium - Irrigation Focused Smart Garden Automation System

## Status
v0.2-alpha - 01.2025
Dry testing complete. With most of the features tested, currently building hydraulic system (and compact enclosures to match), making field testing/deployment possible in the near future.
<p align="center">
  <img src="https://github.com/user-attachments/assets/89cf4c63-4b47-4390-9b8a-68802c780b96" width="640">
</p>
## Overview

Solarium is a self-contained, renewable-powered garden automation platform running on a single ESP32-S3. The system is built around four main subsystems: **FLUCTUS** (power & solar tracking), **TEMPESTA** (weather station with 8 sensors), **IMPLUVIUM** (learning irrigation controller), and **STELLARIA** (adaptive LED lighting). All data flows through **TELEMETRY**, a caching system with two-tier MQTT buffering, while **HMI** provides a custom menu system on a small OLED screen.

Central to the system is IMPLUVIUM, an irrigation system that learns optimal watering through direct soil feedback combined with local environmental data from TEMPESTA for corrections. It measures actual moisture response, models soil redistribution, and adapts pump behavior per zone. The system also tracks solar panels for maximum power generation, monitors weather with 8 sensors, and manages a four-voltage-bus distribution system that intelligently sheds loads as battery drops.

## System Architecture

![solarium Diagram drawio](https://github.com/user-attachments/assets/f8a33487-a0ce-4170-98f9-2d25f3874cc6)

### Core system components

- #### IMPLUVIUM - Irrigation management with learning algorithm
Five-zone irrigation (expandable - currently in prototype phase) with embedded adaptive learning. Measures actual soil moisture response to determine optimal water volume per zone, learns soil redistribution characteristics through delayed readings, and adjusts pump duty targeting specific absorption rates. Integrates with TEMPESTA for temperature correction (±1%/°C). Confidence-weighted predictions with 3-tier safety monitoring (pre-checks, 500ms active monitoring, zone isolation diagnostics). Hardware: capacitive moisture sensors, ABP ΔP level sensor, rotary flow meter, pressure sensor. Learning data stored in RAM (15-cycle circular buffer), with daily midnight backup to FLASH memory.

- #### FLUCTUS - Power management, solar tracking, load shedding orchestration
Manages four voltage buses (3.3V/5V/6.6V/12V) with reference counting, dual INA219 power monitors (solar + battery), overcurrent protection, thermal management with PWM fan control for the MOSFET switching array, and dual-axis solar tracking with 15-minute correction cycles.

- #### TEMPESTA - Multi-sensor weather station with adaptive polling
Eight-sensor weather station: temperature and humidity (SHT40/BME280), pressure (BME280), wind speed/direction (AS5600/custom Hall array), air quality (PMS5003), rainfall and tank intake monitoring (custom pulse counters). Power-aware system adjusts the frequency of sensor polling.

- #### STELLARIA - Adaptive lighting with automatic dimming and toggling
LED lighting with PWM-controlled Meanwell LDD-600L driver. Three modes (Manual/Auto toggle/Power Save), photoresistor feedback with hysteresis, smooth fade ramping, auto-dimming under high power loads.

- #### TELEMETRY - Data cache handling and two-tier MQTT buffering system
Caching hub receiving push-based data injections from seven sources at variable frequencies (500ms to 60min). Implements two-tier MQTT buffering: PSRAM ring buffer (1,820 slots, volatile) with automatic FLASH backup (4,096 slots) at 95% capacity. Boot recovery restores unsent messages from FLASH. 

- #### HMI - Complete custom menu system
Custom UI system with SH1106 OLED screen and EC11 encoder. Framebuffer rendering, 47-state menu hierarchy with table-driven navigation, variable refresh rates.
<p align="center">
  <img src="https://github.com/user-attachments/assets/5b8e38bc-2f30-44e3-ab89-b3ed8657b49c" width="640">
</p>

### Additionally:
- **wifi_helper** - Power-aware WiFi management with exponential backoff and RSSI monitoring
- **ads1115_helper** - Unified ADC interface with automatic retry logic and fault recovery
- **interval_config** - Centralized interval management with persistent storage and preset profiles
- **solar_calc** - Sunrise/sunset calculations for detailed scheduling, time-based callbacks
- **mcp23008_helper** - I2C GPIO expander integration with software quadrature decoder, pulse counting, and power-gated outputs
### Custom drivers
- **abp** - Honeywell ABP ΔP sensor SPI driver library
- **as5600** - AS5600 Hall magnetic encoder I2C driver library, esp-idf-lib compatible

### Third-Party Components
- **Sensor drivers**: esp-idf-lib (SHT4x, BMP280, INA219, DS18B20, OneWire)
- **Serialization**: msgpack-c (library for MQTT payloads)
- **Filesystem**: joltwallet/esp_littlefs (wear-leveling FLASH filesystem for config and backup storage)
- **Framework**: ESP-IDF v5.4 (Espressif's official development framework)

### Original Work (~19,000 lines)

## General system's hardware layout (due to be updated)

![solarium_topo-1](https://github.com/user-attachments/assets/5801f9b7-c7d6-40f8-a188-8f4dde045b36)

## Technical Highlights

### Adaptive Learning Irrigation
IMPLUVIUM learns directly from soil response, using local sensor data rather than external weather services:

- **Embedded Learning**: Runs entirely on ESP32 - no external compute, no training datasets, no cloud dependency. Learns from actual watering outcomes.
- **PPMP Ratio Tracking**: Pre-computes pulses-per-moisture-percent for each zone with recency weighting (70% recent 3 cycles, 30% older). Clamped to hardware-safe bounds.
- **Soil Redistribution Factor**: Measures moisture 5 minutes after watering to model how water redistributes through soil (sandy 1.0 -> clay 3.0). Stops watering early, letting redistribution reach target.
- **2-Tier Confidence Blending**: Below 70% confidence -> linear interpolation between learned and defaults. Above 70% -> 100% learned predictions. Use of boost modifiers accelerating initial confidence convergence.
- **Ratio-Based Pump Adjustment**: Targets specific moisture gain rate. Adjusts pump duty for each cycle based on measured vs target absorption.
- **Temperature Correction**: ±1%/°C from 20°C baseline to account for evaporation differences.
- **Anomaly Detection**: Flags rain events, extreme temps, sensor failures - excludes from learning but keeps system functional.
- **Dual-Layer Storage**: 15-cycle history in RAM circular buffer for runtime learning. Daily midnight backup writes 5 most recent valid cycles to FLASH memory (LittleFS), restored on boot.

The algorithm adapts each zone independently through 5 phases: measure baseline → predict with confidence blending → water with learned pump duty → measure immediate response → verify after 5 minutes (update redistribution factor). All learning runs on-device with no external dependencies.

### Intelligent Power Management
FLUCTUS implements a reference-counted power distribution system with five-stage load shedding:

- **Bus-Level Gating**: 4 Individual buses powered only when needed, reducing idle consumption
- **Reference Counting**: Consumers request bus power by their name with reference counting, preventing premature shutdowns during shared use
- **Five Load States**: Normal -> Power Saving (40% SOC) -> Low Power (25%) -> Very Low (15%) -> Critical (0%)
- **Coordinated Shedding**: All components implement standardized power-save and shutdown APIs
- **Adaptive Sampling**: Power metering adjusts from 500ms active to 15s steady to night shutdown with sensor power-down

When battery drops below 40%, lighting dims. At 25%, lighting disables and weather monitoring slows. At 15%, irrigation stops. At critical levels, only minimal weather monitoring continues.

### Two-Tier "In-Flight Database"
The telemetry system implements a ring buffer architecture that minimizes data loss during failures. Each slot holds 32 bytes of metadata plus 256 bytes of msgpack-encoded payload. The system treats PSRAM as the active "database" and FLASH as the backup, moving data between tiers based on memory pressure.

- **PSRAM Ring Buffer**: 512KB total (~1.8k slots) circular buffer with head/tail pointers
- **Flash Backup**: 1.1MB total (~4k slots), automatically flushes at 95% PSRAM capacity
- **QoS 1 Integration**: Slots auto-dequeue on MQTT PUBACK, manual flush to FLASH before planned MCU reset via HMI
- **Smart Recovery**: On boot, restores PSRAM from FLASH backup then deletes backup file

### Direct-Write Telemetry Architecture
Hybrid push/pull eliminating polling and minimizing lock contention:

**Push-Based Injection**:
- Components write directly to TELEMETRY cache buffers (~1KB total cache)
- Variable update rates: 500ms (active monitoring) to 60min (idle states)
- HMI memcpy's snapshots for local rendering across multiple screens

### Custom HMI System

- **Custom Font**: 475-byte bitmap 5×7 font, full ASCII support
- **47 Menu States**: Hierarchical navigation across all subsystems with realtime and configuration screens
- **Table-Driven Navigation**: Declarative transition tables use
- **Variable Refresh Rates**: 4Hz for static screens, 8Hz for realtime data
- **Framebuffer Rendering**: 1024-byte buffer with efficient SPI flush to SH1106 128×64 OLED
- **EC11 Encoder**: Software quadrature decoder via MCP23008 I2C expander with button support

### Solar Tracking with Differential Photoresistors
FLUCTUS uses four photoresistors in a quadrant arrangement for dual-axis solar tracking via differential light sensing, applying proportional servo corrections every 3 seconds during 30s tracking cycles using 15-minute intervals. Simplified NOAA solar algorithm provides sunrise/sunset detection with 30-minute buffers for automatic wake/sleep transitions and prevents tracking attempts during night.

### Software Quadrature Decoder
The MCP23008 I2C GPIO expander replaces hardware PCNT with interrupt-driven software solutions:

- **Quadrature Decoding**: 16-entry lookup table handles encoder rotation with Gray code validation, filtering noise and invalid transitions
- **Pulse Counting**: Monotonic counters for rainfall (100mL/pulse) and tank monitoring
- **Event-Driven**: ISR -> task notification -> I2C reads (~200µs latency), no polling overhead

## Hardware

- **MCU**: ESP32-S3 16MB FLASH, 8MB PSRAM
- **Storage**: FLASH LittleFS partition for persistent config and backups
- **Display**: SH1106 128×64 OLED (SPI) with EC11 rotary encoder (via MCP23008)
- **Interfaces**: I2C, SPI, UART, GPIO, OneWire, PWM
- **Power**: 4-bus distribution (3.3V/5V/6.6V/12V) with solar input and 12V 14Ah battery
- 
<p align="center">
  <img src="https://github.com/user-attachments/assets/2388d66f-b94a-46c5-af34-02582b1dfcc6" width="640">
</p>

## Development Status

**Alpha version**
Single-node integrated system with all core features operational:
- Six main components fully implemented and tested
- Irrigation learning algorithm with soil redistribution modeling, confidence-weighted predictions, and persistent storage
- Two-tier MQTT buffering (PSRAM + Flash) with boot recovery
- 47-state custom HMI
- Day/night aware operations with NOAA solar calculations
- Five-stage load shedding with coordinated shutdown

**In Progress**:
- MQTT broker integration and topic structure finalization
- Field testing and learning algorithm tuning
- Long-term reliability validation

**Future (Beta)**:
Transition to distributed multi-node architecture with dedicated subsystems communicating via RS485/ESP-NOW, custom MPPT controller replacing external unit, and integration with Home Assistant + InfluxDB database backend for historical analysis.

## Documentation

Component documentation available in respective directories.


