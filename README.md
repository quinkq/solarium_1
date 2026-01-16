# Solarium - Solar-Powered Garden Automation System

## Status
v0.2-alpha - 01.2025
Dry testing complete. With most of the features tested, currently building hydraulic system (and compact enclosures to match), making field testing/deployment possible in the near future.

## Overview

Solarium is a self-contained garden automation platform running on a single ESP32-S3. It monitors weather conditions, waters plants based on learned patterns, tracks solar panels for maximum power generation, and manages a four-voltage-bus distribution system. When battery levels drop, it intelligently sheds non-essential loads to keep critical systems running.

The system is built around four main subsystems: **FLUCTUS** (power & solar tracking), **TEMPESTA** (weather station with 8 sensors), **IMPLUVIUM** (learning irrigation controller), and **STELLARIA** (adaptive LED lighting). All data flows through **TELEMETRY**, a caching system with two-tier MQTT buffering, while **HMI** provides a custom menu system on a small OLED screen.

## System Architecture

![solarium Diagram drawio](https://github.com/user-attachments/assets/f8a33487-a0ce-4170-98f9-2d25f3874cc6)

### Original Work (~19,000 lines)
All core system architecture and components were designed and implemented from scratch:

- #### FLUCTUS - Power management, solar tracking, load shedding orchestration
Manages four voltage buses (3.3V/5V/6.6V/12V) with reference counting, dual INA219 power monitors (solar + battery), overcurrent protection, thermal management with PWM fan control, and dual-axis solar tracking with 15-minute correction cycles.

- #### IMPLUVIUM - Irrigation management with learning algorithm
Five-zone irrigation with adaptive learning that tracks what works and adjusts water volume based on actual soil response. Uses moisture sensors (capacitive analog), ABP ΔP level sensor, rotary flow metering, pressure sensor and state machine with safety interlocks. Learns optimal water volume and pump speed per zone, with temperature correction that adjusts for evaporation. Persistent learning data (5 recent cycles per zone) stored in LittleFS.

- #### TEMPESTA - Multi-sensor weather station with adaptive polling
Eight-sensor weather station: temperature and humidity (SHT40/BME280), pressure (BME280), wind speed/direction (AS5600/custom Hall array), air quality (PMS5003), rainfall and tank intake monitoring (custom pulse counters). Power-aware polling switches between 15-minute and 60-minute cycles. Features 3 custom-built sensors based on Hall effect devices with intelligent power gating (14mA savings per cycle).

- #### STELLARIA - Adaptive lighting with automatic dimming and toggling
LED lighting with PWM-controlled Meanwell LDD-600L driver. Three modes (Manual/Auto toggle/Power Save), photoresistor feedback with hysteresis, smooth fade ramping, auto-dimming under high power loads.

- #### TELEMETRY - Zero-copy data cache and two-tier MQTT buffering system
Caching hub receiving push-based data injections from seven sources at variable frequencies (500ms to 60min). Implements two-tier MQTT buffering: PSRAM ring buffer (1,820 slots, volatile) with automatic FLASH backup (4,096 slots) at 95% capacity. Boot recovery restores unsent messages from flash. MessagePack serialization and simplified NOAA solar calculations for sunrise/sunset callbacks.

- #### HMI - Complete menu system with custom font rendering
Custom UI system with SH1106 OLED screen and EC11 encoder. Framebuffer rendering, 47-state menu hierarchy with table-driven navigation, variable refresh rates (1Hz static / 4Hz realtime), auto power-off with wake-on-input.


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
- **Filesystem**: joltwallet/esp_littlefs (wear-leveling flash filesystem for config and backup storage)
- **Framework**: ESP-IDF v5.4 (Espressif's official development framework)

![solarium_topo-1](https://github.com/user-attachments/assets/5801f9b7-c7d6-40f8-a188-8f4dde045b36)

## Technical Highlights

### Two-Tier "In-Flight Database"
The telemetry system implements a ring buffer architecture that ensures no data loss during power failures:

- **PSRAM Ring Buffer**: 512KB total (~1.8k slots) circular buffer with head/tail pointers
- **Flash Backup**: 1.1MB total (~4k slots), automatically flushes at 95% PSRAM capacity
- **QoS 1 Integration**: Slots auto-dequeue on MQTT PUBACK, manual flush to FLASH before planned MCU reset via HMI
- **Smart Recovery**: On boot, restores PSRAM from flash backup then deletes backup file

Each slot holds 32 bytes of metadata plus 256 bytes of msgpack-encoded payload. The system treats PSRAM as the active "database" and flash as the backup, moving data between tiers based on memory pressure.

### Adaptive Learning Irrigation
IMPLUVIUM waters on a schedule based on needs and learns what works:

- **Adaptive Algorithm**: Learns optimal watering through algorithms that track water volume -> moisture change correlation with recency weighting (70% recent, 30% older events)
- **15-Event History**: Tracks performance across multiple watering cycles per zone
- **3-Tier Confidence Blending**: High confidence (>70%) uses learned predictions, medium (40-70%) blends with defaults, low (<40%) favors safe defaults
- **Temperature Correction**: Adjusts volumes by ±1% per °C deviation from 20°C baseline to account for evaporation
- **Anomaly Detection**: Flags rain events, extreme temps, sensor failures - excludes from learning but keeps system functional
- **Persistent Storage**: Uses FLASH to save 5 most recent valid cycles per zone daily, loads them back in case of power failure

The algorithm adapts to each zone independently through 4 phases: measure baseline -> water with learned parameters -> measure final -> update learning. Learns optimal water volume, pump speed, and soil response rate while enforcing comprehensive safety bounds.

### Intelligent Power Management
FLUCTUS implements a reference-counted power distribution system with five-stage load shedding:

- **Bus-Level Gating**: Individual buses powered only when needed, reducing idle consumption
- **Reference Counting**: Components request bus power by name, preventing premature shutdowns during shared use
- **Five Load States**: Normal -> Power Saving (40% SOC) -> Low Power (25%) -> Very Low (15%) -> Critical (0%)
- **Coordinated Shedding**: All components implement standardized power-save and shutdown APIs
- **Adaptive Sampling**: Power metering adjusts from 500ms active to 15s steady to night shutdown with sensor power-down

When battery drops below 40%, lighting dims. At 25%, lighting disables and weather monitoring slows. At 15%, irrigation stops. At critical levels, only minimal weather monitoring continues.

### Direct-Write Telemetry Architecture
Hybrid push/pull eliminating polling and minimizing lock contention:

**Push-Based Injection**:
- Components write directly to TELEMETRY cache buffers (~1KB total cache)
- Each source has dedicated mutex - no lock contention between components
- Variable update rates: 500ms (active monitoring) to 60min (idle states)
- Seven injection points with automatic timestamping
- HMI memcpy's snapshots for local rendering across multiple screens
- Variable Refresh rates: 4Hz static screens / 8Hz realtime monitoring

### Custom HMI System
Built entirely from scratch without LVGL or U8g2:

- **Custom Font**: 475-byte bitmap 5×7 font, full ASCII support
- **47 Menu States**: Hierarchical navigation across all subsystems with realtime and configuration screens
- **Table-Driven Navigation**: Declarative transition tables use, instead of long switch statements
- **Variable Refresh Rates**: 4Hz for static screens, 8Hz for realtime data
- **Framebuffer Rendering**: 1024-byte buffer with efficient SPI flush to SH1106 128×64 OLED
- **EC11 Encoder**: Software quadrature decoder via MCP23008 I2C expander with button support

### Solar Tracking with Differential Photoresistors
FLUCTUS uses four photoresistors in a quadrant arrangement for dual-axis solar tracking via differential light sensing, applying proportional servo corrections every 3 seconds during 30s tracking cycles using 15-minute intervals. Simplified NOAA solar algorithm provides sunrise/sunset detection with 30-minute buffers for automatic wake/sleep transitions and prevents tracking attempts during night. Smart parking: east-facing at sunset, center position on errors. Servo bus powers up only during correction cycles for energy efficiency.

### Software Quadrature Decoder
The MCP23008 I2C GPIO expander replaces hardware PCNT with interrupt-driven software solutions:

- **Quadrature Decoding**: 16-entry lookup table handles encoder rotation with Gray code validation, filtering noise and invalid transitions
- **Pulse Counting**: Monotonic counters for rainfall (100mL/pulse) and tank monitoring
- **Event-Driven**: ISR -> task notification -> I2C reads (~200µs latency), no polling overhead

## Hardware

**MCU**: ESP32-S3 16MB flash, 8MB PSRAM
**Storage**: FLASH LittleFS partition for persistent config and backups
**Display**: SH1106 128×64 OLED (SPI) with EC11 rotary encoder (via MCP23008)
**Interfaces**: I2C, SPI, UART, GPIO, OneWire, PWM
**Power**: 4-bus distribution (3.3V/5V/6.6V/12V) with solar input and battery bank

![20251013_135107](https://github.com/user-attachments/assets/67a5d10f-e1c2-459f-9a77-ee2e89fb2cb5)

## Development Status

**Alpha Release**
- Single-node integrated system with all core features operational:
- Seven main components fully implemented and tested
- Irrigation learning algorithm with persistent storage and temperature correction
- Two-tier MQTT buffering (PSRAM + Flash) with boot recovery
- 47-state HMI with custom rendering and table-driven navigation
- Day/night aware operations with NOAA solar calculations
- Five-stage load shedding with coordinated shutdown
- Adaptive sensor polling (500ms to 60min depending on context)
- MCP23008 I/O expander integration complete

**In Progress**:
- MQTT broker integration and topic structure finalization
- Field testing and learning algorithm tuning
- Long-term reliability validation

**Future (Beta)**:
Transition to distributed multi-node architecture with dedicated subsystems communicating via RS485/ESP-NOW, custom MPPT controller replacing external unit, and integration with Home Assistant + InfluxDB database backend for historical analysis.

## Documentation

Component documentation available in respective directories.
