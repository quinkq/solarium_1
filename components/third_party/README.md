# Third-Party Components

This directory contains vendored third-party libraries used by the Solarium project. All components are maintained as local copies for build simplicity and stability.

> **Note:** Custom or heavily modified drivers are in `components/custom_drivers/`.

## Component Overview

### Unmodified Components (esp-idf-lib)

The following sensor drivers are sourced from [esp-idf-lib](https://github.com/UncleRus/esp-idf-lib):

| Component | Description | License | Version/Source |
|-----------|-------------|---------|----------------|
| **ads111x** | Driver for ADS1113/ADS1114/ADS1115 I2C ADC | BSD 3-Clause | [esp-idf-lib/ads111x](https://github.com/esp-idf-lib/ads111x) |
| **bmp280** | Driver for BMP280/BME280 digital pressure sensor | BSD 3-Clause | [esp-idf-lib/bmp280](https://github.com/esp-idf-lib/bmp280) |
| **ds18x20** | Driver for DS18B20/DS18S20 1-Wire temperature sensors | BSD 3-Clause | [esp-idf-lib/ds18x20](https://github.com/esp-idf-lib/ds18x20) |
| **ina219** | Driver for INA219/INA220 bidirectional current/power monitor | BSD 3-Clause | [esp-idf-lib/ina219](https://github.com/esp-idf-lib/ina219) |
| **onewire** | Bit-banging 1-Wire driver | BSD 3-Clause | [esp-idf-lib/onewire](https://github.com/esp-idf-lib/onewire) |
| **sht4x** | Driver for SHT40/SHT41/SHT45 temperature/humidity sensors | BSD 3-Clause | [esp-idf-lib/sht4x](https://github.com/esp-idf-lib/sht4x) |
| **esp_idf_lib_helpers** | Common helper utilities for esp-idf-lib components | BSD 3-Clause | [esp-idf-lib/esp_idf_lib_helpers](https://github.com/esp-idf-lib/esp_idf_lib_helpers) |

**License:** BSD 3-Clause (Ruslan V. Uss)
**Source:** Individual component repositories under [esp-idf-lib organization](https://github.com/esp-idf-lib)

> **Note:** These components are vendored unmodified. Original licenses are preserved in each component directory.

---

## msgpack
**Description:** MessagePack binary serialization library for MQTT telemetry
**License:** Boost Software License 1.0
**Source:** [msgpack/msgpack-c](https://github.com/msgpack/msgpack-c)
**Commit:** 306d59d52f75 (Nov 2024)

**Integration:**
- `msgpack-c/` - Complete msgpack-c library (vendored)
- `CMakeLists.txt` - ESP-IDF component wrapper (custom)
- Selected source files compiled: objectc.c, unpack.c, version.c, vrefbuffer.c, zone.c

---

## Dependency Management Strategy

### Why Vendored Locally?

This project uses vendored (locally copied) third-party dependencies instead of git submodules or the ESP Component Registry for the following reasons:

1. **Build Simplicity** - Simple `git clone` workflow with no submodule initialization required
2. **Build Reproducibility** - Exact versions locked in repository, no network dependencies
3. **Rare Updates** - These are stable sensor drivers that rarely need updates
4. **No Modifications** - Unmodified external code kept separate from custom drivers

### Exception: LittleFS

The **LittleFS** filesystem component is managed via ESP-IDF Component Manager (`idf_component.yml`):
```yaml
joltwallet/littlefs: ==1.20.1
```

This is intentional as LittleFS is:
- Well-maintained with regular updates
- Used unmodified
- Cleanly packaged for ESP-IDF component manager

---

## Updating Components

If you need to update an esp-idf-lib component:

1. Visit the upstream repository (links in table above)
2. Download or clone the latest version
3. Replace the local directory
4. Test thoroughly
5. Document the new version/commit in this README

---

## License Compliance

All components retain their original license files and copyright notices:
- **BSD 3-Clause:** esp-idf-lib components (ads111x, bmp280, ds18x20, ina219, onewire, sht4x, esp_idf_lib_helpers)
- **Boost 1.0:** msgpack-c

License files are preserved in each component directory. This project complies with all upstream license terms.

---

## Component Usage in Project

| Component | Used By | Purpose |
|-----------|---------|---------|
| **ads111x** | ads1115_helper | 4× ADC devices (moisture, photoresistors, hall array) |
| **bmp280** | TEMPESTA | Atmospheric pressure sensor |
| **ds18x20** | FLUCTUS | Temperature monitoring + thermal management |
| **ina219** | FLUCTUS | Dual power monitoring (solar PV + battery) |
| **onewire** | FLUCTUS | 1-Wire bus for DS18B20 |
| **sht4x** | TEMPESTA | Temperature/humidity sensor |
| **msgpack** | TELEMETRY | Binary serialization for MQTT |
| **esp_idf_lib_helpers** | All esp-idf-lib | Common utilities |

For detailed hardware integration, see the main project `CLAUDE.md`.

---

**Last Updated:** 2026-01-06
**Project:** Solarium ESP32-S3 Smart Irrigation & Solar Management System
