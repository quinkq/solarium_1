# Honeywell ABP Pressure Sensor Driver

ESP-IDF driver for Honeywell ABP (Basic) series pressure sensors with SPI interface.

## Supported Sensors

- **ABPDJJT001PDSA3** - ±1 psi differential pressure (target sensor)
- **ABPDJJT005PDSA3** - ±5 psi differential pressure  
- **ABPDJJT015PDSA3** - ±15 psi differential pressure
- **ABPDJJT030PDSA3** - ±30 psi differential pressure
- **ABPDJJT060PDSA3** - ±60 psi differential pressure

## Features

- **SPI Interface**: High-speed communication up to 800 kHz
- **14-bit Resolution**: Accurate pressure measurements
- **Multiple Units**: PSI, mbar, and Pascal output
- **Status Monitoring**: Built-in sensor health diagnostics
- **Range Support**: Multiple pressure ranges with automatic calibration
- **Thread Safe**: FreeRTOS compatible with proper error handling

## Hardware Connection

### SPI Pinout (ABPDJJT001PDSA3)

```
Pin 1 (GND)   -> Ground
Pin 2 (Vsupply) -> 3.3V (3.3V ONLY - do not use 5V!)
Pin 3 (SS/CS) -> CS Pin (configurable)
Pin 4 (NC)    -> Not connected  
Pin 5 (MISO)  -> SPI MISO
Pin 6 (SCLK)  -> SPI SCLK
```

### Wiring Example (ESP32)

```
ABP Sensor    ESP32
-----------   -----
Pin 1 (GND)   -> GND
Pin 2 (Vdd)   -> 3.3V
Pin 3 (SS)    -> GPIO5 (or your chosen CS pin)
Pin 5 (MISO)  -> GPIO19
Pin 6 (SCLK)  -> GPIO18
```

## Usage

### 1. Initialize SPI Bus

```c
#include "abp.h"

// Use the convenience function (MOSI not needed for read-only sensor)
esp_err_t ret = abp_spi_bus_init(SPI2_HOST, GPIO_NUM_19, GPIO_NUM_18);
if (ret != ESP_OK) {
    ESP_LOGE("APP", "Failed to initialize SPI bus");
    return;
}

// Alternatively, you can initialize manually:
// spi_bus_config_t bus_config = {
//     .miso_io_num = GPIO_NUM_19,
//     .mosi_io_num = -1,            // Not used for read-only sensor
//     .sclk_io_num = GPIO_NUM_18,
//     .quadwp_io_num = -1,
//     .quadhd_io_num = -1,
//     .max_transfer_sz = 64,
// };
// esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
```

### 2. Initialize Sensor

```c
abp_t sensor = {0};

// Initialize for ±1 psi differential pressure
esp_err_t ret = abp_init(&sensor, SPI2_HOST, GPIO_NUM_5, ABP_RANGE_001PD);
if (ret != ESP_OK) {
    ESP_LOGE("APP", "Failed to initialize ABP sensor");
    return;
}
```

### 3. Read Pressure

```c
abp_data_t data;

esp_err_t ret = abp_read_pressure(&sensor, &data);
if (ret == ESP_OK && data.valid) {
    printf("Pressure: %.3f psi\n", data.pressure_psi);
    printf("Pressure: %.1f mbar\n", data.pressure_mbar);
    printf("Pressure: %.0f Pa\n", data.pressure_pa);
} else {
    printf("Failed to read pressure\n");
}
```

### 4. Cleanup

```c
abp_deinit(&sensor);
```

## API Reference

### Types

#### `abp_pressure_range_t`

Supported pressure ranges:

- `ABP_RANGE_001PD` - ±1 psi differential
- `ABP_RANGE_005PD` - ±5 psi differential  
- `ABP_RANGE_015PD` - ±15 psi differential
- `ABP_RANGE_030PD` - ±30 psi differential
- `ABP_RANGE_060PD` - ±60 psi differential

#### `abp_status_t`

Sensor status values:

- `ABP_STATUS_NORMAL` - Normal operation
- `ABP_STATUS_COMMAND_MODE` - Command mode (not used)
- `ABP_STATUS_STALE_DATA` - Stale data warning
- `ABP_STATUS_DIAGNOSTIC_FAULT` - Sensor fault

#### `abp_data_t`

Measurement data structure:

```c
typedef struct {
    float pressure_psi;      // Pressure in PSI
    float pressure_mbar;     // Pressure in mbar  
    float pressure_pa;       // Pressure in Pascal
    abp_status_t status;     // Sensor status
    uint32_t raw_pressure;   // Raw 14-bit count
    bool valid;              // Data validity flag
} abp_data_t;
```

### Functions

#### `abp_init()`

Initialize ABP sensor with SPI configuration.

#### `abp_deinit()`

Deinitialize and cleanup sensor resources.

#### `abp_read_pressure()`

Read calibrated pressure data in multiple units.

#### `abp_read_raw()`

Read raw 14-bit pressure count and status.

#### `abp_convert_pressure()`

Convert raw count to engineering units (PSI).

### Utility Functions

#### `abp_psi_to_mbar()` / `abp_psi_to_pascal()`

Convert PSI to other pressure units.

#### `abp_mbar_to_psi()`

Convert mbar to PSI.

## Integration Example

See `abp_example.c` for complete integration examples including:

- Continuous monitoring task
- Error handling patterns
- Application integration points

## Specifications

| Parameter | Value |
|-----------|--------|
| Supply Voltage | 3.3V (±10%) |
| Interface | SPI Mode 0 |
| Clock Speed | Up to 800 kHz |
| Resolution | 14-bit (16384 counts) |
| Response Time | < 1ms |
| Accuracy | ±1.5% FSS (Total Error Band) |
| Temperature Range | 0°C to 50°C |

## Troubleshooting

### Common Issues

1. **No Response from Sensor**
   - Check 3.3V power supply
   - Verify SPI wiring (especially MISO and SCLK)
   - Ensure CS pin is correctly configured

2. **Invalid/Stale Data**
   - Add delay between reads (>1ms)
   - Check for loose connections
   - Verify sensor power stability

3. **Incorrect Pressure Readings**
   - Ensure correct pressure range is selected
   - Check sensor part number matches configuration
   - Verify differential pressure connections

### Debug Tips

Enable debug logging:

```c
esp_log_level_set("abp", ESP_LOG_DEBUG);
```

This will show raw SPI data and conversion details.

## References

- [Honeywell ABP Series Datasheet](https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/en-us/products/sensors/pressure-sensors/board-mount-pressure-sensors/basic-abp-series/documents/sps-siot-basic-board-mount-pressure-abp-series-datasheet-32305128-ciid-155789.pdf)
- [ESP-IDF SPI Master Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)

## License

This driver is provided under the MIT License. See individual source files for license details.
