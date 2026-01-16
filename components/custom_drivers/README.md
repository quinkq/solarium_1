# Custom Drivers

Custom and heavily modified drivers for the Solarium project.

## Components

| Component | Type | Description |
|-----------|------|-------------|
| **i2cdev** | Custom rewrite | I2C abstraction layer for ESP-IDF |
| **mcp23008** | Modified | MCP23008 GPIO expander with interrupt register access (see MODIFICATIONS.md) |
| **abp** | Custom | Honeywell ABP SPI pressure sensor driver |
| **as5600** | Custom | AS5600 I2C magnetic rotary encoder for wind speed |

---

**Note:** Unmodified third-party libraries are in `components/third_party/`.
