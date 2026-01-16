# mcp23008 - Local Modifications

**Source:** [esp-idf-lib/mcp23008](https://github.com/esp-idf-lib/mcp23008)
**License:** BSD 3-Clause (Ruslan V. Uss)

## Modifications

Added two functions to read interrupt-related registers:

```c
esp_err_t mcp23008_port_get_interrupt_flags(i2c_dev_t *dev, uint8_t *val);
esp_err_t mcp23008_port_get_interrupt_capture(i2c_dev_t *dev, uint8_t *val);
```

These additions expose INTF and INTCAP registers, enabling proper interrupt handling. Used by `mcp23008_helper` component for interrupt-driven operation.
