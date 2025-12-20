#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <inttypes.h>

// ########################## Global Device Definitions ##########################

// ########################## I2C Settings ##########################
// Bus A (I2C_NUM_0): Always-powered devices (INA219, MCP23008)
// - Power source: Uninterrupted 3.3V (Bus A)
// - Pull-ups: Must be on Bus A 3.3V rail
#define I2C_BUS_A_PORT         I2C_NUM_0
#define I2C_BUS_A_SDA_PIN      GPIO_NUM_1
#define I2C_BUS_A_SCL_PIN      GPIO_NUM_2

// Bus B (I2C_NUM_1): Toggled-power devices (ADS1115, SHT4x, BMP280, AS5600)
// - Power source: Toggled 3.3V (Bus B, switched from Bus A)
// - Pull-ups: Must be on Bus B 3.3V rail
#define I2C_BUS_B_PORT         I2C_NUM_1
#define I2C_BUS_B_SDA_PIN      GPIO_NUM_15
#define I2C_BUS_B_SCL_PIN      GPIO_NUM_16

// ########################## SPI Settings ##########################
// SPI2 bus shared by: ABP pressure sensor, HMI display
#define SPI2_MOSI_PIN GPIO_NUM_11
#define SPI2_MISO_PIN GPIO_NUM_10
#define SPI2_SCLK_PIN GPIO_NUM_9

// ########################## FUNCTION DECLARATIONS ################################


#endif // MAIN_H