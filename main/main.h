#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <inttypes.h>

// ########################## Global Device Definitions ##########################

// ########################## I2C Settings ##########################
#define I2C_PORT_NUM I2C_NUM_0

// ########################## SPI Settings ##########################
// SPI2 bus shared by: ABP pressure sensor, HMI display
#define SPI2_MOSI_PIN GPIO_NUM_11
#define SPI2_MISO_PIN GPIO_NUM_10
#define SPI2_SCLK_PIN GPIO_NUM_9

// ########################## FUNCTION DECLARATIONS ################################


#endif // MAIN_H