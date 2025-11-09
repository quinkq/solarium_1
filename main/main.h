#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ########################## Global Device Definitions ##########################

// ########################## I2C Settings ##########################
#define I2C_PORT_NUM I2C_NUM_0

// ########################## SPI Settings ##########################
// SPI2 bus shared by: ABP pressure sensor, HMI display
#define SPI2_MOSI_PIN 9
#define SPI2_MISO_PIN 10
#define SPI2_SCLK_PIN 11

// ########################## Debug display settings ##########################
// Debuging display variables
#define SERIAL_DISPLAY_INTERVAL_MS 2000 // Update display every 2 seconds

extern SemaphoreHandle_t xDisplayDataMutex;

extern float latest_sht_temp;
extern float latest_sht_hum;
extern float latest_bmp_temp;
extern float latest_bmp_press;
extern float latest_bmp_hum; // If BME280
extern float latest_as5600_angle;
extern uint16_t latest_as5600_raw;

// ########################## FUNCTION DECLARATIONS ################################


#endif // MAIN_H